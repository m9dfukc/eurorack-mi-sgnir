// Copyright 2026 m9dfukc.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// See http://creativecommons.org/licenses/MIT/ for more information.
//
// -----------------------------------------------------------------------------
//
// Schliere: a long chain of second-order allpass filters with feedback.

#include "rings/dsp/schliere.h"

// The Rings makefile builds with -fno-move-loop-invariants; for the chain
// loop that rematerializes constants in every stage.
// No automatic fused multiply-add: on the Cortex-M4 vfma takes 3 cycles and
// stalls the next dependent operation, vmul + vadd take 2 (cycle model: 72 ->
// 65 % of the block budget). Rounding differs by at most 1.5e-6. The one
// explicit fused operation in Stage() is deliberate (see there).
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC optimize("move-loop-invariants")
#pragma GCC optimize("fp-contract=off")
#endif

// Forced inlining for the helpers of the per-sample code.
#if defined(__GNUC__)
#define SCHLIERE_INLINE inline __attribute__((always_inline))
#else
#define SCHLIERE_INLINE inline
#endif

#include <cmath>

#include "stmlib/dsp/dsp.h"

#include "rings/dsp/dsp.h"

namespace rings {

using namespace stmlib;

namespace {

const float kPi = 3.14159265f;
const float kLn2 = 0.69314718f;

const float kFrequencyCenter = 632.4555f;  // knob at noon: sqrt(20 * 20000) Hz
const float kFrequencyLnRange = 7.6009025f;  // ln(2000): half an octave margin
const float kFrequencyMin = 20.0f;
const float kFrequencyMax = 20000.0f;

const float kFeedbackMax = 0.97f;  // only reachable with CV
const float kFeedbackKnee = 0.9f;
const float kHold = 0.15f;  // hold zone per side, fraction of the detent spacing
// Reciprocal of the crossfade width: multiplied per sample instead of a
// division (rounding differs by at most 1.2e-6).
const float kCrossfadeScale = 1.0f / (1.0f - 2.0f * kHold);
const float kSmoothTime = 0.010f;  // knob smoothing, 10 ms
const float kTapWeightHz = 3000.0f;  // per pole (two poles)
const float kLoopHighpassHz = 10.0f;
const float kLoopKnee = 0.5f;
const float kLoopCeiling = 1.0f;
const float kOutKnee = 0.75f;
const float kOutCeiling = 0.98f;
const float kLimiterHoldTime = 0.3f;
const float kDetentHysteresis = 0.03f;

const float kExciteLevel = 0.5f;  // [hypothesis] tune by ear on hardware
const float kBurstTime = 0.003f;
const float kBurstLowpassHz = 4000.0f;

const float kGateTime = 0.005f;  // fade out/in around a chain change

const uint32_t kAllTaps = (1u << kSchliereTaps) - 1;

inline float Clamp(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

// Linear up to the knee, then a soft knee with asymptote at the ceiling.
// SoftClip() replaces tanh() of the OWL version (max. deviation 0.024).
inline float Soft(float x, float knee, float ceiling) {
  float a = fabsf(x);
  if (a <= knee) {
    return x;
  }
  float range = ceiling - knee;
  float y = knee + range * SoftClip((a - knee) / range);
  return x < 0.0f ? -y : y;
}

// First-order trapezoidal lowpass with a zero at fs/2. G = g / (1 + g).
inline float Lowpass(float x, float* s, float G) {
  float v = (x - *s) * G;
  float y = v + *s;
  *s = y + v;
  return y;
}

inline float LowpassCoefficient(float hz, float sample_rate) {
  float g = tanf(kPi * hz / sample_rate);
  return g / (1.0f + g);
}

// One stage: trapezoidal state variable filter, allpass = x - c * bandpass.
// g2 = 2 g: the second integrator update 2 (s1 + g v1) - s1 is folded into
// s1 + 2 g v1 (algebraically identical, one operation less).
// The output update is one fused multiply-add (vfms): slower in the cycle
// model, but 4 bytes shorter per stage. The unrolled chain is limited by
// instruction fetch from flash, and on the module the shorter stage wins:
// 59 instead of 61 % (48 kHz), 69 instead of 72 % (96 kHz) [measured, diag7,
// flash log 2026-11-21]. Rounding differs by at most 1.2e-6.
SCHLIERE_INLINE float Stage(
    float x, float* s, float a1, float a2, float g2, float c) {
  float v1 = a1 * s[0] + a2 * (x - s[1]);
  s[0] = 2.0f * v1 - s[0];
  s[1] += g2 * v1;
  return __builtin_fmaf(-c, v1, x);
}

// Stages [from, to) of the chain. Called with constant bounds only.
SCHLIERE_INLINE float Stages(
    float x, float* s, size_t from, size_t to,
    float a1, float a2, float g2, float c) {
  for (size_t n = from; n < to; ++n) {
    x = Stage(x, s + 2 * n, a1, a2, g2, c);
  }
  return x;
}

// Chain layouts. Each one has its length, its taps and its Chain(): the
// stages, storing only the nodes read by the taps (union of the L and R taps
// and the chain end). Chain() is written out with constant segment ends
// instead of looping over a node table, so that GCC unrolls the chain
// completely (cycle model: 18.5 instead of 29 cycles per stage). The host
// tests check every tap of every layout against the textbook allpass.

// S: 61 stages, the layout of the OWL patch.
struct LayoutS {
  static const size_t kStages = 61;
  static const bool kOversampling = true;
  static const uint8_t kTapsL[kSchliereTaps];
  static const uint8_t kTapsR[kSchliereTaps];
  static SCHLIERE_INLINE void Chain(
      float x, float* s, float* node, float a1, float a2, float g2, float c) {
    node[0] = x;
    node[1] = x = Stages(x, s, 0, 1, a1, a2, g2, c);
    node[9] = x = Stages(x, s, 1, 9, a1, a2, g2, c);
    node[11] = x = Stages(x, s, 9, 11, a1, a2, g2, c);
    node[17] = x = Stages(x, s, 11, 17, a1, a2, g2, c);
    node[21] = x = Stages(x, s, 17, 21, a1, a2, g2, c);
    node[25] = x = Stages(x, s, 21, 25, a1, a2, g2, c);
    node[31] = x = Stages(x, s, 25, 31, a1, a2, g2, c);
    node[33] = x = Stages(x, s, 31, 33, a1, a2, g2, c);
    node[41] = x = Stages(x, s, 33, 41, a1, a2, g2, c);
    node[49] = x = Stages(x, s, 41, 49, a1, a2, g2, c);
    node[51] = x = Stages(x, s, 49, 51, a1, a2, g2, c);
    node[61] = Stages(x, s, 51, 61, a1, a2, g2, c);
  }
};
const uint8_t LayoutS::kTapsL[kSchliereTaps] = { 1, 9, 17, 25, 33, 41, 49 };
const uint8_t LayoutS::kTapsR[kSchliereTaps] = { 1, 11, 21, 31, 41, 51, 61 };

// L: 98 stages (196 poles), taps 13 and 16-17 stages apart (about 4 : 5).
struct LayoutL {
  static const size_t kStages = 98;
  static const bool kOversampling = false;
  static const uint8_t kTapsL[kSchliereTaps];
  static const uint8_t kTapsR[kSchliereTaps];
  static SCHLIERE_INLINE void Chain(
      float x, float* s, float* node, float a1, float a2, float g2, float c) {
    node[0] = x;
    node[1] = x = Stages(x, s, 0, 1, a1, a2, g2, c);
    node[14] = x = Stages(x, s, 1, 14, a1, a2, g2, c);
    node[17] = x = Stages(x, s, 14, 17, a1, a2, g2, c);
    node[27] = x = Stages(x, s, 17, 27, a1, a2, g2, c);
    node[33] = x = Stages(x, s, 27, 33, a1, a2, g2, c);
    node[40] = x = Stages(x, s, 33, 40, a1, a2, g2, c);
    node[49] = x = Stages(x, s, 40, 49, a1, a2, g2, c);
    node[53] = x = Stages(x, s, 49, 53, a1, a2, g2, c);
    node[66] = x = Stages(x, s, 53, 66, a1, a2, g2, c);
    node[79] = x = Stages(x, s, 66, 79, a1, a2, g2, c);
    node[82] = x = Stages(x, s, 79, 82, a1, a2, g2, c);
    node[98] = Stages(x, s, 82, 98, a1, a2, g2, c);
  }
};
const uint8_t LayoutL::kTapsL[kSchliereTaps] = { 1, 14, 27, 40, 53, 66, 79 };
const uint8_t LayoutL::kTapsR[kSchliereTaps] = { 1, 17, 33, 49, 66, 82, 98 };

static_assert(LayoutL::kStages == kSchliereStagesL &&
              LayoutS::kStages == kSchliereStages,
              "chain lengths do not match schliere.h");

// Resonance knob -> ln(Q): straight lines between Q 0.01 / 0.026 / 0.51 /
// 3.81 / 16 at 0, 25, 50, 75, 100 %.
inline float LnQ(float knob) {
  static const float table[5] = {
    -4.6051702f, -3.6496587f, -0.6733446f, 1.3376292f, 2.7725887f
  };
  float u = Clamp(knob, 0.0f, 1.0f) * 4.0f;
  int32_t i = u >= 4.0f ? 3 : static_cast<int32_t>(u);
  return table[i] + (table[i + 1] - table[i]) * (u - i);
}

// Crossfade position between two detents: hold zone, then linear or S-curve.
inline float Crossfade(float fraction, bool linear) {
  float t = Clamp((fraction - kHold) * kCrossfadeScale, 0.0f, 1.0f);
  return linear ? t : t * t * (3.0f - 2.0f * t);
}

// Inf or NaN, checked on the bits (immune to -ffast-math).
inline bool IsBroken(float x) {
  union { float f; uint32_t u; } v;
  v.f = x;
  return (v.u & 0x7f800000u) == 0x7f800000u;
}

}  // namespace

void SchliereHalfband::Init() {
  for (int32_t i = 0; i < kNumCoefficients; ++i) {
    x_[i] = y_[i] = 0.0f;
  }
}

void Schliere::Init(SchliereVariant variant) {
  switch (variant) {
    case SCHLIERE_VARIANT_S:
    case SCHLIERE_VARIANT_O:
      switches_.linear_crossfade = false;
      switches_.smooth_tap_weights = true;
      switches_.hard_feedback_limit = false;
      switches_.feedback_smoothing_hz = 0.0f;
      switches_.feedback_cv_smoothing_hz =
          variant == SCHLIERE_VARIANT_S ? 12000.0f : 0.0f;
      switches_.oversampled_feedback = true;
      break;

    case SCHLIERE_VARIANT_Z:
    case SCHLIERE_VARIANT_W:
      switches_.linear_crossfade = false;
      switches_.smooth_tap_weights = true;
      switches_.hard_feedback_limit = false;
      switches_.feedback_smoothing_hz =
          variant == SCHLIERE_VARIANT_Z ? 1500.0f : 12000.0f;
      switches_.feedback_cv_smoothing_hz = 0.0f;
      switches_.oversampled_feedback = false;
      break;

    case SCHLIERE_VARIANT_A:
      switches_.linear_crossfade = true;
      switches_.smooth_tap_weights = false;
      switches_.hard_feedback_limit = true;
      switches_.feedback_smoothing_hz = 0.0f;
      switches_.feedback_cv_smoothing_hz = 0.0f;
      switches_.oversampled_feedback = false;
      break;
  }

  loop_hp_ = 1.0f - expf(-2.0f * kPi * kLoopHighpassHz / kSampleRate);
  tap_weight_g_ = LowpassCoefficient(kTapWeightHz, kSampleRate);
  feedback_g_ = switches_.feedback_smoothing_hz > 0.0f
      ? LowpassCoefficient(switches_.feedback_smoothing_hz, kSampleRate)
      : 0.0f;
  feedback_cv_g_ = switches_.feedback_cv_smoothing_hz > 0.0f
      ? LowpassCoefficient(switches_.feedback_cv_smoothing_hz, kSampleRate)
      : 0.0f;
  slew_sample_ = 1.0f - expf(-1.0f / (kSmoothTime * kSampleRate));
  slew_block_ = 0.0f;
  slew_block_size_ = 0;
  burst_decay_ = expf(-1.0f / (kBurstTime * kSampleRate));
  burst_lp_coefficient_ = 1.0f - expf(
      -2.0f * kPi * kBurstLowpassHz / kSampleRate);

  tap_active_ = kAllTaps;
  rng_state_ = 0x21436587;
  limiter_blocks_ = 0;
  tap_detent_ = 0;
  primed_ = false;
  chain_ = pending_chain_ = SCHLIERE_CHAIN_S;
  gate_ = 1.0f;
  Reset();
}

void Schliere::Reset() {
  for (size_t i = 0; i < 2 * kSchliereStagesL; ++i) {
    state_[i] = 0.0f;
  }
  chain_end_ = 0.0f;
  up_in_.Init();
  up_loop_.Init();
  down_.Init();
  down_odd_ = 0.0f;
  cv_previous_ = 0.0f;
  loop_lp_ = 0.0f;
  loop_state_ = 0.0f;
  loop_peak_ = 0.0f;
  feedback_now_ = 0.0f;
  burst_level_ = 0.0f;
  burst_lp_ = 0.0f;
  impulse_pending_ = false;
}

void Schliere::ComputeCoefficients(
    float knob_f, float knob_q, float octaves, float* g, float* k) const {
  float f = kFrequencyCenter * expf(
      kFrequencyLnRange * (knob_f - 0.5f) + kLn2 * octaves);
  f = Clamp(f, kFrequencyMin, kFrequencyMax);
  *g = tanf(kPi * f / kSampleRate);
  *k = expf(-LnQ(knob_q));
}

// FeedbackAmount(), ChainInput() and ChainInputOversampled() are forced
// inline: they run once per sample, and inlined, the halfband loops unroll.
SCHLIERE_INLINE float Schliere::FeedbackAmount(float knob_plus_cv) const {
  return kFeedbackMax * (switches_.hard_feedback_limit
      ? Clamp(knob_plus_cv, -1.0f, 1.0f)
      : Soft(knob_plus_cv, kFeedbackKnee, 1.0f));
}

// Chain input at 48 kHz: input and feedback loop.
SCHLIERE_INLINE float Schliere::ChainInput(float in, float cv) {
  float fb = FeedbackAmount(feedback_ + cv);
  if (feedback_g_ > 0.0f) {
    fb = Lowpass(Lowpass(fb, &feedback_lp_[0], feedback_g_),
                 &feedback_lp_[1], feedback_g_);
  }
  feedback_now_ = fb;
  return sqrtf(1.0f - fb * fb) * in + fb * loop_state_;
}

// Chain input at 96 kHz: input, CV and loop are upsampled, the feedback
// product is formed there, then the result is decimated. Sum frequencies
// above 24 kHz then fall into the decimation filter instead of folding back.
SCHLIERE_INLINE float Schliere::ChainInputOversampled(float in, float cv) {
  float in_2x[2] = { up_in_.Branch(in, 0), up_in_.Branch(in, 1) };
  // The CV only moves at block rate (ramped): linear interpolation is enough
  // instead of a third halfband. (The OWL version upsampled an audio-rate
  // CV here.)
  float cv_2x[2] = { 0.5f * (cv_previous_ + cv), cv };
  cv_previous_ = cv;
  float loop_2x[2] = {
    up_loop_.Branch(loop_state_, 0), up_loop_.Branch(loop_state_, 1)
  };
  float x[2];
  float fb = 0.0f;
  for (int32_t h = 0; h < 2; ++h) {
    fb = FeedbackAmount(feedback_ + cv_2x[h]);
    x[h] = sqrtf(1.0f - fb * fb) * in_2x[h] + fb * loop_2x[h];
  }
  feedback_now_ = fb;
  float y = 0.5f * (down_.Branch(x[0], 0) + down_.Branch(down_odd_, 1));
  down_odd_ = x[1];
  return y;
}

SCHLIERE_RAM_CODE float Schliere::Excitation() {
  float e = 0.0f;
  if (impulse_pending_) {
    e += kExciteLevel;
    impulse_pending_ = false;
  }
  if (burst_level_ > 1.0e-4f) {
    rng_state_ = rng_state_ * 1664525u + 1013904223u;
    float noise = static_cast<float>(rng_state_ >> 8) / 8388608.0f - 1.0f;
    burst_lp_ += burst_lp_coefficient_ * (noise * burst_level_ - burst_lp_);
    burst_level_ *= burst_decay_;
    e += kExciteLevel * burst_lp_;
  } else {
    burst_lp_ = 0.0f;
  }
  return e;
}

void Schliere::UpdateTapDetent() {
  float u = tap_ * (kSchliereTaps - 1);
  if (tap_detent_ == 0 ||
      fabsf(u - static_cast<float>(tap_detent_ - 1)) >
          kHold + kDetentHysteresis) {
    int32_t nearest = static_cast<int32_t>(u + 0.5f);
    tap_detent_ = fabsf(u - static_cast<float>(nearest)) <=
        kHold - kDetentHysteresis ? nearest + 1 : 0;
  }
}

bool Schliere::Broken() const {
  return IsBroken(chain_end_) ||
      IsBroken(loop_state_) ||
      IsBroken(loop_lp_);
}

// The sample loop. Forced inline into Process(): one copy per layout, each
// with its chain unrolled and its tap table and chain end as constants.
template<typename Layout>
SCHLIERE_INLINE void Schliere::Run(
    const SchliereParameters& p,
    const float* in,
    float* out_l,
    float* out_r,
    size_t size,
    Block* b) {
  float a1 = b->a1;
  float a2 = b->a2;
  const float a1_step = b->a1_step;
  const float a2_step = b->a2_step;
  const float tap_cv_step = b->tap_cv_step;
  const float feedback_cv_step = b->feedback_cv_step;
  bool first = b->first;
  const bool exciting = b->exciting;
  const bool oversampled =
      Layout::kOversampling && switches_.oversampled_feedback;

  // Taps with a non-zero smoothed weight; the others are skipped.
  uint32_t tap_active = b->tap_active;
  const bool smooth_tap_weights = switches_.smooth_tap_weights;

  // Nodes of the current sample; Chain() writes only those the taps read.
  float node[Layout::kStages + 1];
  node[Layout::kStages] = chain_end_;  // read below even when size is 0

  for (size_t i = 0; i < size; ++i) {
    g_ += g_step_;
    k_ += k_step_;
    float g2 = 2.0f * g_;  // local copy: keeps the stage loop free of reloads
    float c = 2.0f * k_;

    tap_cv_ += tap_cv_step;
    feedback_cv_ += feedback_cv_step;
    float feedback_cv = feedback_cv_;
    if (feedback_cv_g_ > 0.0f) {
      feedback_cv = Lowpass(
          Lowpass(feedback_cv, &feedback_cv_lp_[0], feedback_cv_g_),
          &feedback_cv_lp_[1], feedback_cv_g_);
    }

    feedback_ += (p.feedback - feedback_) * slew_sample_;
    float x = oversampled
        ? ChainInputOversampled(in[i], feedback_cv)
        : ChainInput(in[i], feedback_cv);
    if (exciting) {
      x += Excitation();
    }

    Layout::Chain(x, state_, node, a1, a2, g2, c);
    a1 += a1_step;
    a2 += a2_step;

    // Taps: hold zone around each detent, crossfade in between. The weights
    // run through two lowpasses, so the crossfade stays band-limited.
    tap_ += (p.tap - tap_) * slew_sample_;
    float u = Clamp(tap_ + tap_cv_, 0.0f, 1.0f) * (kSchliereTaps - 1);
    int32_t index = u >= static_cast<float>(kSchliereTaps - 1)
        ? kSchliereTaps - 2
        : static_cast<int32_t>(u);
    float t = Crossfade(u - index, switches_.linear_crossfade);
    float left = 0.0f;
    float right_classic = 0.0f;
    // Visit only the two taps under the knob and those whose weight still
    // decays, in ascending order (same summation order as a full loop).
    uint32_t pending = smooth_tap_weights
        ? tap_active | (3u << index)
        : 3u << index;
    while (pending) {
      int32_t j = __builtin_ctz(pending);
      pending &= pending - 1;
      float w = j == index ? 1.0f - t : (j == index + 1 ? t : 0.0f);
      if (smooth_tap_weights) {
        float* lp = tap_lp_[j];
        if (first) {
          lp[0] = lp[1] = w;
        }
        if (w == 0.0f && lp[0] == 0.0f && lp[1] == 0.0f) {
          tap_active &= ~(1u << j);
          continue;  // inactive tap, settled
        }
        tap_active |= 1u << j;
        w = Lowpass(Lowpass(w, &lp[0], tap_weight_g_), &lp[1], tap_weight_g_);
        if (fabsf(lp[0]) < 1.0e-7f && fabsf(lp[1]) < 1.0e-7f &&
            fabsf(w) < 1.0e-7f) {
          lp[0] = lp[1] = 0.0f;  // flush the decayed tail
        }
      } else if (w == 0.0f) {
        continue;
      }
      left += w * node[Layout::kTapsL[j]];
      right_classic += w * node[Layout::kTapsR[j]];
    }
    first = false;

    // Spread: mono (R = L) -> classic layout -> R at the chain end, with
    // hold zones at the three detents.
    spread_ += (p.spread - spread_) * slew_sample_;
    float v = Clamp(spread_, 0.0f, 1.0f) * 2.0f;
    float right;
    if (v < 1.0f) {
      float t_spread = Crossfade(v, false);
      right = left + t_spread * (right_classic - left);
    } else {
      float t_spread = Crossfade(v - 1.0f, false);
      right = right_classic + t_spread * (
          node[Layout::kStages] - right_classic);
    }

    // Feedback loop: highpass, soft limit.
    loop_lp_ += (left - loop_lp_) * loop_hp_;
    float loop = left - loop_lp_;
    loop_state_ = Soft(loop, kLoopKnee, kLoopCeiling);
    if (fabsf(loop) > loop_peak_) {
      loop_peak_ = fabsf(loop);
    }

    // The gate is 1 except around a chain change.
    mix_ += (p.mix - mix_) * slew_sample_;
    // A member, not a local: it is only needed after the chain, and as a
    // local it would occupy an FPU register across the chain (spills).
    gate_ += gate_step_;
    float dry = (1.0f - mix_) * in[i];
    float wet = mix_ * gate_;
    out_l[i] = Soft(dry + wet * left, kOutKnee, kOutCeiling);
    out_r[i] = Soft(dry + wet * right, kOutKnee, kOutCeiling);
  }
  b->tap_active = tap_active;
  chain_end_ = node[Layout::kStages];
}

SCHLIERE_RAM_CODE void Schliere::Process(
    const SchliereParameters& p,
    const float* in,
    float* out_l,
    float* out_r,
    size_t size) {
  if (size != slew_block_size_) {
    slew_block_size_ = size;
    slew_block_ = 1.0f - expf(-static_cast<float>(size) /
        (kSmoothTime * kSampleRate));
  }

  bool first = !primed_;
  if (first) {
    frequency_ = p.frequency;
    resonance_ = p.resonance;
    tap_ = p.tap;
    feedback_ = p.feedback;
    spread_ = p.spread;
    mix_ = p.mix;
    tap_cv_ = p.tap_cv;
    feedback_cv_ = p.feedback_cv;
    feedback_cv_lp_[0] = feedback_cv_lp_[1] = p.feedback_cv;
    cv_previous_ = p.feedback_cv;
    feedback_lp_[0] = feedback_lp_[1] = FeedbackAmount(p.feedback);
  }

  // Chain change: the wet signal fades out, then the chain restarts silent
  // with the new length, then the wet signal fades back in.
  if (pending_chain_ != chain_ && gate_ <= 0.0f) {
    chain_ = pending_chain_;
    Reset();
  }
  float inverse_size = 1.0f / static_cast<float>(size);
  // Linear ramp; the last step snaps to 0 or 1 (rounding would otherwise
  // leave a sliver and add a block).
  float gate_move = static_cast<float>(size) / (kGateTime * kSampleRate);
  float gate_end;
  if (pending_chain_ != chain_) {
    gate_end = gate_ - gate_move;
    if (gate_end < 0.5f * gate_move) {
      gate_end = 0.0f;
    }
  } else {
    gate_end = gate_ + gate_move;
    if (gate_end > 1.0f - 0.5f * gate_move) {
      gate_end = 1.0f;
    }
  }

  // Frequency and resonance: smoothed per block, coefficients ramped
  // linearly over the block.
  frequency_ += (p.frequency - frequency_) * slew_block_;
  resonance_ += (p.resonance - resonance_) * slew_block_;
  float g_target, k_target;
  ComputeCoefficients(frequency_, resonance_, p.octaves, &g_target, &k_target);
  if (first) {
    g_ = g_target;
    k_ = k_target;
    primed_ = true;
  }
  g_step_ = (g_target - g_) * inverse_size;
  k_step_ = (k_target - k_) * inverse_size;

  // The stage coefficients a1 = 1 / (1 + g (g + k)), a2 = g a1 are ramped
  // linearly over the block instead of dividing per sample. Exact at the
  // block boundaries; identical to the per-sample form when nothing moves.
  float g_end = g_ + g_step_ * static_cast<float>(size);
  float k_end = k_ + k_step_ * static_cast<float>(size);
  float a1_start = 1.0f / (1.0f + (g_ + g_step_) * ((g_ + g_step_) +
                                                    (k_ + k_step_)));
  float a1_end = 1.0f / (1.0f + g_end * (g_end + k_end));
  float a2_start = (g_ + g_step_) * a1_start;
  float a2_end = g_end * a1_end;
  Block b;
  b.a1 = a1_start;
  b.a2 = a2_start;
  b.a1_step = b.a2_step = 0.0f;
  if (size > 1) {
    b.a1_step = (a1_end - a1_start) / static_cast<float>(size - 1);
    b.a2_step = (a2_end - a2_start) / static_cast<float>(size - 1);
  }

  // CVs arrive once per block: ramp them over the block.
  b.tap_cv_step = (p.tap_cv - tap_cv_) * inverse_size;
  b.feedback_cv_step = (p.feedback_cv - feedback_cv_) * inverse_size;

  gate_step_ = (gate_end - gate_) * inverse_size;

  if (p.excite) {
    if (p.excite_type == SCHLIERE_EXCITE_IMPULSE) {
      impulse_pending_ = true;
    } else {
      burst_level_ = 1.0f;
    }
  }
  // An excitation can only start here, at a block boundary: when none is
  // pending or still decaying, skip Excitation() for the whole block.
  b.exciting = impulse_pending_ || burst_level_ > 1.0e-4f;
  if (!b.exciting) {
    burst_lp_ = 0.0f;
  }

  b.tap_active = first ? kAllTaps : tap_active_;
  b.first = first;

  loop_peak_ = 0.0f;
  switch (chain_) {
    case SCHLIERE_CHAIN_L:
      Run<LayoutL>(p, in, out_l, out_r, size, &b);
      break;
    default:
      Run<LayoutS>(p, in, out_l, out_r, size, &b);
      break;
  }
  tap_active_ = b.tap_active;
  gate_ = gate_end;

  // Garbage in the state (should never happen): silence and start over.
  if (Broken()) {
    Reset();
    for (size_t i = 0; i < size; ++i) {
      out_l[i] = out_r[i] = 0.0f;
    }
  }

  UpdateTapDetent();
  if (loop_peak_ > kLoopKnee && fabsf(feedback_now_) > 1.0e-4f) {
    limiter_blocks_ = static_cast<int32_t>(
        kLimiterHoldTime * kSampleRate / static_cast<float>(size));
  } else if (limiter_blocks_ > 0) {
    --limiter_blocks_;
  }
}

}  // namespace rings
