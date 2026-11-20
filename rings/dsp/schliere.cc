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

const uint8_t kTapsL[kSchliereTaps] = { 1, 9, 17, 25, 33, 41, 49 };
const uint8_t kTapsR[kSchliereTaps] = { 1, 11, 21, 31, 41, 51, 61 };

// Halfband coefficients (Valenzuela/Constantinides, elliptic, transition
// 0.045): flat to 20 kHz, at least 77 dB stopband from 28 kHz.
const float kHalfbandCoefficients[6] = {
  0.06396838f, 0.22734146f, 0.42965512f,
  0.62193724f, 0.78589107f, 0.92904445f
};

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
inline float Stage(float x, float* s, float a1, float a2, float g, float c) {
  float v3 = x - s[1];
  float v1 = a1 * s[0] + a2 * v3;
  float v2 = s[1] + g * v1;
  s[0] = 2.0f * v1 - s[0];
  s[1] = 2.0f * v2 - s[1];
  return x - c * v1;
}

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
  float t = Clamp((fraction - kHold) / (1.0f - 2.0f * kHold), 0.0f, 1.0f);
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

float SchliereHalfband::Branch(float x, int32_t first) {
  for (int32_t i = first; i < kNumCoefficients; i += 2) {
    float y = kHalfbandCoefficients[i] * (x - y_[i]) + x_[i];
    x_[i] = x;
    y_[i] = y;
    x = y;
  }
  return x;
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

  rng_state_ = 0x21436587;
  limiter_blocks_ = 0;
  tap_detent_ = 0;
  primed_ = false;
  Reset();
}

void Schliere::Reset() {
  for (size_t i = 0; i < 2 * kSchliereStages; ++i) {
    state_[i] = 0.0f;
  }
  for (size_t i = 0; i <= kSchliereStages; ++i) {
    node_[i] = 0.0f;
  }
  up_in_.Init();
  up_cv_.Init();
  up_loop_.Init();
  down_.Init();
  down_odd_ = 0.0f;
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

float Schliere::FeedbackAmount(float knob_plus_cv) const {
  return kFeedbackMax * (switches_.hard_feedback_limit
      ? Clamp(knob_plus_cv, -1.0f, 1.0f)
      : Soft(knob_plus_cv, kFeedbackKnee, 1.0f));
}

// Chain input at 48 kHz: input and feedback loop.
float Schliere::ChainInput(float in, float cv) {
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
float Schliere::ChainInputOversampled(float in, float cv) {
  float in_2x[2] = { up_in_.Branch(in, 0), up_in_.Branch(in, 1) };
  float cv_2x[2] = { up_cv_.Branch(cv, 0), up_cv_.Branch(cv, 1) };
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

float Schliere::Excitation() {
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
  return IsBroken(node_[kSchliereStages]) ||
      IsBroken(loop_state_) ||
      IsBroken(loop_lp_);
}

void Schliere::Process(
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
    feedback_lp_[0] = feedback_lp_[1] = FeedbackAmount(p.feedback);
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
  float inverse_size = 1.0f / static_cast<float>(size);
  g_step_ = (g_target - g_) * inverse_size;
  k_step_ = (k_target - k_) * inverse_size;

  // CVs arrive once per block: ramp them over the block.
  float tap_cv_step = (p.tap_cv - tap_cv_) * inverse_size;
  float feedback_cv_step = (p.feedback_cv - feedback_cv_) * inverse_size;

  if (p.excite) {
    if (p.excite_type == SCHLIERE_EXCITE_IMPULSE) {
      impulse_pending_ = true;
    } else {
      burst_level_ = 1.0f;
    }
  }

  loop_peak_ = 0.0f;
  for (size_t i = 0; i < size; ++i) {
    g_ += g_step_;
    k_ += k_step_;
    float g = g_;  // local copy: keeps the stage loop free of reloads
    float a1 = 1.0f / (1.0f + g * (g + k_));
    float a2 = g * a1;
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
    float x = switches_.oversampled_feedback
        ? ChainInputOversampled(in[i], feedback_cv)
        : ChainInput(in[i], feedback_cv);
    x += Excitation();

    // The chain.
    node_[0] = x;
    float* s = state_;
    for (size_t n = 1; n <= kSchliereStages; ++n, s += 2) {
      x = Stage(x, s, a1, a2, g, c);
      node_[n] = x;
    }

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
    for (int32_t j = 0; j < static_cast<int32_t>(kSchliereTaps); ++j) {
      float w = j == index ? 1.0f - t : (j == index + 1 ? t : 0.0f);
      if (switches_.smooth_tap_weights) {
        if (first) {
          tap_lp_[j][0] = tap_lp_[j][1] = w;
        }
        w = Lowpass(Lowpass(w, &tap_lp_[j][0], tap_weight_g_),
                    &tap_lp_[j][1], tap_weight_g_);
      }
      left += w * node_[kTapsL[j]];
      right_classic += w * node_[kTapsR[j]];
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
          node_[kSchliereStages] - right_classic);
    }

    // Feedback loop: highpass, soft limit.
    loop_lp_ += (left - loop_lp_) * loop_hp_;
    float loop = left - loop_lp_;
    loop_state_ = Soft(loop, kLoopKnee, kLoopCeiling);
    if (fabsf(loop) > loop_peak_) {
      loop_peak_ = fabsf(loop);
    }

    mix_ += (p.mix - mix_) * slew_sample_;
    float dry = (1.0f - mix_) * in[i];
    out_l[i] = Soft(dry + mix_ * left, kOutKnee, kOutCeiling);
    out_r[i] = Soft(dry + mix_ * right, kOutKnee, kOutCeiling);
  }

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
