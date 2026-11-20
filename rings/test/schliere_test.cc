// Host tests for rings/dsp/schliere.cc.
//
// Ported from the OWL reference tests (acdc-workbench, schliere_test.cpp)
// where they apply to Rings (variants S, O, Z; two outputs; block size 24;
// CVs at block rate), plus tests for the Rings additions (spread, dry/wet
// mix, excite, detent display). Transfer functions are measured from impulse
// responses and checked against the textbook allpass and the Smear manual.
//
// Build and run from the repository root:
//   c++ -std=c++11 -O2 -DTEST -I. -o schliere_test \
//       rings/test/schliere_test.cc rings/dsp/schliere.cc && ./schliere_test

#include <cmath>
#include <complex>
#include <cstdio>
#include <vector>

#include "rings/dsp/schliere.h"

using namespace rings;

static int fails;

static void check(const char* name, bool ok, double a, double b = 0) {
  printf("%-4s %-70s %10.5f %10.5f\n", ok ? "ok" : "FAIL", name, a, b);
  if (!ok) fails++;
}

static const double PI = 3.14159265358979, FS = 48000;
static const size_t kBlock = 24;
typedef std::complex<double> cplx;

struct Out {
  std::vector<float> l, r;
};

// Knob settings in the units of the reference tests.
static double FrequencyKnob(double hz) {
  return 0.5 + std::log(hz / 632.4555) / std::log(2000.0);
}
static double TapKnob(double detent) { return (detent - 1.0) / 6.0; }

// Knob + CV position that gives the feedback amount fb (inverse of the soft
// knee with SoftClip, by bisection).
static double SoftClipRef(double x) {
  if (x > 3.0) return 1.0;
  return x * (27.0 + x * x) / (27.0 + 9.0 * x * x);
}
static double FeedbackKnob(double fb) {
  const double v = fb / 0.97;
  if (v <= 0.9) return v;
  double lo = 0.0, hi = 3.0;
  for (int i = 0; i < 60; ++i) {
    const double mid = 0.5 * (lo + hi);
    (0.9 + 0.1 * SoftClipRef(mid) < v ? lo : hi) = mid;
  }
  return 0.9 + 0.1 * lo;
}

static SchliereParameters Settings(double hz, double q, double detent,
                                   double fb) {
  SchliereParameters p;
  p.frequency = static_cast<float>(FrequencyKnob(hz));
  p.octaves = 0.0f;
  p.resonance = static_cast<float>(q);
  p.tap = static_cast<float>(TapKnob(detent));
  p.tap_cv = 0.0f;
  p.feedback = static_cast<float>(FeedbackKnob(fb));
  p.feedback_cv = 0.0f;
  p.spread = 0.5f;
  p.mix = 1.0f;
  p.excite = false;
  p.excite_type = SCHLIERE_EXCITE_IMPULSE;
  return p;
}

// Runs n samples (rounded up to whole blocks). The callback may change the
// parameters before each block (t = first sample of the block).
template<typename Input, typename Control>
static Out Run(Schliere* s, SchliereParameters p, size_t n, Input in,
               Control control) {
  Out o;
  float buffer[kBlock], l[kBlock], r[kBlock];
  for (size_t t = 0; t < n; t += kBlock) {
    control(&p, t);
    for (size_t i = 0; i < kBlock; ++i) {
      buffer[i] = static_cast<float>(in(t + i));
    }
    s->Process(p, buffer, l, r, kBlock);
    o.l.insert(o.l.end(), l, l + kBlock);
    o.r.insert(o.r.end(), r, r + kBlock);
  }
  return o;
}

template<typename Input>
static Out Run(Schliere* s, const SchliereParameters& p, size_t n, Input in) {
  return Run(s, p, n, in, [](SchliereParameters*, size_t) { });
}

static const size_t T0 = 72;  // time of the impulse

static Out Impulse(SchliereVariant variant, const SchliereParameters& p,
                   size_t n, double amp) {
  static Schliere s;
  s.Init(variant);
  return Run(&s, p, T0 + n, [&](size_t t) { return t == T0 ? amp : 0.0; });
}

static cplx Response(const std::vector<float>& y, double f, double amp) {
  cplx acc = 0;
  const cplx step = std::polar(1.0, -2 * PI * f / FS);
  cplx w = 1;
  for (size_t i = T0; i < y.size(); ++i) {
    acc += static_cast<double>(y[i]) * w;
    w *= step;
  }
  return acc / amp;
}

static double Db(double x) { return 20 * std::log10(x + 1e-30); }

static double GroupDelay(const std::vector<float>& y, double f, double amp) {
  const double df = 0.1;
  double d = std::arg(Response(y, f + df, amp) / Response(y, f - df, amp));
  return -d / (2 * PI * 2 * df);
}

// Second-order allpass, textbook formula (bilinear with prewarping).
static cplx Reference(double f, double f0, double q, int stages) {
  const double w0 = 2 * PI * f0 / FS, al = std::sin(w0) / (2 * q);
  const cplx z = std::polar(1.0, -2 * PI * f / FS);
  const cplx h = ((1 - al) - 2 * std::cos(w0) * z + (1 + al) * z * z) /
      ((1 + al) - 2 * std::cos(w0) * z + (1 - al) * z * z);
  return std::pow(h, stages);
}

static double Peak(const std::vector<float>& v, size_t from = 0,
                   size_t to = 0) {
  double m = 0;
  if (to == 0) to = v.size();
  for (size_t i = from; i < to; ++i) m = std::fmax(m, std::fabs(v[i]));
  return m;
}

static bool Finite(const Out& o) {
  for (float x : o.l) if (!std::isfinite(x)) return false;
  for (float x : o.r) if (!std::isfinite(x)) return false;
  return true;
}

static int Notches(const std::vector<float>& y, double amp, double limit_db) {
  int count = 0;
  const int points = 6000;
  std::vector<double> m(points);
  for (int i = 0; i < points; ++i) {
    m[i] = Db(std::abs(Response(
        y, 20 * std::pow(1000.0, static_cast<double>(i) / (points - 1)), amp)));
  }
  for (int i = 1; i + 1 < points; ++i) {
    if (m[i] < m[i - 1] && m[i] <= m[i + 1] && m[i] < limit_db) count++;
  }
  return count;
}

static unsigned rng = 1;
static double Noise() {
  rng = rng * 1664525u + 1013904223u;
  return (rng >> 8) / 8388608.0 - 1.0;
}

// Line power at f (Blackman-Harris window over n samples from `from`).
static double Line(const std::vector<float>& y, size_t from, size_t n,
                   double f, double fs) {
  cplx acc = 0;
  for (size_t i = 0; i < n; ++i) {
    const double x = static_cast<double>(i) / (n - 1);
    const double w = 0.35875 - 0.48829 * std::cos(2 * PI * x) +
        0.14128 * std::cos(4 * PI * x) - 0.01168 * std::cos(6 * PI * x);
    acc += static_cast<double>(y[from + i]) * w *
        std::polar(1.0, -2 * PI * f * (from + i) / fs);
  }
  return std::abs(acc);
}

int main() {
  const double A = 0.25;
  const SchliereVariant Z = SCHLIERE_VARIANT_Z;
  const SchliereVariant S = SCHLIERE_VARIANT_S;

  // ---- The chain (shared by all variants; Z has no oversampling delay) ----
  {
    const double knobs[5] = { 0, 0.25, 0.5, 0.75, 1 };
    const double qs[5] = { 0.01, 0.026, 0.51, 3.81, 16 };
    double err = 0;
    for (int i = 0; i < 5; ++i) {
      Out o = Impulse(Z, Settings(1000, knobs[i], 1, 0), 96000, A);
      for (double f : { 300.0, 900.0, 1000.0, 1150.0, 4000.0 }) {
        err = std::fmax(err, std::abs(Response(o.l, f, A) -
                                      Reference(f, 1000, qs[i], 1)));
      }
    }
    check("one stage matches the textbook allpass for Q 0.01 ... 16", err < 2e-3, err);
  }
  {
    Out o = Impulse(Z, Settings(1000, 0.5, 7, 0), 16384, A);
    double dev = 0;
    for (double f : { 30.0, 200.0, 1000.0, 3000.0, 15000.0 }) {
      dev = std::fmax(dev, std::fabs(Db(std::abs(Response(o.l, f, A)))));
      dev = std::fmax(dev, std::fabs(Db(std::abs(Response(o.r, f, A)))));
    }
    check("top tap, no feedback: level flat at L and R (dB)", dev < 0.01, dev);
    double err = std::abs(Response(o.l, 700, A) - Reference(700, 1000, 0.51, 49)) +
        std::abs(Response(o.r, 700, A) - Reference(700, 1000, 0.51, 61));
    check("  L = 49 stages (98 poles), R = 61 stages (122 poles) at spread 0.5", err < 5e-3, err);
  }
  {
    SchliereParameters p = Settings(1000, 1, 7, 0);
    p.spread = 1.0f;
    Out o = Impulse(Z, p, 4 * 48000, A);
    double gd = GroupDelay(o.l, 1000, A), tail = GroupDelay(o.r, 1000, A);
    check("98 poles at 1 kHz, Q 16: group delay 0.50 s (Smear manual)", std::fabs(gd - 0.501) < 0.005, gd, 0.501);
    check("  R at spread 1 = chain end, 122 poles: 0.50 s * 61 / 49", std::fabs(tail - 0.501 * 61 / 49) < 0.007, tail, 0.501 * 61 / 49);
  }
  {
    Out o = Impulse(Z, Settings(400, 1, 7, 0), 10 * 48000, A);
    double gd = GroupDelay(o.l, 400, A);
    check("98 poles at 400 Hz, Q 16: group delay 1.25 s (Smear manual)", std::fabs(gd - 1.249) < 0.012, gd, 1.249);
  }
  {
    static Schliere s;
    s.Init(Z);
    Out o = Run(&s, Settings(1000, 0.5, 4.5, 0), T0 + 8192,
                [&](size_t t) { return t == T0 ? A : 0.0; });
    int l = Notches(o.l, A, -25), r = Notches(o.r, A, -25);
    check("halfway between taps 4 and 5: 8 notches at L (Smear manual)", l == 8, l, 8);
    check("  10 notches at R (taps 20 poles apart)", r == 10, r, 10);
    check("  detent display off between two taps", s.tap_detent() == 0, s.tap_detent());
  }
  {
    static Schliere s;
    s.Init(Z);
    Out o = Run(&s, Settings(1000, 0.5, 1, 0), 48000,
                [](size_t) { return 0.4 * Noise(); });
    double diff = 0;
    for (size_t i = 0; i < o.l.size(); ++i) diff = std::fmax(diff, std::fabs(o.l[i] - o.r[i]));
    check("bottom tap: L and R identical", diff == 0, diff);
  }
  {
    SchliereParameters lo = Settings(1000, 0.5, 1, 0);
    lo.frequency = 0.0f;
    SchliereParameters hi = Settings(1000, 0.5, 1, 0);
    hi.frequency = 1.0f;
    Out a = Impulse(Z, lo, 96000, A);
    Out b = Impulse(Z, hi, 4096, A);
    double e1 = std::abs(Response(a.l, 20, A) + 1.0);
    double e2 = std::abs(Response(b.l, 20000, A) + 1.0);
    check("frequency knob at the stops: 20 Hz and 20 kHz", e1 < 0.01 && e2 < 0.01, e1, e2);
  }
  {
    SchliereParameters p = Settings(1000, 0.5, 1, 0);
    p.octaves = 1.0f;
    Out o = Impulse(Z, p, 4096, A);
    double e = std::abs(Response(o.l, 2000, A) + 1.0);
    check("octaves +1 (V/OCT +1 V) moves 1 kHz to 2 kHz", e < 0.01, e);
  }
  {
    static Schliere s;
    s.Init(Z);
    SchliereParameters p = Settings(1000, 0.5, 1, 0);
    p.tap_cv = 4.0f / 6.0f;
    Out o = Run(&s, p, T0 + 8192, [&](size_t t) { return t == T0 ? A : 0.0; });
    double err = std::abs(Response(o.l, 700, A) - Reference(700, 1000, 0.51, 33));
    check("tap CV reaches tap 5 (66 poles) with the knob at 0", err < 5e-3, err);
    check("  detent display follows the knob, not the CV: detent 1", s.tap_detent() == 1, s.tap_detent());
  }
  {
    // Tap CV swept across detents at block rate: the detent display must not
    // switch (it follows the knob only).
    static Schliere s;
    s.Init(Z);
    int changes = 0, last = -1;
    Run(&s, Settings(1000, 0.5, 4, 0), 48000, [](size_t) { return 0.0; },
        [&](SchliereParameters* p, size_t t) {
          p->tap_cv = static_cast<float>(0.6 * std::sin(2 * PI * 7 * t / FS));
          if (t == 0) return;  // no block processed yet
          if (last >= 0 && s.tap_detent() != last) changes++;
          last = s.tap_detent();
        });
    check("tap CV swept across detents: detent display never switches", changes == 0, changes);
    // Knob slowly across the edge of the hold zone and back, 0.5 % jitter.
    static Schliere q;
    q.Init(Z);
    changes = 0;
    last = -1;
    Run(&q, Settings(1000, 0.5, 4, 0), 4000 * kBlock, [](size_t) { return 0.0; },
        [&](SchliereParameters* p, size_t t) {
          const double step = static_cast<double>(t / kBlock);
          const double pos = 4 + 0.15 * std::sin(PI * step / 4000.0) +
              0.005 * ((static_cast<int>(step) * 7919) % 3 - 1);
          p->tap = static_cast<float>(TapKnob(pos));
          if (last >= 0 && q.tap_detent() != last) changes++;
          last = q.tap_detent();
        });
    check("  knob jittering at the edge of the hold zone: switches at most twice", changes <= 2, changes);
  }
  {
    static Schliere s;
    s.Init(Z);
    Out o = Run(&s, Settings(1000, 0.5, 7, 0), 48000, [](size_t) { return 0.2; });
    double e = std::fmax(std::fabs(o.l.back() - 0.2), std::fabs(o.r.back() - 0.2));
    check("DC passes at unity (forward path is DC coupled)", e < 1e-4, e);
  }
  {
    static Schliere s;
    s.Init(Z);
    Out o = Run(&s, Settings(1000, 0.5, 7, 0.9), 4 * 48000, [](size_t) { return 0.2; });
    const double want = 0.2 * std::sqrt(1 - 0.81);
    check("feedback 90 %: DC is not amplified (0.2 * sqrt(1 - fb^2))", std::fabs(o.l.back() - want) < 1e-3, o.l.back(), want);
  }
  {
    const double a = 0.001, want = Db(std::sqrt(1.9 / 0.1));
    Out o = Impulse(Z, Settings(1000, 0.5, 7, 0.9), 4 * 48000, a);
    double hi = -1e9, lo = 1e9, f_hi = 0;
    for (double f = 900; f < 1100; f += 0.25) {
      double m = Db(std::abs(Response(o.l, f, a)));
      if (m > hi) { hi = m; f_hi = f; }
      lo = std::fmin(lo, m);
    }
    check("feedback 90 %: peaks at +12.8 dB", std::fabs(hi - want) < 0.3, hi, want);
    check("  troughs at -12.8 dB", std::fabs(lo + want) < 0.3, lo, -want);
    SchliereParameters n = Settings(1000, 0.5, 7, 0);
    n.feedback_cv = static_cast<float>(-FeedbackKnob(0.9));
    Out on = Impulse(Z, n, 4 * 48000, a);
    double swapped = Db(std::abs(Response(on.l, f_hi, a)));
    check("  negative feedback by CV: the peak becomes a trough", std::fabs(swapped + want) < 0.5, swapped, -want);
  }
  for (SchliereVariant v : { Z, S }) {
    const char* label = v == S ? "S" : "Z";
    char name[128];
    {
      static Schliere s;
      s.Init(v);
      SchliereParameters p = Settings(200, 1, 7, 0.97);
      Out o = Run(&s, p, 10 * 48000, [](size_t t) {
        return 0.6 * ((t / 218) % 2 ? 1.0 : -1.0) + 0.3 * Noise();
      });
      double m = std::fmax(Peak(o.l), Peak(o.r));
      snprintf(name, sizeof(name), "%s: hot square + noise, feedback and Q at max: below 0.98", label);
      check(name, Finite(o) && m <= 0.98f, m, 0.98);
      check("  limiter in the loop reported", s.limiter_active(), s.limiter_active());
    }
    {
      static Schliere s;
      s.Init(v);
      Out o = Run(&s, Settings(1000, 0.5, 7, 0.97), 14 * 48000, [](size_t t) {
        return t < 48000 ? 0.5 * std::sin(2 * PI * 997 * t / FS) : 0.0;
      });
      double early = Peak(o.l, 48000, 2 * 48000), late = Peak(o.l, 13 * 48000);
      snprintf(name, sizeof(name), "%s: feedback at maximum rings out, no self-oscillation (dB)", label);
      check(name, Db(late / early) < -60, Db(late / early), -60);
    }
    {
      // Block-rate modulation of all parameters (Rings reads CVs per block).
      static Schliere s;
      s.Init(v);
      Out o = Run(&s, Settings(800, 0.8, 4, 0.5), 5 * 48000,
                  [](size_t) { return 0.5 * Noise(); },
                  [](SchliereParameters* p, size_t t) {
                    const double x = 2 * PI * t / FS;
                    p->octaves = static_cast<float>(3.0 * std::sin(130 * x));
                    p->tap_cv = static_cast<float>(std::sin(90 * x));
                    p->feedback_cv = static_cast<float>(std::sin(210 * x));
                    p->spread = static_cast<float>(0.5 + 0.5 * std::sin(70 * x));
                    p->resonance = static_cast<float>(0.5 + 0.5 * std::sin(50 * x));
                  });
      double m = std::fmax(Peak(o.l), Peak(o.r));
      snprintf(name, sizeof(name), "%s: all parameters modulated at block rate: finite, bounded", label);
      check(name, Finite(o) && m <= 0.98f, m, 0.98);
    }
  }
  {
    static Schliere s;
    s.Init(Z);
    Out o = Run(&s, Settings(500, 1, 7, 0), 3 * 48000,
                [](size_t t) { return 0.25 * std::sin(2 * PI * 500 * t / FS); },
                [](SchliereParameters* p, size_t t) {
                  p->octaves = (t / 4800) % 2 ? 2.0f : -2.0f;
                });
    check("frequency jumping by 4 octaves at Q 16 while ringing: no level burst", Finite(o) && Peak(o.l) < 0.75, Peak(o.l), 0.75);
  }

  // ---- Oversampled chain input (S, O) ----
  {
    double image = -1e9, alias = -1e9;
    for (double f : { 1000.0, 10000.0, 20000.0 }) {
      SchliereHalfband up;
      up.Init();
      std::vector<float> y;
      for (int i = 0; i < 48000; ++i) {
        float x = static_cast<float>(std::sin(2 * PI * f * i / FS));
        y.push_back(up.Branch(x, 0));
        y.push_back(up.Branch(x, 1));
      }
      image = std::fmax(image, Db(Line(y, 48000, 48000, 48000 - f, 96000) /
                                  Line(y, 48000, 48000, f, 96000)));
    }
    for (double f : { 30000.0, 40000.0 }) {
      SchliereHalfband down;
      down.Init();
      std::vector<float> y, ref;
      float odd = 0.0f;
      for (int i = 0; i < 48000; ++i) {
        float a = static_cast<float>(std::sin(2 * PI * f * 2 * i / 96000));
        float b = static_cast<float>(std::sin(2 * PI * f * (2 * i + 1) / 96000));
        y.push_back(0.5f * (down.Branch(a, 0) + down.Branch(odd, 1)));
        odd = b;
        ref.push_back(static_cast<float>(std::sin(2 * PI * (48000 - f) * i / FS)));
      }
      alias = std::fmax(alias, Db(Line(y, 24000, 24000, 48000 - f, FS) /
                                  Line(ref, 24000, 24000, 48000 - f, FS)));
    }
    check("halfband: images when going up to 96 kHz (dB)", image < -75, image, -75);
    check("halfband: folding when going down from 30/40 kHz (dB)", alias < -75, alias, -75);
  }
  {
    Out a = Impulse(S, Settings(1000, 0.5, 7, 0), 16384, A);
    Out b = Impulse(Z, Settings(1000, 0.5, 7, 0), 16384, A);
    double dev = 0;
    for (double f : { 30.0, 200.0, 1000.0, 3000.0, 15000.0 }) {
      dev = std::fmax(dev, std::fabs(Db(std::abs(Response(a.l, f, A)))));
    }
    check("S: top tap, no feedback: level flat at L (dB)", dev < 0.01, dev);
    const double extra = GroupDelay(a.l, 1000, A) - GroupDelay(b.l, 1000, A);
    check("  extra delay at 1 kHz: 2.8 samples (two halfbands)", std::fabs(extra * FS - 2.8) < 0.3, extra * FS, 2.8);
  }
  {
    Out a = Impulse(S, Settings(1000, 0.5, 4.5, 0.5), 16384, A);
    Out b = Impulse(SCHLIERE_VARIANT_O, Settings(1000, 0.5, 4.5, 0.5), 16384, A);
    double d = 0;
    for (size_t i = 0; i < a.l.size(); ++i) {
      d = std::fmax(d, std::fabs(a.l[i] - b.l[i]));
      d = std::fmax(d, std::fabs(a.r[i] - b.r[i]));
    }
    check("S: without feedback CV identical to O", d == 0, d);
  }
  {
    static Schliere s;
    s.Init(S);
    Out o = Run(&s, Settings(1000, 0.5, 7, 0), 48000, [](size_t) { return 0.2; });
    check("S: DC passes at unity", std::fabs(o.l.back() - 0.2) < 1e-4, o.l.back(), 0.2);
  }

  // ---- Rings additions ----
  {
    static Schliere s;
    s.Init(S);
    SchliereParameters p = Settings(1000, 0.5, 5.3, 0.4);
    p.spread = 0.0f;
    Out o = Run(&s, p, 48000, [](size_t) { return 0.3 * Noise(); });
    double diff = 0;
    for (size_t i = 0; i < o.l.size(); ++i) diff = std::fmax(diff, std::fabs(o.l[i] - o.r[i]));
    check("spread 0: R = L (mono) at any tap", diff == 0, diff);
  }
  {
    SchliereParameters p = Settings(1000, 0.5, 3, 0);
    p.spread = 1.0f;
    Out o = Impulse(Z, p, 16384, A);
    double err = std::abs(Response(o.l, 700, A) - Reference(700, 1000, 0.51, 17)) +
        std::abs(Response(o.r, 700, A) - Reference(700, 1000, 0.51, 61));
    check("spread 1: L at tap 3 (17 stages), R at the chain end (61 stages)", err < 5e-3, err);
    // The hold zones keep the detents exact under small knob deviations.
    p.spread = 0.96f;
    Out q = Impulse(Z, p, 16384, A);
    double hold = std::abs(Response(q.r, 700, A) - Reference(700, 1000, 0.51, 61));
    check("  spread 0.96 is still inside the hold zone of the chain end", hold < 5e-3, hold);
  }
  {
    static Schliere s;
    s.Init(S);
    SchliereParameters p = Settings(1000, 0.5, 6, 0.6);
    p.mix = 0.0f;
    std::vector<double> in;
    Out o = Run(&s, p, 48000, [&](size_t) { in.push_back(0.3 * Noise()); return in.back(); });
    double diff = 0;
    for (size_t i = 0; i < o.l.size(); ++i) {
      diff = std::fmax(diff, std::fabs(o.l[i] - static_cast<float>(in[i])));
      diff = std::fmax(diff, std::fabs(o.r[i] - static_cast<float>(in[i])));
    }
    check("mix 0: both outputs carry the dry input", diff < 1e-6, diff);
  }
  {
    // 50/50 with the dry signal: the classic phaser notches (full
    // cancellation where the chain turns the phase by an odd multiple of pi).
    SchliereParameters p = Settings(1000, 0.5, 7, 0);
    p.mix = 0.5f;
    Out o = Impulse(Z, p, 16384, A);
    double lo = 1e9, hi = -1e9;
    for (double f = 300; f < 3000; f += 0.5) {
      double m = Db(std::abs(Response(o.l, f, A)));
      lo = std::fmin(lo, m);
      hi = std::fmax(hi, m);
    }
    check("mix 0.5: phaser notches below -40 dB, peaks at 0 dB", lo < -40 && std::fabs(hi) < 0.05, lo, hi);
  }
  {
    // An excitation impulse is the same as an input impulse of kExciteLevel
    // (0.5) at the first sample of the block (feedback 0). Z, because S and O
    // run the input through the 96 kHz halfbands, the excitation not.
    static Schliere s;
    s.Init(Z);
    SchliereParameters p = Settings(1000, 0.8, 5, 0);
    const size_t trigger = 10 * kBlock;
    Out e = Run(&s, p, 8192, [](size_t) { return 0.0; },
                [&](SchliereParameters* q, size_t t) { q->excite = t == trigger; });
    static Schliere r;
    r.Init(Z);
    Out i = Run(&r, p, 8192, [&](size_t t) { return t == trigger ? 0.5 : 0.0; });
    double d = 0;
    for (size_t n = 0; n < e.l.size(); ++n) d = std::fmax(d, std::fabs(e.l[n] - i.l[n]));
    check("excite impulse = input impulse of 0.5 at the trigger", d < 1e-6 && Peak(e.l) > 0.01, d, Peak(e.l));
  }
  {
    static Schliere s;
    s.Init(S);
    SchliereParameters p = Settings(1000, 0.5, 1, 0);
    p.excite_type = SCHLIERE_EXCITE_NOISE_BURST;
    const size_t trigger = 10 * kBlock;
    Out o = Run(&s, p, 48000, [](size_t) { return 0.0; },
                [&](SchliereParameters* q, size_t t) { q->excite = t == trigger; });
    double burst = Peak(o.l, trigger, trigger + 480);
    double after = Peak(o.l, trigger + 2400);
    check("excite noise burst: audible, then silent after 50 ms (peak, after)", Finite(o) && burst > 0.05 && after < 1e-4, burst, after);
  }

  printf("%s\n", fails ? "FAILURES" : "all cases ok");
  return fails;
}
