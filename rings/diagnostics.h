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
// Diagnostic builds for measuring the CPU load on the module without a
// debugger. All off in release builds.
//
// SCHLIERE_CPU_METER: in the Schliere bank, LED 1 shows the load of the last
// second (green < 60 %, yellow < 80 %, red < 95 %, red blinking above)
// instead of the limiter. The buttons work as in the release.
//
// SCHLIERE_BENCHMARK (needs SCHLIERE_CPU_METER): in the Schliere bank,
// Schliere runs only on every 16th block (audio passes dry), so the module
// is never overloaded. In every bank the LEDs blink the peak load in percent
// of the block budget: 1.5 s off, LED 1 red = tens, 1 s off, LED 2 green =
// ones (a digit 0 = one yellow blink), 2 s status. Status LED 1: bank green
// = grounded / yellow = bonus models / red = Schliere. Status LED 2: the
// Schliere variant, green = S 48 kHz / yellow = S 96 kHz / red = L; off =
// base load only. In the Schliere bank, left short toggles base load only
// (instead of dry/wet); the right button works as in the release. A trigger
// (STRUM, note change, right button) is kept until the next measured block,
// so the excitation is measured too; it then plays 16 times slower.

#ifndef RINGS_DIAGNOSTICS_H_
#define RINGS_DIAGNOSTICS_H_

// #define SCHLIERE_CPU_METER
// #define SCHLIERE_BENCHMARK

#endif  // RINGS_DIAGNOSTICS_H_
