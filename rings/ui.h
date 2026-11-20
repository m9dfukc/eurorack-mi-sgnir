// Copyright 2015 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
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
// User interface

#ifndef RINGS_UI_H_
#define RINGS_UI_H_

#include "stmlib/stmlib.h"

#include "stmlib/ui/event_queue.h"

#include "rings/drivers/leds.h"
#include "rings/drivers/switches.h"

namespace rings {

class Settings;
class CvScaler;
class Part;
class StringSynthPart;

enum UiMode {
  UI_MODE_NORMAL,
  UI_MODE_DISPLAY_FREQUENCY_LOCKING,
  UI_MODE_CALIBRATION_C1,
  UI_MODE_CALIBRATION_C3,
  UI_MODE_CALIBRATION_LOW,
  UI_MODE_CALIBRATION_HIGH,
  UI_MODE_EASTER_EGG_INTRO,
  UI_MODE_EASTER_EGG_OUTRO,
  UI_MODE_PANIC,
};

enum FactoryTestingCommand {
  FACTORY_TESTING_READ_POT,
  FACTORY_TESTING_READ_CV,
  FACTORY_TESTING_READ_GATE,
  FACTORY_TESTING_SET_BYPASS,
  FACTORY_TESTING_CALIBRATE,
  FACTORY_TESTING_READ_NORMALIZATION,
};

class Ui {
 public:
  Ui() { }
  ~Ui() { }
  
  void Init(
      Settings* settings,
      CvScaler* cv_scaler,
      Part* part,
      StringSynthPart* string_synth);
  void Poll();
  void DoEvents();
  void FlushEvents();
  void Panic() {
    mode_ = UI_MODE_PANIC;
  }
  
  // Called from the audio callback while the Schliere bank is active.
  void set_schliere_status(bool limiter_active, int32_t tap_detent) {
    schliere_limiter_active_ = limiter_active;
    schliere_tap_detent_ = tap_detent;
  }
  
  // Schliere bank: excitation requested with the right button (called from
  // the audio callback, clears the request).
  inline bool TakeSchliereExciteRequest() {
    bool request = schliere_excite_request_;
    schliere_excite_request_ = false;
    return request;
  }
  
  // CPU load of the last audio block in percent (called from the audio
  // callback). Shown as a peak-hold meter on LED 1 in the Schliere bank.
  void set_load(uint32_t percent) {
    if (percent > load_peak_) {
      load_peak_ = percent;
    }
  }
  
  // Diagnostic benchmark: peak cycles of Schliere::Process and of the rest
  // of FillBuffer (called from the audio callback).
  void set_schliere_cycles(uint32_t cycles) {
    if (cycles > schliere_cycles_peak_) {
      schliere_cycles_peak_ = cycles;
    }
  }
  void set_base_cycles(uint32_t cycles) {
    if (cycles > base_cycles_peak_) {
      base_cycles_peak_ = cycles;
    }
  }
  
  void set_strumming_flag(bool flag) {
    if (flag) {
      // Make sure the LED is off for a short enough time.
      strumming_flag_counter_ = std::min(50L, strumming_flag_interval_ >> 2);
      strumming_flag_interval_ = 0;
    }
  }
  
  uint8_t HandleFactoryTestingRequest(uint8_t command);
  
 private:
  void OnSwitchPressed(const stmlib::Event& e);
  void OnSwitchReleased(const stmlib::Event& e);
  void StartCalibration();
  void CalibrateC1();
  void CalibrateC3();
  void StartNormalizationCalibration();
  void CalibrateLow();
  void CalibrateHigh();
  void SaveState();
  void ToggleFrequencyLocking();
  void SetModel(int32_t model);
  void UpdateSchliereLeds(bool blink, bool slow_blink);
  void AnimateEasterEggLeds();

  stmlib::EventQueue<16> queue_;
  
  Leds leds_;
  Switches switches_;
  uint32_t press_time_[kNumSwitches];
  UiMode mode_;
  int32_t strumming_flag_counter_;
  int32_t strumming_flag_interval_;
  
  // Schliere bank: LED 1 shows the dry/wet step after a press of the left
  // button, the tap detent after it changes, otherwise the loop limiter.
  volatile uint32_t schliere_cycles_peak_;
  volatile uint32_t base_cycles_peak_;
  uint32_t benchmark_value_;
  uint32_t benchmark_start_;
  volatile uint32_t load_peak_;
  uint32_t load_shown_;
  uint32_t load_time_;
  volatile bool schliere_excite_request_;
  bool benchmark_base_only_;  // diagnostics: show the base load only
  uint32_t excite_display_time_;
  volatile bool schliere_limiter_active_;
  volatile int32_t schliere_tap_detent_;
  int32_t shown_tap_detent_;
  uint32_t mix_display_time_;
  uint32_t detent_display_time_;
  
  Settings* settings_;
  CvScaler* cv_scaler_;
  Part* part_;
  StringSynthPart* string_synth_;
  
  DISALLOW_COPY_AND_ASSIGN(Ui);
};

}  // namespace rings

#endif  // RINGS_UI_H_
