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
// User interface.

#include "rings/ui.h"

#include <algorithm>

#include <stm32f4xx_conf.h>

#include "stmlib/system/system_clock.h"

#include "rings/cv_scaler.h"
#include "rings/diagnostics.h"
#include "rings/dsp/part.h"
#include "rings/dsp/string_synth_part.h"

namespace rings {

const int32_t kLongPressDuration = 3000;
const int32_t kMediumPressDuration = 200;
// Both switches must be held this long to enter calibration.
const int32_t kCalibrationPressDuration = 6000;
// Schliere bank, right button: held this long (but released before
// kLongPressDuration, which changes the bank) selects the next variant
// (S 48 kHz -> S 96 kHz -> L).
const int32_t kSchliereToggleDuration = 1000;
const uint32_t kSchliereDisplayDuration = 1500;
const uint32_t kMixBlinkPeriod = 600;  // dry/wet display: 0.3 s on, 0.3 s off


// Diagnostics (rings/diagnostics.h): benchmark display timing.
const uint32_t kBlinkPeriod = 800;
const uint32_t kBenchmarkLeadIn = 1500;  // all off before the tens
const uint32_t kBenchmarkGap = 1000;  // all off between tens and ones
const uint32_t kBenchmarkPause = 2000;

using namespace std;
using namespace stmlib;

void Ui::Init(
    Settings* settings,
    CvScaler* cv_scaler,
    Part* part,
    StringSynthPart* string_synth) {
  leds_.Init();
  switches_.Init();
  
  settings_ = settings;
  cv_scaler_ = cv_scaler;
  part_ = part;
  string_synth_ = string_synth;
  
  if (switches_.pressed_immediate(1)) {
    State* state = settings_->mutable_state();
    if (state->color_blind == 1) {
      state->color_blind = 0; 
    } else {
      state->color_blind = 1; 
    }
    settings_->Save();
  }
  
  schliere_cycles_peak_ = 0;
  base_cycles_peak_ = 0;
  benchmark_value_ = 0;
  benchmark_start_ = 0;
  load_peak_ = 0;
  load_shown_ = 0;
  load_time_ = 0;
  benchmark_base_only_ = false;
  schliere_excite_request_ = false;
  excite_display_time_ = 0;
  schliere_limiter_active_ = false;
  schliere_tap_detent_ = 0;
  shown_tap_detent_ = 0;
  mix_display_time_ = 0;
  detent_display_time_ = 0;
  
  part_->set_polyphony(settings_->state().polyphony);
  part_->set_model(static_cast<ResonatorModel>(settings_->state().model));
  string_synth_->set_polyphony(settings_->state().polyphony);
  string_synth_->set_fx(static_cast<FxType>(settings_->state().model));
  mode_ = settings_->state().easter_egg
      ? UI_MODE_EASTER_EGG_INTRO
      : UI_MODE_NORMAL;
}

void Ui::SaveState() {
  settings_->mutable_state()->polyphony = part_->polyphony();
  settings_->mutable_state()->model = part_->model();
  settings_->Save();
}

void Ui::SetModel(int32_t model) {
  part_->set_model(static_cast<ResonatorModel>(model));
  string_synth_->set_fx(static_cast<FxType>(model));
}

void Ui::UpdateSchliereLeds(bool blink, bool slow_blink) {
  uint32_t now = system_clock.milliseconds();
#ifdef SCHLIERE_BENCHMARK
  {
    uint32_t tens = benchmark_value_ / 10;
    uint32_t ones = benchmark_value_ % 10;
    uint32_t tens_blinks = tens ? tens : 1;
    uint32_t ones_blinks = ones ? ones : 1;
    uint32_t t = now - benchmark_start_;
    uint32_t tens_start = kBenchmarkLeadIn;
    uint32_t tens_end = tens_start + tens_blinks * kBlinkPeriod;
    uint32_t ones_start = tens_end + kBenchmarkGap;
    uint32_t ones_end = ones_start + ones_blinks * kBlinkPeriod;
    uint32_t total = ones_end + kBenchmarkPause;
    if (benchmark_start_ == 0 || t >= total) {
      // Latch the peaks measured during the last repetition.
      uint32_t cycles = base_cycles_peak_;
      if (!benchmark_base_only_) {
        cycles += schliere_cycles_peak_;
      }
      benchmark_value_ = cycles / 840;  // percent of 84 000 cycles
      if (benchmark_value_ > 199) {
        benchmark_value_ = 199;  // up to 19 red blinks
      }
      schliere_cycles_peak_ = 0;
      base_cycles_peak_ = 0;
      benchmark_start_ = now | 1;
      t = 0;
    }
    if (t < tens_start || (t >= tens_end && t < ones_start)) {
      leds_.set(0, false, false);
      leds_.set(1, false, false);
    } else if (t < tens_end) {
      bool on = ((t - tens_start) % kBlinkPeriod) < kBlinkPeriod / 2;
      leds_.set(0, on, on && tens == 0);
      leds_.set(1, false, false);
    } else if (t < ones_end) {
      bool on = ((t - ones_start) % kBlinkPeriod) < kBlinkPeriod / 2;
      leds_.set(0, false, false);
      leds_.set(1, on && ones == 0, on);
    } else {
      // Pause: LED 1 = bank (green = grounded models, yellow = bonus
      // models, red = Schliere); LED 2 = Schliere variant (green = S
      // 48 kHz, yellow = S 96 kHz, red = L) or off = base load only.
      bool schliere = settings_->state().schliere;
      leds_.set(0, schliere || part_->model() >= 3, !schliere);
      SchliereSelection selection = settings_->schliere_selection();
      bool on = !benchmark_base_only_;
      leds_.set(1, on && selection != SCHLIERE_SELECTION_S_48K,
                on && selection <= SCHLIERE_SELECTION_S_96K);
    }
    return;
  }
#endif  // SCHLIERE_BENCHMARK
  int32_t detent = schliere_tap_detent_;
  if (detent != shown_tap_detent_) {
    shown_tap_detent_ = detent;
    if (detent != 0) {
      detent_display_time_ = now;
    }
  }
  
  // LED 1: excite flavor after a short press of the left button (steady
  // green / yellow / red = impulse / mallet / ping), dry/wet step after a
  // long press (4 / 3 / 2 / 1 yellow blinks = 100 / 75 / 50 / 25 % wet),
  // tap reaching a detent (green), or the loop limiter at work (red). The
  // latest button action clears the other display.
  if (excite_display_time_ &&
      now - excite_display_time_ < kSchliereDisplayDuration) {
    switch (settings_->schliere_excite()) {
      case SCHLIERE_EXCITE_IMPULSE: leds_.set(0, false, true); break;
      case SCHLIERE_EXCITE_MALLET: leds_.set(0, true, true); break;
      default: leds_.set(0, true, false); break;
    }
  } else if (mix_display_time_ &&
             now - mix_display_time_ <
                 kMixBlinkPeriod * (4 - settings_->state().schliere_dry_steps)) {
    // Wet share in quarters: 4 / 3 / 2 / 1 yellow blinks = 100 / 75 / 50 /
    // 25 % wet. Blinking tells it apart from the steady flavor display, which
    // uses the same colors (user 2026-11-21).
    bool on = ((now - mix_display_time_) % kMixBlinkPeriod) <
        kMixBlinkPeriod / 2;
    leds_.set(0, on, on);
  } else if (detent_display_time_ &&
             now - detent_display_time_ < kSchliereDisplayDuration &&
             shown_tap_detent_ != 0) {
    leds_.set(0, false, true);
  } else {
#ifdef SCHLIERE_CPU_METER
    uint32_t load = load_peak_;
    if (load >= load_shown_ || now - load_time_ > 1000) {
      load_shown_ = load;
      load_time_ = now;
      load_peak_ = 0;
    }
    if (load_shown_ < 60) {
      leds_.set(0, false, true);
    } else if (load_shown_ < 80) {
      leds_.set(0, true, true);
    } else if (load_shown_ < 95) {
      leds_.set(0, true, false);
    } else {
      leds_.set(0, blink, false);
    }
#else
    leds_.set(0, schliere_limiter_active_, false);
#endif  // SCHLIERE_CPU_METER
  }
  
  // LED 2: hard blink marks the bank, the color the variant (green = S
  // 48 kHz, yellow = S 96 kHz, red = L). While the right button is held
  // between 1 and 3 s, it previews the variant selected on release.
  SchliereSelection selection = settings_->schliere_selection();
  if (switches_.pressed(1) && press_time_[1] != 0 &&
      static_cast<int32_t>(now - press_time_[1]) >= kSchliereToggleDuration) {
    selection = NextSchliereSelection(selection);
  }
  leds_.set(1, slow_blink && selection != SCHLIERE_SELECTION_S_48K,
            slow_blink && selection <= SCHLIERE_SELECTION_S_96K);
}

void Ui::ToggleFrequencyLocking() {
  if (settings_->state().frequency_locked) {
    settings_->UnlockFrequency();
  } else {
    settings_->LockFrequency(cv_scaler_->pot_transpose());
  }
  SaveState();
}

void Ui::AnimateEasterEggLeds() {
  mode_ = settings_->state().easter_egg
      ? UI_MODE_EASTER_EGG_INTRO
      : UI_MODE_EASTER_EGG_OUTRO;
}

void Ui::Poll() {
  // 1kHz.
  system_clock.Tick();
  switches_.Debounce();
  
  for (uint8_t i = 0; i < kNumSwitches; ++i) {
    if (switches_.just_pressed(i)) {
      queue_.AddEvent(CONTROL_SWITCH, i, 0);
      press_time_[i] = system_clock.milliseconds();
      // Schliere bank: the right button excites the chain as soon as it goes
      // down (not when the left one is held: frequency locking/calibration).
      if (i == 1 && mode_ == UI_MODE_NORMAL && settings_->state().schliere &&
          !switches_.pressed(0)) {
        schliere_excite_request_ = true;
      }
    }
    if (switches_.pressed(i) && press_time_[i] != 0) {
      int32_t pressed_time = system_clock.milliseconds() - press_time_[i];
      int32_t long_press_duration = switches_.pressed(1 - i)
          ? kCalibrationPressDuration
          : kLongPressDuration;
      if (pressed_time > long_press_duration) {
        queue_.AddEvent(CONTROL_SWITCH, i, pressed_time);
        press_time_[i] = 0;
      } else if (pressed_time > kMediumPressDuration &&
                 mode_ == UI_MODE_NORMAL &&
                 switches_.pressed(1 - i)) {
        mode_ = UI_MODE_DISPLAY_FREQUENCY_LOCKING;
      }
    }
    if (switches_.released(i) && press_time_[i] != 0) {
      queue_.AddEvent(
          CONTROL_SWITCH,
          i,
          system_clock.milliseconds() - press_time_[i] + 1);
      press_time_[i] = 0;
    }
  }
  
  bool blink = (system_clock.milliseconds() & 127) > 64;
  bool slow_blink = (system_clock.milliseconds() & 255) > 128;
  switch (mode_) {
    case UI_MODE_NORMAL:
      {
        uint8_t pwm_counter = system_clock.milliseconds() & 15;
        uint8_t triangle = (system_clock.milliseconds() >> 5) & 31;
        triangle = triangle < 16 ? triangle : 31 - triangle;

        if (settings_->state().color_blind == 1) {
          uint8_t mode_red_brightness[] = {
            0, 15, 1,
            0, triangle, uint8_t(triangle >> 3)
          };
          uint8_t mode_green_brightness[] = {
            4, 15, 0, 
            uint8_t(triangle >> 1), triangle, 0,
          };
          
          uint8_t poly_counter = (system_clock.milliseconds() >> 7) % 12;
          uint8_t poly_brightness = (poly_counter >> 1) < part_->polyphony() &&
                (poly_counter & 1);
          uint8_t poly_red_brightness = part_->polyphony() >= 2
              ? 8 + 8 * poly_brightness
              : 0;
          uint8_t poly_green_brightness = part_->polyphony() <= 3
              ? 8 + 8 * poly_brightness
              : 0;
          if (part_->polyphony() == 1 || part_->polyphony() == 4) {
            poly_red_brightness >>= 3;
            poly_green_brightness >>= 2;
          }
          leds_.set(
              0,
              pwm_counter < poly_red_brightness,
              pwm_counter < poly_green_brightness);
          leds_.set(
              1,
              pwm_counter < mode_red_brightness[part_->model()],
              pwm_counter < mode_green_brightness[part_->model()]);
        } else {
          leds_.set(0, part_->polyphony() >= 2, part_->polyphony() <= 2);
          leds_.set(1, part_->model() >= 1, part_->model() <= 1);
          // Fancy modes!
          if (part_->polyphony() == 3) {
            leds_.set(0, true, pwm_counter < triangle);
          }
          if (part_->model() >= 3) {
            bool led_1 = part_->model() >= 4 && pwm_counter < triangle;
            bool led_2 = part_->model() <= 4 && pwm_counter < triangle;
            leds_.set(1, led_1, led_2);
          }
        }
#ifdef SCHLIERE_BENCHMARK
        UpdateSchliereLeds(blink, slow_blink);
#else
        if (settings_->state().schliere) {
          UpdateSchliereLeds(blink, slow_blink);
        }
#endif  // SCHLIERE_BENCHMARK
        ++strumming_flag_interval_;
        if (strumming_flag_counter_) {
          --strumming_flag_counter_;
          leds_.set(0, false, false);
        }
      }
      break;
    
    case UI_MODE_CALIBRATION_C1:
      leds_.set(0, blink, blink);
      leds_.set(1, false, false);
      break;

    case UI_MODE_CALIBRATION_C3:
      leds_.set(0, false, false);
      leds_.set(1, blink, blink);
      break;

    case UI_MODE_CALIBRATION_LOW:
      leds_.set(0, slow_blink, 0);
      leds_.set(1, slow_blink, 0);
      break;

    case UI_MODE_CALIBRATION_HIGH:
      leds_.set(0, false, slow_blink);
      leds_.set(1, false, slow_blink);
      break;
    
    case UI_MODE_DISPLAY_FREQUENCY_LOCKING:
      // Preview the state applied on release: red = lock, green = unlock.
      {
        bool locked = settings_->state().frequency_locked;
        leds_.set(0, !locked && blink, locked && blink);
        leds_.set(1, !locked && blink, locked && blink);
      }
      break;
    
    case UI_MODE_EASTER_EGG_INTRO:
      {
        uint8_t pwm_counter = system_clock.milliseconds() & 15;
        uint8_t triangle_1 = (system_clock.milliseconds() / 7) & 31;
        uint8_t triangle_2 = (system_clock.milliseconds() / 17) & 31;
        triangle_1 = triangle_1 < 16 ? triangle_1 : 31 - triangle_1;
        triangle_2 = triangle_2 < 16 ? triangle_2 : 31 - triangle_2;
        leds_.set(
            0,
            triangle_1 > pwm_counter,
            triangle_2 > pwm_counter);
        leds_.set(
            1,
            triangle_2 > pwm_counter,
            triangle_1 > pwm_counter);
      }
      break;

    case UI_MODE_EASTER_EGG_OUTRO:
      {
        uint8_t pwm_counter = 7;
        uint8_t triangle_1 = (system_clock.milliseconds() / 9) & 31;
        uint8_t triangle_2 = (system_clock.milliseconds() / 13) & 31;
        triangle_1 = triangle_1 < 16 ? triangle_1 : 31 - triangle_1;
        triangle_2 = triangle_2 < 16 ? triangle_2 : 31 - triangle_2;
        leds_.set(0, triangle_1 < pwm_counter, triangle_1 > pwm_counter);
        leds_.set(1, triangle_2 > pwm_counter, triangle_2 < pwm_counter);
      }
      break;
    
    case UI_MODE_PANIC:
      leds_.set(0, blink, false);
      leds_.set(1, blink, false);
      break;
  }
  leds_.Write();
}

void Ui::FlushEvents() {
  queue_.Flush();
}

void Ui::OnSwitchPressed(const Event& e) {

}

void Ui::OnSwitchReleased(const Event& e) {
  // Both switches held for more than kMediumPressDuration: the first release
  // toggles frequency locking, holding on enters calibration.
  if (mode_ == UI_MODE_DISPLAY_FREQUENCY_LOCKING) {
    if (e.data >= kCalibrationPressDuration) {
      StartCalibration();
    } else {
      ToggleFrequencyLocking();
      mode_ = UI_MODE_NORMAL;
    }
    press_time_[0] = press_time_[1] = 0;
    return;
  }
  
  // Check if the other switch is still pressed. A short press of both
  // switches is ignored, so that calibration is only entered by holding them.
  if (switches_.pressed(1 - e.control_id)) {
    if (mode_ == UI_MODE_CALIBRATION_C1) {
      StartNormalizationCalibration();
    } else if (e.data >= kCalibrationPressDuration) {
      StartCalibration();
    }
    press_time_[0] = press_time_[1] = 0;
    return;
  }
  
  switch (e.control_id) {
    case 0:
      if (e.data >= kLongPressDuration) {
        if (cv_scaler_->easter_egg()) {
          settings_->ToggleEasterEgg();
          AnimateEasterEggLeds();
        } else if (settings_->state().schliere) {
          // Dry/wet step (polyphony has no meaning in the Schliere bank). The
          // excite flavor is on the short press: changed more often (user
          // 2026-11-21).
          settings_->NextSchliereMix();
          mix_display_time_ = system_clock.milliseconds() | 1;
          excite_display_time_ = 0;
        } else {
          part_->set_polyphony(3);
          string_synth_->set_polyphony(3);
        }
        SaveState();
      } else {
        switch (mode_) {
          case UI_MODE_CALIBRATION_C1:
            CalibrateC1();
            break;
          case UI_MODE_CALIBRATION_C3:
            CalibrateC3();
            break;
          case UI_MODE_CALIBRATION_LOW:
            CalibrateLow();
            break;
          case UI_MODE_CALIBRATION_HIGH:
            CalibrateHigh();
            break;
          default:
#ifdef SCHLIERE_BENCHMARK
            if (settings_->state().schliere) {
              // Diagnostic: show the base load without Schliere::Process.
              benchmark_base_only_ = !benchmark_base_only_;
              break;
            }
#endif  // SCHLIERE_BENCHMARK
            if (settings_->state().schliere) {
              settings_->NextSchliereExcite();
              excite_display_time_ = system_clock.milliseconds() | 1;
              mix_display_time_ = 0;
              SaveState();
              break;
            }
            {
              int32_t polyphony = part_->polyphony();
              if (polyphony == 3) {
                polyphony = 2;
              }
              polyphony <<= 1;
              if (polyphony > 4) {
                polyphony = 1;
              }
              part_->set_polyphony(polyphony);
              string_synth_->set_polyphony(polyphony);
              SaveState();
            }
            break;
          }
      }
      break;
    
    case 1:
      if (e.data >= kLongPressDuration) {
        if (cv_scaler_->easter_egg()) {
          settings_->ToggleEasterEgg();
          AnimateEasterEggLeds();
        } else {
          // Bank cycle: grounded models -> bonus models -> Schliere.
          State* state = settings_->mutable_state();
          int32_t model = part_->model();
          if (state->schliere) {
            state->schliere = 0;
            SetModel(model >= 3 ? model - 3 : model);
          } else if (model >= 3) {
            state->schliere = 1;
          } else {
            SetModel(model + 3);
          }
        }
      } else if (settings_->state().schliere) {
        // The press itself excited the chain. Held 1..3 s: next variant,
        // S 48 kHz -> S 96 kHz (as on the AC/DC) -> L (48 kHz only).
        if (e.data >= kSchliereToggleDuration) {
          settings_->NextSchliereSelection();
        }
      } else {
        int32_t model = part_->model();
        if (model >= 3) {
          model -= 3;
        } else {
          model = (model + 1) % 3;
        }
        SetModel(model);
      }
      SaveState();
      break;
    
    default:
      break;
  }
}

void Ui::StartCalibration() {
  mode_ = UI_MODE_CALIBRATION_C1;
}

void Ui::CalibrateC1() {
  cv_scaler_->CalibrateC1();
  cv_scaler_->CalibrateOffsets();
  mode_ = UI_MODE_CALIBRATION_C3;
}

void Ui::CalibrateC3() {
  bool success = cv_scaler_->CalibrateC3();
  if (success) {
    settings_->Save();
    mode_ = UI_MODE_NORMAL;
  } else {
    mode_ = UI_MODE_PANIC;
  }
}

void Ui::StartNormalizationCalibration() {
  cv_scaler_->StartNormalizationCalibration();
  mode_ = UI_MODE_CALIBRATION_LOW;
}

void Ui::CalibrateLow() {
  cv_scaler_->CalibrateLow();
  mode_ = UI_MODE_CALIBRATION_HIGH;
}

void Ui::CalibrateHigh() {
  bool success = cv_scaler_->CalibrateHigh();
  if (success) {
    settings_->Save();
    mode_ = UI_MODE_NORMAL;
  } else {
    mode_ = UI_MODE_PANIC;
  }
}

void Ui::DoEvents() {
  while (queue_.available()) {
    Event e = queue_.PullEvent();
    if (e.control_type == CONTROL_SWITCH) {
      if (e.data == 0) {
        OnSwitchPressed(e);
      } else {
        OnSwitchReleased(e);
      }
    }
  }
  if (queue_.idle_time() > 800 && mode_ == UI_MODE_PANIC) {
    mode_ = UI_MODE_NORMAL;
  }
  if (mode_ == UI_MODE_EASTER_EGG_INTRO || mode_ == UI_MODE_EASTER_EGG_OUTRO) {
    if (queue_.idle_time() > 3000) {
      mode_ = UI_MODE_NORMAL;
      queue_.Touch();
    }
  } else if (queue_.idle_time() > 1000) {
    queue_.Touch();
  }
}

uint8_t Ui::HandleFactoryTestingRequest(uint8_t command) {
  uint8_t argument = command & 0x1f;
  command = command >> 5;
  uint8_t reply = 0;
  switch (command) {
    case FACTORY_TESTING_READ_POT:
    case FACTORY_TESTING_READ_CV:
      reply = cv_scaler_->adc_value(argument);
      break;
    
    case FACTORY_TESTING_READ_NORMALIZATION:
      reply = cv_scaler_->normalization(argument);
      break;      
    
    case FACTORY_TESTING_READ_GATE:
      reply = argument == 2
          ? cv_scaler_->gate_value()
          : switches_.pressed(argument);
      break;
      
    case FACTORY_TESTING_SET_BYPASS:
      part_->set_bypass(argument);
      break;
      
    case FACTORY_TESTING_CALIBRATE:
      {
        switch (argument) {
          case 0:
            StartCalibration();
            break;
          
          case 1:
            CalibrateC1();
            break;
          
          case 2:
            CalibrateC3();
            break;
          
          case 3:
            StartNormalizationCalibration();
            break;

          case 4:
            CalibrateLow();
            break;
          
          case 5:
            CalibrateHigh();
            queue_.Touch();
            break;
        }
      }
      break;
  }
  return reply;
}

}  // namespace rings
