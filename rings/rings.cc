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

#include "rings/drivers/adc.h"
#include "rings/drivers/codec.h"
#include "rings/drivers/debug_pin.h"
#include "rings/drivers/debug_port.h"
#include "rings/drivers/system.h"
#include "rings/drivers/version.h"
#include "rings/dsp/part.h"
#include "rings/dsp/schliere.h"
#include "rings/dsp/strummer.h"
#include "rings/dsp/string_synth_part.h"
#include "rings/cv_scaler.h"
#include "rings/diagnostics.h"
#include "rings/settings.h"
#include "rings/ui.h"

// #define PROFILE_INTERRUPT 1

using namespace rings;
using namespace stmlib;

uint16_t reverb_buffer[32768] __attribute__ ((section (".ccmdata")));

Codec codec;
CvScaler cv_scaler;
DebugPort debug_port;
Part part;
Settings settings;
StringSynthPart string_synth;
Strummer strummer;
Ui ui;

// Default interrupt handlers.
extern "C" {

int __errno;

void NMI_Handler() { }
void HardFault_Handler() { while (1); }
void MemManage_Handler() { while (1); }
void BusFault_Handler() { while (1); }
void UsageFault_Handler() { while (1); }
void SVC_Handler() { }
void DebugMon_Handler() { }
void PendSV_Handler() { }

void SysTick_Handler() {
  ui.Poll();
  if (settings.freshly_baked()) {
    if (debug_port.readable()) {
      uint8_t command = debug_port.Read();
      uint8_t response = ui.HandleFactoryTestingRequest(command);
      debug_port.Write(response);
    }
  }
}

}

float in[kMaxBlockSize];
float out[kMaxBlockSize];
float aux[kMaxBlockSize];

const float kNoiseGateThreshold = 0.00003f;
Schliere schliere;
bool schliere_active = false;

// Peak cycle count of FillBuffer() (DWT cycle counter), for profiling over
// SWD. Budget per block: 24 samples at 48 kHz = 500 us = 84 000 cycles.
volatile uint32_t fill_buffer_cycles_peak = 0;
const uint32_t kBlockBudgetCycles = 84000;

// Overload guard: if FillBuffer() runs over budget for this many blocks in a
// row while Schliere is active, leave the Schliere bank. Over budget, the
// audio interrupt starves the UI, so the buttons could not do it.
const int32_t kOverloadBlocks = 400;  // 0.2 s
int32_t overload_blocks = 0;

// SCHLIERE_BENCHMARK (rings/diagnostics.h): see there.
uint32_t benchmark_counter = 0;
bool benchmark_excite = false;
float benchmark_l[kMaxBlockSize];
float benchmark_r[kMaxBlockSize];
float in_level = 0.0f;

void FillBuffer(Codec::Frame* input, Codec::Frame* output, size_t size) {
#ifdef PROFILE_INTERRUPT
  TIC
#endif  // PROFILE_INTERRUPT
  uint32_t start_cycles = DWT->CYCCNT;
  PerformanceState performance_state;
  Patch patch;
  
  cv_scaler.DetectAudioNormalization(input, size);
  cv_scaler.Read(&patch, &performance_state, settings.state());
  
  if (settings.state().schliere) {
    if (!schliere_active) {
      schliere.Init(SCHLIERE_VARIANT_S);  // start from a silent chain
      schliere_active = true;
    }
    for (size_t i = 0; i < size; ++i) {
      in[i] = static_cast<float>(input[i].r) / 32768.0f;
    }
    // No input for the strummer: with STRUM unpatched it only reacts to V/OCT
    // note changes, never to onsets of the signal being processed (and it
    // replaces the probe pattern the unpatched STRUM jack reads).
    strummer.Process(NULL, size, &performance_state);
    SchliereParameters schliere_parameters;
    cv_scaler.ReadSchliere(
        patch, performance_state, settings.state(), &schliere_parameters);
    // The STRUM LED flag shows only what excites the chain.
    performance_state.strum = schliere_parameters.excite;
    if (ui.TakeSchliereExciteRequest()) {
      schliere_parameters.excite = true;  // right button
    }
    // Right button: S at 48 kHz (default), S with its 96 kHz feedback path
    // (≈ 10 % more of the block budget on the module; with block-rate CVs
    // the difference is small, 48 kHz judged fine; user 2026-11-20), or L
    // with the longer chain, 48 kHz only. Changing the chain fades the wet
    // signal out and back in (Schliere::set_chain).
    SchliereSelection selection = settings.schliere_selection();
    schliere.set_oversampled_feedback(selection == SCHLIERE_SELECTION_S_96K);
    schliere.set_chain(SchliereChainFor(selection));
#ifdef SCHLIERE_BENCHMARK
    // Schliere runs on every 16th block only: keep a trigger until then, so
    // that the excitation is measured too. It then plays 16 times slower.
    benchmark_excite |= schliere_parameters.excite;
    if ((++benchmark_counter & 15) == 0) {
      schliere_parameters.excite = benchmark_excite;
      benchmark_excite = false;
      uint32_t t0 = DWT->CYCCNT;
      schliere.Process(
          schliere_parameters, in, benchmark_l, benchmark_r, size);
      ui.set_schliere_cycles(DWT->CYCCNT - t0);
    }
    for (size_t i = 0; i < size; ++i) {
      out[i] = aux[i] = in[i];
    }
#else
    schliere.Process(schliere_parameters, in, out, aux, size);
#endif  // SCHLIERE_BENCHMARK
    ui.set_schliere_status(schliere.limiter_active(), schliere.tap_detent());
  } else if (settings.state().easter_egg) {
    schliere_active = false;
    for (size_t i = 0; i < size; ++i) {
      in[i] = static_cast<float>(input[i].r) / 32768.0f;
    }
    strummer.Process(NULL, size, &performance_state);
    string_synth.Process(performance_state, patch, in, out, aux, size);
  } else {
    schliere_active = false;
    // Apply noise gate.
    for (size_t i = 0; i < size; ++i) {
      float in_sample = static_cast<float>(input[i].r) / 32768.0f;
      float error, gain;
      error = in_sample * in_sample - in_level;
      in_level += error * (error > 0.0f ? 0.1f : 0.0001f);
      gain = in_level <= kNoiseGateThreshold 
            ? (1.0f / kNoiseGateThreshold) * in_level : 1.0f;
      in[i] = gain * in_sample;
    }
    strummer.Process(in, size, &performance_state);
    part.Process(performance_state, patch, in, out, aux, size);
  }
  
  for (size_t i = 0; i < size; ++i) {
    output[i].l = Clip16(static_cast<int32_t>(out[i] * 32768.0f));
    output[i].r = Clip16(static_cast<int32_t>(aux[i] * 32768.0f));
  }
  ui.set_strumming_flag(performance_state.strum);
  uint32_t cycles = DWT->CYCCNT - start_cycles;
  if (cycles > fill_buffer_cycles_peak) {
    fill_buffer_cycles_peak = cycles;
  }
  ui.set_load(cycles * 100 / kBlockBudgetCycles);
#ifdef SCHLIERE_BENCHMARK
  if (!schliere_active || (benchmark_counter & 15) != 0) {
    ui.set_base_cycles(cycles);  // everything but Schliere::Process
  }
#endif  // SCHLIERE_BENCHMARK
  if (schliere_active && cycles > kBlockBudgetCycles) {
    if (++overload_blocks >= kOverloadBlocks) {
      settings.mutable_state()->schliere = 0;  // saved with the next press
      overload_blocks = 0;
    }
  } else {
    overload_blocks = 0;
  }
#ifdef PROFILE_INTERRUPT
  TOC
#endif  // PROFILE_INTERRUPT
}

void Init() {
  System sys;
  Version version;
  
  sys.Init(true);
  version.Init();
  
  // Flash prefetch buffer: SystemInit only enables the caches. Measured on
  // the module: about 10 % less load with Schliere. Not on revision A
  // silicon (errata: prefetch not functional).
  if ((DBGMCU->IDCODE >> 16) != 0x1000) {
    FLASH->ACR |= FLASH_ACR_PRFTEN;
  }
  
  // Enable the DWT cycle counter (profiling, see fill_buffer_cycles_peak).
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  strummer.Init(0.01f, kSampleRate / kMaxBlockSize);
  part.Init(reverb_buffer);
  string_synth.Init(reverb_buffer);

  settings.Init();
  cv_scaler.Init(settings.mutable_calibration_data());
  ui.Init(&settings, &cv_scaler, &part, &string_synth);
  
  if (!codec.Init(!version.revised(), kSampleRate)) {
    ui.Panic();
  }
  if (!codec.Start(kMaxBlockSize, &FillBuffer)) {
    ui.Panic();
  }
  codec.set_line_input_gain(22);

  if (settings.freshly_baked()) {
#ifdef PROFILE_INTERRUPT
    DebugPin::Init();
#else
    debug_port.Init();
#endif  // PROFILE_INTERRUPT
  }
  sys.StartTimers();
}

int main(void) {
  Init();
  while (1) {
    ui.DoEvents();
  }
}
