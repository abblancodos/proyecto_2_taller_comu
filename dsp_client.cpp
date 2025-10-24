/**
 * dsp_client.cpp
 *
 * Copyright (C) 2023  Pablo Alvarado
 * EL5802 Procesamiento Digital de Señales
 * Escuela de Ingeniería Electrónica
 * Tecnológico de Costa Rica
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the authors nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "dsp_client.h"
#include "jack_client.h"
#include "hilbert_filter_coeffs.h"  // NEW: Include filter coefficients
#include <boost/circular_buffer.hpp>
#include <cstring>
#include <cmath>
#include <iostream>

dsp_client::dsp_client() : jack::client(),_ffilter(),
    _volume(1.0f),
    _current_mode(Mode::Stopped),
    _tx_modulation(ModulationScheme::SSB_USB),
    _rx_modulation(ModulationScheme::SSB_USB),
    _tx_carrier_freq(10000.0f),
    _rx_carrier_freq(10000.0f),
    _processing_active(false),
    _osc_y_n_minus_1(0.0f),
    _osc_y_n_minus_2(0.0f),
    _osc_a1(0.0f),
    _hilbert_filter(nullptr),  // NEW
    _osc_sin_y_n_minus_1(0.0f),  // NEW
    _osc_sin_y_n_minus_2(0.0f),  // NEW
    _osc_sin_a1(0.0f)  // NEW
{
}

dsp_client::~dsp_client() {
    if (_hilbert_filter) {
        delete _hilbert_filter;
    }
}

bool dsp_client::init_subclass() {
  // Setup impulse response for filter
  std::unique_ptr<float[]> delta(new float[this->buffer_size()]);
  memset(delta.get(),0,this->buffer_size()*sizeof(float));
  delta[0]=1.0f;
  
  _ffilter.set_block_size(buffer_size());
  _ffilter.set_filter(delta.get(),buffer_size(),2*buffer_size()-1);
  _power = 0.0f;

  // Initialize past buffers ring buffer
  const std::size_t total_buffers
    = static_cast<std::size_t>(std::ceil(0.5*sample_rate()/buffer_size()));

  _past_buffers.allocate(total_buffers,
                         std::vector<float>(buffer_size(),float()));
  _past_buffers.push_back();
  
  // NEW: Initialize Hilbert filter
  _hilbert_filter = new HilbertFIR(hilbert::HILBERT_COEFFS, hilbert::FILTER_LENGTH);
  
  // NEW: Initialize message delay buffer (for group delay compensation)
  _message_delay.set_capacity(hilbert::GROUP_DELAY);
  for (size_t i = 0; i < hilbert::GROUP_DELAY; ++i) {
      _message_delay.push_back(0.0f);
  }
  
  return true;
}

bool dsp_client::process(jack_nframes_t nframes,
                         const sample_t *const in,
                         sample_t *const out) {

  const sample_t* inptr = in;
  const sample_t* endptr = in + nframes;
  sample_t* outptr = out;

  // Process based on current mode - everything inline
  switch(_current_mode) {
    case Mode::Passthrough:
      // Simple passthrough with volume control
      while(inptr != endptr) {
        *outptr = *inptr * _volume;
        ++outptr;
        ++inptr;
      }
      break;

    case Mode::Transmit:
      if (_processing_active) {
        // SSB MODULATION (INLINE)
        // Input: message signal from WAV file
        // Output: SSB modulated signal
        
        while(inptr != endptr) {
          // Step 1: Get message sample
          float message = *inptr;
          
          // Step 2: Apply Hilbert transform to get m̂(t)
          float message_hilbert = _hilbert_filter->process(message);
          
          // Step 3: Delay original message by group delay to synchronize
          _message_delay.push_back(message);
          float message_delayed = _message_delay.front();
          
          // Step 4: Generate carrier signals using difference equation oscillators
          // Cosine: cos(ωc*t)
          float carrier_cos = -_osc_a1 * _osc_y_n_minus_1 - _osc_y_n_minus_2;
          _osc_y_n_minus_2 = _osc_y_n_minus_1;
          _osc_y_n_minus_1 = carrier_cos;
          
          // Sine: sin(ωc*t) - uses separate oscillator with 90° phase shift
          float carrier_sin = -_osc_sin_a1 * _osc_sin_y_n_minus_1 - _osc_sin_y_n_minus_2;
          _osc_sin_y_n_minus_2 = _osc_sin_y_n_minus_1;
          _osc_sin_y_n_minus_1 = carrier_sin;
          
          // Step 5: Modulate based on sideband selection
          float modulated;
          if (_tx_modulation == ModulationScheme::SSB_USB || 
              _tx_modulation == ModulationScheme::SSB_USB_SC) {
            // USB: s(t) = m(t)*cos(ωc*t) - m̂(t)*sin(ωc*t)
            modulated = message_delayed * carrier_cos - message_hilbert * carrier_sin;
          } else {
            // LSB: s(t) = m(t)*cos(ωc*t) + m̂(t)*sin(ωc*t)
            modulated = message_delayed * carrier_cos + message_hilbert * carrier_sin;
          }
          
          // Apply volume and output
          *outptr = modulated * 20 * _volume;
          
          ++outptr;
          ++inptr;
        }
      } else {
        memset(out, 0, nframes * sizeof(sample_t));
      }
      break;

    case Mode::Receive:
      if (_processing_active) {
        // SSB DEMODULATION (INLINE)
        // Input: received SSB signal from microphone
        // Output: demodulated message signal
        
        while(inptr != endptr) {
          // Step 1: Get received SSB signal
          float received = *inptr;
          
          // Step 2: Generate local oscillator (coherent carrier)
          float local_cos = -_osc_a1 * _osc_y_n_minus_1 - _osc_y_n_minus_2;
          _osc_y_n_minus_2 = _osc_y_n_minus_1;
          _osc_y_n_minus_1 = local_cos;
          
          // Step 3: Coherent detection (multiply by 2x carrier)
          // This shifts the SSB signal back to baseband plus a high-freq component
          float demod_raw = 2.0f * received * local_cos;
          
          // Step 4: Low-pass filter using Hilbert filter as makeshift LPF
          // Note: In production, use a proper low-pass filter here
          // The Hilbert filter acts as a bandpass, but works for demonstration
          float demod_filtered = _hilbert_filter->process(demod_raw);
          
          // Step 5: Delay compensation
          _message_delay.push_back(demod_filtered);
          float demodulated = _message_delay.front();
          
          // Apply volume and output
          *outptr = demodulated * _volume;
          
          ++outptr;
          ++inptr;
        }
      } else {
        memset(out, 0, nframes * sizeof(sample_t));
      }
      break;

    case Mode::Stopped:
      memset(out, 0, nframes * sizeof(sample_t));
      break;
  }

  // Calculate block energy for power meter
  float block_energy = 0.0f;
  const sample_t* energy_ptr = out;
  const sample_t* energy_end = out + nframes;
  while(energy_ptr != energy_end) {
    float sample = *energy_ptr;
    block_energy += sample * sample;
    ++energy_ptr;
  }
  block_energy /= nframes;

  // Apply low pass filter to block energy
  constexpr float c = 0.6f;
  _power = std::max(block_energy*c + (1.0f-c)*_power, block_energy);

  // Save output for GUI display
  _past_buffers.push_back();
  memcpy(_past_buffers.back().data(), out, nframes*sizeof(sample_t));

  return true;
}

void dsp_client::play_sine(float freq, float amplitude) {
  // Initialize BOTH cosine and sine oscillators
  const float norm_freq = freq / sample_rate();
  const float avg_signal_power = (1.0f / (2.0f * sample_rate() + 1.0f) * 1000);
  
  // Coefficient: a1 = -2*cos(2π*f/Fs)
  _osc_a1 = -2.0f * std::cos(2.0f * std::numbers::pi_v<float> * norm_freq);
  _osc_sin_a1 = _osc_a1;  // Same coefficient for both
  
  // Initialize cosine oscillator: cos(ωn)
  // y(-2) = cos(-2ω) = cos(2ω)
  // y(-1) = cos(-ω) = cos(ω)
  _osc_y_n_minus_2 = amplitude * std::sqrt(avg_signal_power) * 
                     std::cos(2.0f * std::numbers::pi_v<float> * norm_freq);
  _osc_y_n_minus_1 = amplitude * std::sqrt(avg_signal_power) * 
                     std::cos(std::numbers::pi_v<float> * norm_freq);
  
  // Initialize sine oscillator: sin(ωn) with 90° phase shift
  // y(-2) = sin(-2ω) = -sin(2ω)
  // y(-1) = sin(-ω) = -sin(ω)
  _osc_sin_y_n_minus_2 = -amplitude * std::sqrt(avg_signal_power) * 
                         std::sin(2.0f * std::numbers::pi_v<float> * norm_freq);
  _osc_sin_y_n_minus_1 = -amplitude * std::sqrt(avg_signal_power) * 
                         std::sin(std::numbers::pi_v<float> * norm_freq);
}

void dsp_client::set_mode(Mode mode) {
  _current_mode = mode;
  
  // Reset oscillators when changing modes
  _osc_y_n_minus_1 = 0.0f;
  _osc_y_n_minus_2 = 0.0f;
  _osc_sin_y_n_minus_1 = 0.0f;
  _osc_sin_y_n_minus_2 = 0.0f;
  
  // Reset Hilbert filter
  if (_hilbert_filter) {
    _hilbert_filter->reset();
  }
  
  // Clear message delay buffer
  for (size_t i = 0; i < _message_delay.capacity(); ++i) {
    _message_delay[i] = 0.0f;
  }
}

void dsp_client::set_transmit_carrier_freq(float freq) {
  _tx_carrier_freq = freq;
  if (_current_mode == Mode::Transmit && _processing_active) {
    play_sine(_tx_carrier_freq, 1.0f);
  }
}

void dsp_client::set_receive_carrier_freq(float freq) {
  _rx_carrier_freq = freq;
  if (_current_mode == Mode::Receive && _processing_active) {
    play_sine(_rx_carrier_freq, 1.0f);
  }
}

void dsp_client::set_transmit_modulation(ModulationScheme scheme) {
  _tx_modulation = scheme;
}

void dsp_client::set_receive_modulation(ModulationScheme scheme) {
  _rx_modulation = scheme;
}

void dsp_client::start_processing() {
  _processing_active = true;
  
  // Initialize oscillators with appropriate carrier frequency
  float freq = (_current_mode == Mode::Transmit) ? _tx_carrier_freq : _rx_carrier_freq;
  play_sine(freq, 1.0f);
  
  // Reset Hilbert filter
  if (_hilbert_filter) {
    _hilbert_filter->reset();
  }
}

void dsp_client::stop_processing() {
  _processing_active = false;
  
  // Reset oscillator state
  _osc_y_n_minus_1 = 0.0f;
  _osc_y_n_minus_2 = 0.0f;
  _osc_sin_y_n_minus_1 = 0.0f;
  _osc_sin_y_n_minus_2 = 0.0f;
}

const std::vector<dsp_client::sample_t>& dsp_client::last_buffer() {
  return _past_buffers.back();
}

float dsp_client::power() const {
  return _power;
}

void dsp_client::set_volume(float vol) {
  _volume = vol * vol;
}

float dsp_client::volume() const {
  return std::sqrt(_volume);
}