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
#include <boost/circular_buffer.hpp>

#include <cstring>
#include <cmath>

dsp_client::dsp_client() : jack::client(),_ffilter(),
    _volume(1.0f),
    _current_mode(Mode::Stopped),
    _tx_modulation(ModulationScheme::SSB_USB),
    _rx_modulation(ModulationScheme::SSB_USB),
    _tx_carrier_freq(1000.0f),
    _rx_carrier_freq(1000.0f),
    _processing_active(false),
    _osc_y_n_minus_1(0.0f),
    _osc_y_n_minus_2(0.0f),
    _osc_a1(0.0f)
{
}

dsp_client::~dsp_client() {
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
  
  return true;
}

bool dsp_client::process(jack_nframes_t nframes,
                         const sample_t *const in,
                         sample_t *const out) {

  const sample_t* inptr = in;
  const sample_t* endptr = in + nframes;
  sample_t* outptr = out;

  // Process based on current mode - no function calls, everything inline
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
        // Input comes from WAV file
        // Generate carrier using difference equation oscillator
        // y(n) = -a1*y(n-1) - y(n-2)
        // where a1 = -2*cos(ω0)
        
        while(inptr != endptr) {
          // Difference equation oscillator (Proakis Eq. 4.52)
          float y_n = -_osc_a1 * _osc_y_n_minus_1 - _osc_y_n_minus_2;
          
          // TODO: Implement modulation schemes here based on _tx_modulation
          // Modulate the input signal with the carrier
          // For now, just output the carrier oscillator
          *outptr = y_n * _volume * 0.5f;
          
          // Update oscillator state
          _osc_y_n_minus_2 = _osc_y_n_minus_1;
          _osc_y_n_minus_1 = y_n;
          
          ++outptr;
          ++inptr;
        }
      } else {
        // Not processing, output silence
        memset(out, 0, nframes * sizeof(sample_t));
      }
      break;

    case Mode::Receive:
      if (_processing_active) {
        // Input comes from microphone
        // Generate local oscillator for demodulation
        
        while(inptr != endptr) {
          // Difference equation oscillator for local carrier
          float y_n = -_osc_a1 * _osc_y_n_minus_1 - _osc_y_n_minus_2;
          
          // TODO: Implement demodulation schemes here based on _rx_modulation
          // Mix input with local oscillator for demodulation
          // For now, just pass through
          *outptr = *inptr * _volume;
          
          // Update oscillator state
          _osc_y_n_minus_2 = _osc_y_n_minus_1;
          _osc_y_n_minus_1 = y_n;
          
          ++outptr;
          ++inptr;
        }
      } else {
        // Not processing, output silence
        memset(out, 0, nframes * sizeof(sample_t));
      }
      break;

    case Mode::Stopped:
      // Output silence
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
  // Initialize the difference equation oscillator
  // Based on Proakis Eq. 4.52: y(n) = -a1*y(n-1) - y(n-2)
  // where a1 = -2*cos(ω0) and a2 = 1
  
  // Normalized frequency
  const float norm_freq = freq / sample_rate();
  
  // Calculate average signal power to prevent saturation
  const float avg_signal_power = (1.0f / (2.0f * sample_rate() + 1.0f));
  
  // Set coefficient: a1 = -2*cos(2π*norm_freq)
  _osc_a1 = 2.0f * std::cos(2.0f * std::numbers::pi_v<float> * norm_freq);
  
  // Set initial conditions to start oscillation with proper scaling
  // y(-1) = 0
  // y(-2) = -amplitude * sqrt(avg_signal_power) * sin(2π*norm_freq)
  _osc_y_n_minus_1 = 0.0f;
  _osc_y_n_minus_2 = -amplitude * std::sqrt(avg_signal_power) * std::sin(2.0f * std::numbers::pi_v<float> * norm_freq);
}

void dsp_client::set_mode(Mode mode) {
  _current_mode = mode;
  
  // Reset oscillator when changing modes
  _osc_y_n_minus_1 = 0.0f;
  _osc_y_n_minus_2 = 0.0f;
}

void dsp_client::set_transmit_carrier_freq(float freq) {
  _tx_carrier_freq = freq;
  if (_current_mode == Mode::Transmit && _processing_active) {
    // Reinitialize oscillator with new frequency
    play_sine(_tx_carrier_freq, 1.0f);
  }
}

void dsp_client::set_receive_carrier_freq(float freq) {
  _rx_carrier_freq = freq;
  if (_current_mode == Mode::Receive && _processing_active) {
    // Reinitialize oscillator with new frequency
    play_sine(_rx_carrier_freq, 1.0f);
  }
}

void dsp_client::set_transmit_modulation(ModulationScheme scheme) {
  _tx_modulation = scheme;
  // Future: reset any modulation-specific state here
}

void dsp_client::set_receive_modulation(ModulationScheme scheme) {
  _rx_modulation = scheme;
  // Future: reset any demodulation-specific state here
}

void dsp_client::start_processing() {
  _processing_active = true;
  
  // Initialize the oscillator with the appropriate carrier frequency
  float freq = (_current_mode == Mode::Transmit) ? _tx_carrier_freq : _rx_carrier_freq;
  play_sine(freq, 1.0f);
}

void dsp_client::stop_processing() {
  _processing_active = false;
  
  // Reset oscillator state
  _osc_y_n_minus_1 = 0.0f;
  _osc_y_n_minus_2 = 0.0f;
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
