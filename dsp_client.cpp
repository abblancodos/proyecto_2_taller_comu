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

dsp_client::dsp_client() 
  : jack::client()
  , _ffilter()
  , _volume(1.0f)
  , _current_mode(Mode::Stopped)
  , _tx_modulation(ModulationScheme::SSB_USB)
  , _rx_modulation(ModulationScheme::SSB_USB)
  , _tx_carrier_freq(10000.0f)
  , _rx_carrier_freq(10000.0f)
  , _processing_active(false)
  , _osc_y_n_minus_1(0.0f)
  , _osc_y_n_minus_2(0.0f)
  , _osc_a1(0.0f)
  , _osc_sin_y_n_minus_1(0.0f)
  , _osc_sin_y_n_minus_2(0.0f)
  , _osc_sin_a1(0.0f)
  , _hilbert_filter(nullptr)
  , _lpf_demod(nullptr)
  // NEW: FSK initialization
  , _fsk4_block_counter(0)
  , _fsk4_current_buffer(0)
  , _modulation_gain(1.0f)  // NEW: Default gain = 1x
  , _tx_fsk4_frequencies{1000.0f, 2000.0f, 3000.0f, 4000.0f}  // NEW: TX FSK freqs
  , _rx_fsk4_frequencies{1000.0f, 2000.0f, 3000.0f, 4000.0f}  // NEW: RX FSK freqs
{
  // Initialize FSK buffers
  for (int i = 0; i < 2; ++i) {
    _fsk4_buffer[i].magnitudes.fill(0.0f);
    _fsk4_buffer[i].magnitudes_sq.fill(0.0f);
    _fsk4_buffer[i].strongest_index = 0;
  }
}

dsp_client::~dsp_client() {
    if (_hilbert_filter) {
        delete _hilbert_filter;
    }
    if (_lpf_demod) {
        delete _lpf_demod;
    }
}

// ============================================================================
// past_buffers_t implementation
// ============================================================================

void dsp_client::past_buffers_t::allocate(const std::size_t cnt,
                                          const std::vector<float>& proto) {
  _data.clear();
  _data.resize(cnt, proto);
  _idx = 0;
}

std::vector<float>& dsp_client::past_buffers_t::back() {
  return _data[_idx];
}

const std::vector<float>& dsp_client::past_buffers_t::back() const {
  return _data[_idx];
}

void dsp_client::past_buffers_t::push_back() {
  _idx = (_idx + 1) % _data.size();
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
  
  // NEW: Initialize Hilbert filter (with optimization)
  _hilbert_filter = new FIRFilter(hilbert::HILBERT_COEFFS, hilbert::FILTER_LENGTH, FIRFilterType::HILBERT);
  
  // NEW: Initialize Low-Pass Filter for demodulation (standard FIR)
  _lpf_demod = new FIRFilter(lpf_demod::LPF_COEFFS, lpf_demod::FILTER_LENGTH, FIRFilterType::STANDARD);
  
  // NEW: Initialize message delay buffer (for group delay compensation)
  _message_delay.set_capacity(hilbert::GROUP_DELAY);
  for (size_t i = 0; i < hilbert::GROUP_DELAY; ++i) {
      _message_delay.push_back(0.0f);
  }
  
  // NEW: Initialize demodulation delay buffer (for LPF group delay compensation)
  _demod_delay.set_capacity(lpf_demod::GROUP_DELAY);
  for (size_t i = 0; i < lpf_demod::GROUP_DELAY; ++i) {
      _demod_delay.push_back(0.0f);
  }
  
  const float BAUDRATE = 100.0f;  // symbols/second
  const size_t BUFFER_SIZE = 65536;  // Large circular buffer

  _fsk4_tx = std::make_unique<fsk4::FSK4TransmitterBuffer>(
    _tx_fsk4_frequencies[0],
    _tx_fsk4_frequencies[1],
    _tx_fsk4_frequencies[2],
    _tx_fsk4_frequencies[3],
    BAUDRATE,
    sample_rate(),
    BUFFER_SIZE
  );

  _fsk4_rx = std::make_unique<fsk4::FSK4ReceiverBuffer>(
    _rx_fsk4_frequencies[0],
    _rx_fsk4_frequencies[1],
    _rx_fsk4_frequencies[2],
    _rx_fsk4_frequencies[3],
    BAUDRATE,
    sample_rate(),
    buffer_size(),
    BUFFER_SIZE
  );
        

  
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
        // Switch on transmit modulation scheme
        switch(_tx_modulation) {

          case ModulationScheme::FSK_4:
            // NEW: 4-FSK TRANSMIT
            _fsk4_tx->get_samples(out, nframes);
            inptr = out;
            endptr = out + nframes;
            while(inptr != endptr){
              *out *= _volume * _modulation_gain;
              ++out;
            }

          case ModulationScheme::SSB_USB:
          case ModulationScheme::SSB_USB_SC:
            // SSB USB MODULATION
            // Input: message signal from WAV file
            // Output: SSB USB modulated signal
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
              
              // Step 5: USB modulation
              // USB: s(t) = m(t)*cos(ωc*t) - m̂(t)*sin(ωc*t)
              float modulated = message_delayed * carrier_cos - message_hilbert * carrier_sin;
              
              // Apply volume and output
              *outptr = modulated * _modulation_gain * _volume;
              
              ++outptr;
              ++inptr;
            }
            break;
            
          case ModulationScheme::SSB_LSB:
          case ModulationScheme::SSB_LSB_SC:
            // SSB LSB MODULATION
            // Input: message signal from WAV file
            // Output: SSB LSB modulated signal
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
              
              // Step 5: LSB modulation
              // LSB: s(t) = m(t)*cos(ωc*t) + m̂(t)*sin(ωc*t)
              float modulated = message_delayed * carrier_cos + message_hilbert * carrier_sin;
              
              // Apply volume and output
              *outptr = modulated * _modulation_gain * _volume;
              
              ++outptr;
              ++inptr;
            }
            break;
            
          default:
            // Unknown modulation scheme - output silence
            memset(out, 0, nframes * sizeof(sample_t));
            break;
        }
      } else {
        memset(out, 0, nframes * sizeof(sample_t));
      }
      break;

    case Mode::Receive:
      if (_processing_active) {
        // Switch on receive modulation scheme
        switch(_rx_modulation) {
          case ModulationScheme::FSK_4:
            _fsk4_rx->put_samples(in, nframes);
            break;

          case ModulationScheme::SSB_USB:
          case ModulationScheme::SSB_USB_SC:
            // SSB USB DEMODULATION
            // Input: received SSB USB signal from microphone
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
              
              // Step 4: Low-pass filter to remove high-frequency components
              float demod_filtered = _lpf_demod->process(demod_raw);
              
              // Step 5: Delay compensation for LPF group delay
              _demod_delay.push_back(demod_filtered);
              float demodulated = _demod_delay.front();
              
              // Apply volume and output
              *outptr = demodulated * _modulation_gain * _volume;
              
              ++outptr;
              ++inptr;
            }
            break;
            
          case ModulationScheme::SSB_LSB:
          case ModulationScheme::SSB_LSB_SC:
            // SSB LSB DEMODULATION
            // Input: received SSB LSB signal from microphone
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
              
              // Step 4: Low-pass filter to remove high-frequency components
              float demod_filtered = _lpf_demod->process(demod_raw);
              
              // Step 5: Delay compensation for LPF group delay
              _demod_delay.push_back(demod_filtered);
              float demodulated = _demod_delay.front();
              
              // Apply volume and output
              *outptr = demodulated * _modulation_gain * _volume;
              
              ++outptr;
              ++inptr;
            }
            break;
            
          default:
            // Unknown modulation scheme - output silence
            memset(out, 0, nframes * sizeof(sample_t));
            break;
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


// ============================================================================
// NEW: FSK RECEIVE - Uses RX frequencies and RX detector
// ============================================================================

void dsp_client::process_fsk4_rx(const sample_t* in, std::size_t nframes) {
  if (!_rx_fsk4_detector) return;
  
  // Process through RX Goertzel filters
  _rx_fsk4_detector->process_block(in, nframes);
  
  // Get results from RX detector
  const auto& magnitudes = _rx_fsk4_detector->get_current_magnitudes();
  const auto& magnitudes_sq = _rx_fsk4_detector->get_current_magnitudes_squared();
  int strongest = _rx_fsk4_detector->get_strongest_frequency_index();
  
  // Write to buffer (same as before)
  int write_buffer = 1 - _fsk4_current_buffer.load(std::memory_order_relaxed);
  
  _fsk4_buffer[write_buffer].magnitudes = magnitudes;
  _fsk4_buffer[write_buffer].magnitudes_sq = magnitudes_sq;
  _fsk4_buffer[write_buffer].strongest_index = strongest;
  
  _fsk4_current_buffer.store(write_buffer, std::memory_order_release);
  _fsk4_block_counter.fetch_add(1, std::memory_order_relaxed);
}

// ============================================================================
// NEW: FSK TRANSMIT - Uses TX frequencies
// ============================================================================

void dsp_client::process_fsk4_tx(sample_t* out, std::size_t nframes) {
  // Generate FSK samples using TX frequencies
  for (std::size_t i = 0; i < nframes; ++i) {
    // Use TX frequency for current symbol
    float freq = _tx_fsk4_frequencies[_fsk_tx.current_symbol];
    float omega = 2.0f * std::numbers::pi_v<float> * freq / sample_rate();
    
    float sample = std::sin(_fsk_tx.phase);
    _fsk_tx.phase += omega;
    
    if (_fsk_tx.phase > 2.0f * std::numbers::pi_v<float>) {
      _fsk_tx.phase -= 2.0f * std::numbers::pi_v<float>;
    }
    
    _fsk_tx.samples_in_symbol++;
    
    // Apply modulation gain and volume
    out[i] = sample * _modulation_gain * _volume;
  }
  
  if (_fsk_tx.samples_in_symbol >= _fsk_tx.samples_per_symbol) {
    _fsk_tx.samples_in_symbol = 0;
  }
}



// ============================================================================
// NEW: FSK DATA ACCESS (Non-RT thread)
// ============================================================================

dsp_client::FSK4Data dsp_client::get_fsk4_data() const {
  FSK4Data data;
  
  if (!_fsk4_detector) {
    data.valid = false;
    return data;
  }
  
  // Read from current buffer (lock-free)
  int read_buffer = _fsk4_current_buffer.load(std::memory_order_acquire);
  
  data.magnitudes = _fsk4_buffer[read_buffer].magnitudes;
  data.magnitudes_sq = _fsk4_buffer[read_buffer].magnitudes_sq;
  data.strongest_index = _fsk4_buffer[read_buffer].strongest_index;
  data.block_counter = _fsk4_block_counter.load(std::memory_order_relaxed);
  data.valid = true;
  
  return data;
}

// ============================================================================
// NEW: FSK FREQUENCY CONTROL - Separate TX and RX
// ============================================================================

void dsp_client::set_tx_fsk4_frequencies(float f1, float f2, float f3, float f4) {
  _tx_fsk4_frequencies = {f1, f2, f3, f4};
  
  // Update TX detector if it exists
  if (_tx_fsk4_detector) {
    _tx_fsk4_detector->set_frequencies(f1, f2, f3, f4);
  }
  
  std::cout << "TX FSK frequencies updated: "
            << f1 << ", " << f2 << ", " << f3 << ", " << f4 << " Hz" << std::endl;
}

void dsp_client::set_rx_fsk4_frequencies(float f1, float f2, float f3, float f4) {
  _rx_fsk4_frequencies = {f1, f2, f3, f4};
  
  // Update RX detector if it exists
  if (_rx_fsk4_detector) {
    _rx_fsk4_detector->set_frequencies(f1, f2, f3, f4);
  }
  
  std::cout << "RX FSK frequencies updated: "
            << f1 << ", " << f2 << ", " << f3 << ", " << f4 << " Hz" << std::endl;
}

void dsp_client::set_fsk_symbol(int symbol) {
  if (symbol >= 0 && symbol < 4) {
    _fsk_tx.current_symbol = symbol;
    // Note: We don't reset phase to maintain phase continuity (CPM)
    // If you want discontinuous FSK, add: _fsk_tx.reset();
  }
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

  // NEW: Reset FSK state
  if (_fsk4_detector) {
    _fsk4_detector->reset();
  }
  _fsk_tx.reset();
}


void dsp_client::set_modulation_gain(float gain) {
  // Clamp gain to reasonable range
  _modulation_gain = std::max(0.0f, std::min(gain, 30.0f));
  
  std::cout << "Modulation gain set to: " << _modulation_gain << "x" << std::endl;
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