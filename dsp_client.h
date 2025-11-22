/**
 * dsp_client.h
 *
 * Copyright (C) 2023-2024 Pablo Alvarado
 * EL5805 Procesamiento Digital de Señales
 * Escuela de Ingeniería Electrónica
 * Tecnológico de Costa Rica
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the authors nor the names of its contributors may be
 * used to endorse or promote products derived from this software without
 * specific prior written permission.
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
#ifndef _DSP_CLIENT_H
#define _DSP_CLIENT_H

#include "jack_client.h"
#include "freq_filter_goertzel.h"
#include "prealloc_ringbuffer.h"
#include "fir_filter.h"  // Unified FIR filter class (handles both Hilbert and standard FIR)
#include "fsk4_system.h"
#include "jack_client.h"
#include "hilbert_filter_coeffs.h"  // Hilbert filter coefficients
#include "lpf_demod_coeffs.h"       // Low-pass filter coefficients
#include "fir_filter.h"              // Unified FIR filter class
#include <boost/circular_buffer.hpp>
#include <numbers>
#include <cstring>
#include <cmath>
#include <iostream>
#include <boost/circular_buffer.hpp>
#include <memory>
#include <atomic>
#include <array>


class dsp_client : public jack::client {
public:
  enum class ModulationScheme {
    SSB_USB,      // Single Sideband - Upper Sideband
    SSB_LSB,      // Single Sideband - Lower Sideband
    SSB_USB_SC,   // SSB-USB Suppressed Carrier
    SSB_LSB_SC,   // SSB-LSB Suppressed Carrier
    FSK_4         // 4-level Frequency Shift Keying
  };

  enum class Mode {
    Passthrough,  // Direct audio passthrough
    Transmit,     // Modulation mode
    Receive,      // Demodulation mode
    Stopped       // No processing
  };

  dsp_client();
  ~dsp_client();
// Mode control
  void set_mode(Mode mode);
  Mode get_mode() const { return _current_mode; }
  
  // Modulation control
  void set_transmit_modulation(ModulationScheme scheme);
  void set_receive_modulation(ModulationScheme scheme);
  ModulationScheme get_transmit_modulation() const { return _tx_modulation; }
  ModulationScheme get_receive_modulation() const { return _rx_modulation; }
  
  // Carrier frequency control
  void set_transmit_carrier_freq(float freq);
  void set_receive_carrier_freq(float freq);
  float get_transmit_carrier_freq() const { return _tx_carrier_freq; }
  float get_receive_carrier_freq() const { return _rx_carrier_freq; }
  
  // Processing control
  void start_processing();
  void stop_processing();
  bool is_processing() const { return _processing_active; }

  // Display data access
  const std::vector<sample_t>& last_buffer();
  float power() const;
  void set_volume(float vol);
  float volume() const;

  // NEW: FSK-4 data access (lock-free, safe for non-RT thread)
  struct FSK4Data {
    std::array<float, 4> magnitudes;       // Current magnitudes
    std::array<float, 4> magnitudes_sq;    // Magnitude squared (faster)
    int strongest_index;                   // 0, 1, 2, or 3
    uint64_t block_counter;                // Increments each block
    bool valid;                            // Is data valid?
  };
  
  FSK4Data get_fsk4_data() const;          // Thread-safe getter
  
  // NEW: FSK-4 configuration
  void set_fsk4_frequencies(float f1, float f2, float f3, float f4);
  std::array<float, 4> get_fsk4_frequencies() const { return _fsk4_frequencies; }

    // NEW: Modulation gain control (to avoid magic numbers like *20)
  void set_modulation_gain(float gain);
  float get_modulation_gain() const { return _modulation_gain; }
  
  // NEW: Separate TX and RX FSK frequencies
  void set_tx_fsk4_frequencies(float f1, float f2, float f3, float f4);
  void set_rx_fsk4_frequencies(float f1, float f2, float f3, float f4);
  std::array<float, 4> get_tx_fsk4_frequencies() const { return _tx_fsk4_frequencies; }
  std::array<float, 4> get_rx_fsk4_frequencies() const { return _rx_fsk4_frequencies; }

protected:
  bool init_subclass() override;
  bool process(jack_nframes_t nframes,
               const sample_t *const in,
               sample_t *const out) override;

private:
  freq_filter _ffilter;
  float _volume;
  Mode _current_mode;
  ModulationScheme _tx_modulation;
  ModulationScheme _rx_modulation;
  float _tx_carrier_freq;
  float _rx_carrier_freq;
  bool _processing_active;

  // Oscillator state for SSB (using difference equations)
  float _osc_y_n_minus_1;
  float _osc_y_n_minus_2;
  float _osc_a1;
  
  float _osc_sin_y_n_minus_1;
  float _osc_sin_y_n_minus_2;
  float _osc_sin_a1;

  // Hilbert filter and delay buffers for SSB
  FIRFilter* _hilbert_filter;
  FIRFilter* _lpf_demod;

  // FSK-4 Transmitter and Receiver with circular buffers
  std::unique_ptr<fsk4::FSK4TransmitterBuffer> _fsk4_tx;
  std::unique_ptr<fsk4::FSK4ReceiverBuffer> _fsk4_rx;

  boost::circular_buffer<float> _message_delay;
  boost::circular_buffer<float> _demod_delay;

  // Power measurement
  float _power;
  
  // Past buffers for display
  struct past_buffers_t {
    void allocate(const std::size_t cnt,const std::vector<float>& proto);
    std::vector<float>& back();
    const std::vector<float>& back() const;
    void push_back();
  private:
    std::vector< std::vector<float> > _data;
    std::size_t _idx;
  };
  past_buffers_t _past_buffers;

  // Helper method for sine generation
  void play_sine(float freq, float amplitude);

  // NEW: 4-FSK Support
  // ==================
  std::unique_ptr<fsk4_detector> _fsk4_detector;
  std::array<float, 4> _fsk4_frequencies;
  
  // Lock-free atomic storage for FSK data (safe for RT → non-RT communication)
  std::atomic<uint64_t> _fsk4_block_counter;
  
  // We use a simple double-buffering approach:
  // - RT thread writes to one buffer
  // - Non-RT thread reads from other buffer
  // - Atomic flag indicates which is current
  struct FSK4Buffer {
    std::array<float, 4> magnitudes;
    std::array<float, 4> magnitudes_sq;
    int strongest_index;
  };
  
  FSK4Buffer _fsk4_buffer[2];
  std::atomic<int> _fsk4_current_buffer;  // 0 or 1
  
  // FSK transmitter state
  struct FSKTransmitter {
    float phase;
    float current_freq;
    int current_symbol;
    std::size_t samples_in_symbol;
    std::size_t samples_per_symbol;
    
    FSKTransmitter() 
      : phase(0.0f)
      , current_freq(1000.0f)
      , current_symbol(0)
      , samples_in_symbol(0)
      , samples_per_symbol(1024)
    {}
    
    void reset() {
      phase = 0.0f;
      samples_in_symbol = 0;
    }
    
    bool symbol_complete() const {
      return samples_in_symbol >= samples_per_symbol;
    }
  };
  
  FSKTransmitter _fsk_tx;
  
  // FSK helper methods
  void init_fsk4();
  void process_fsk4_rx(const sample_t* in, std::size_t nframes);
  void process_fsk4_tx(sample_t* out, std::size_t nframes);
  float generate_fsk_sample();

  // NEW: Modulation gain (replaces hardcoded *20 multipliers)
  float _modulation_gain;
  
  // NEW: Separate FSK frequencies for TX and RX
  std::array<float, 4> _tx_fsk4_frequencies;
  std::array<float, 4> _rx_fsk4_frequencies;
  
  // NEW: Separate detectors for TX and RX (if you want independent configs)
  std::unique_ptr<fsk4_detector> _tx_fsk4_detector;
  std::unique_ptr<fsk4_detector> _rx_fsk4_detector;


  
public:
  // NEW: FSK transmitter control (called from non-RT thread)
  void set_fsk_symbol(int symbol);  // Set next symbol to transmit (0-3)
  int get_current_fsk_symbol() const { return _fsk_tx.current_symbol; }
};

#endif // DSP_CLIENT_H