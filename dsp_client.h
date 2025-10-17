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
#include "freq_filter.h"
#include "prealloc_ringbuffer.h"
#include <boost/circular_buffer.hpp>

/**
 * Jack client class
 *
 * This class wraps some basic jack functionality.
 */
class dsp_client : public jack::client {
public:
  // Modulation schemes enumeration
  enum class ModulationScheme {
    SSB_USB,      // Single Sideband Upper Sideband
    SSB_LSB,      // Single Sideband Lower Sideband
    SSB_USB_SC,   // SSB USB Suppressed Carrier
    SSB_LSB_SC,   // SSB LSB Suppressed Carrier
    FSK_4         // 4-FSK
  };

  // Operating modes
  enum class Mode {
    Passthrough,  // Direct audio bypass
    Transmit,     // Modulation mode (WAV file input)
    Receive,      // Demodulation mode (microphone input)
    Stopped       // No processing
  };

  /**
   * The default constructor performs some basic connections.
   */
  dsp_client();
  
  /**
   * Destructor
   */
  ~dsp_client();

  /**
   * Initialize subclass-specific components
   */
  virtual bool init_subclass() override;

  /**
   * DSP functionality - main processing callback
   */
  virtual bool process(jack_nframes_t nframes,
                      const sample_t *const in,
                      sample_t *const out) override;

  /**
   * Return the last captured buffer
   */
  const std::vector<sample_t>& last_buffer();

  /**
   * Return the current average output power
   */
  float power() const;

  /**
   * Set the volume (0.0 to 2.0, where 1.0 is unity)
   */
  void set_volume(float vol);

  /**
   * Get current volume setting
   */
  float volume() const;

  /**
   * Set the current operating mode
   */
  void set_mode(Mode mode);

  /**
   * Set transmit carrier frequency in Hz
   */
  void set_transmit_carrier_freq(float freq);

  /**
   * Set receive carrier frequency in Hz
   */
  void set_receive_carrier_freq(float freq);

  /**
   * Set transmit modulation scheme
   */
  void set_transmit_modulation(ModulationScheme scheme);

  /**
   * Set receive modulation scheme
   */
  void set_receive_modulation(ModulationScheme scheme);

  /**
   * Start processing (enable modulation/demodulation)
   */
  void start_processing();

  /**
   * Stop processing
   */
  void stop_processing();

  /**
   * Initialize the difference equation oscillator
   * 
   * @param freq Oscillator frequency in Hz
   * @param amplitude Amplitude scaling factor
   * @param window_size_seconds Time window for energy calculation
   * @param signal_energy Total signal energy for normalization
   */
  void play_sine(float freq, float amplitude);

private:
  // Filter and buffers
  freq_filter _ffilter;
  prealloc_ringbuffer< std::vector<sample_t> > _past_buffers;
  
  // Power and volume
  float _power;
  float _volume;

  // Current mode and configuration
  Mode _current_mode;
  ModulationScheme _tx_modulation;
  ModulationScheme _rx_modulation;
  float _tx_carrier_freq;
  float _rx_carrier_freq;
  bool _processing_active;

  // Difference equation oscillator state (Proakis method)
  // y(n) = a1*y(n-1) - y(n-2)
  // where a1 = 2*cos(2π*f/Fs)
  float _osc_y_n_minus_1;   // y(n-1) - previous output
  float _osc_y_n_minus_2;   // y(n-2) - two samples ago
  float _osc_a1;            // 2*cos(2π*f/Fs) coefficient

  // Signal energy tracking
  float _signal_energy;

  // Legacy variables (kept for compatibility, may be removed later)
  bool modFSK;
  bool modOn;
  bool modSSB;
  bool passOn;
  float ganancia_actual;
  boost::circular_buffer<sample_t> cb_in;
  boost::circular_buffer<sample_t> cb_out;
  float sine_a_1;
  float last_sample;
  float last_last_sample;
  float sine_freq;
};

#endif // _DSP_CLIENT_H