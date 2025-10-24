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
#include "hilbert_fir.h"  // NEW: Add this include
#include <boost/circular_buffer.hpp>

class dsp_client : public jack::client {
public:
  enum class ModulationScheme {
    SSB_USB,      
    SSB_LSB,      
    SSB_USB_SC,   
    SSB_LSB_SC,   
    FSK_4         
  };

  enum class Mode {
    Passthrough,  
    Transmit,     
    Receive,      
    Stopped       
  };

  dsp_client();
  ~dsp_client();

  virtual bool init_subclass() override;
  virtual bool process(jack_nframes_t nframes,
                      const sample_t *const in,
                      sample_t *const out) override;

  const std::vector<sample_t>& last_buffer();
  float power() const;
  void set_volume(float vol);
  float volume() const;
  void set_mode(Mode mode);
  void set_transmit_carrier_freq(float freq);
  void set_receive_carrier_freq(float freq);
  void set_transmit_modulation(ModulationScheme scheme);
  void set_receive_modulation(ModulationScheme scheme);
  void start_processing();
  void stop_processing();
  void play_sine(float freq, float amplitude);

private:
  freq_filter _ffilter;
  prealloc_ringbuffer< std::vector<sample_t> > _past_buffers;
  
  float _power;
  float _volume;

  Mode _current_mode;
  ModulationScheme _tx_modulation;
  ModulationScheme _rx_modulation;
  float _tx_carrier_freq;
  float _rx_carrier_freq;
  bool _processing_active;

  // Difference equation oscillator state (Proakis method)
  float _osc_y_n_minus_1;   
  float _osc_y_n_minus_2;   
  float _osc_a1;            

  float _signal_energy;

  // NEW: SSB-specific members
  HilbertFIR* _hilbert_filter;           // Hilbert transform filter
  boost::circular_buffer<sample_t> _message_delay;  // Group delay compensation
  
  // NEW: Quadrature oscillator for sine (90° phase shift)
  float _osc_sin_y_n_minus_1;
  float _osc_sin_y_n_minus_2;
  float _osc_sin_a1;

  // Legacy variables
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