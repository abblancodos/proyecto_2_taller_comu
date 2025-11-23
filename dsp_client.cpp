/**
 * dsp_client.cpp
 *
 * Copyright (C) 2023  Pablo Alvarado
 * EL5802 Procesamiento Digital de Se��ales
 * Escuela de Ingenier��a Electr��nica
 * Tecnol��gico de Costa Rica
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
#include "fir_filter.h"            // Unified FIR filter class
#include "hilbert_filter_coeffs.h" // Hilbert filter coefficients
#include "jack_client.h"
#include "lpf_demod_coeffs.h" // Low-pass filter coefficients
#include <boost/circular_buffer.hpp>
#include <cmath>
#include <cstring>
#include <iostream>
#include <numbers>

#include "digital_link.h"

/*
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
  , _tx_fsk4_frequencies{1000.0f, 2000.0f, 3000.0f, 4000.0f}  // NEW: TX FSK
freqs , _rx_fsk4_frequencies{1000.0f, 2000.0f, 3000.0f, 4000.0f}  // NEW: RX FSK
freqs
  // , _rx_controller(std::make_unique<DigitalRxController>())
  , _recovered_audio_buffer(48000)  // 1 segundo de buffer
  , _symbol_counter(0)
{
  // Initialize FSK buffers
  for (int i = 0; i < 2; ++i) {
    _fsk4_buffer[i].magnitudes.fill(0.0f);
    _fsk4_buffer[i].magnitudes_sq.fill(0.0f);
    _fsk4_buffer[i].strongest_index = 0;
  }
}
*/

dsp_client::dsp_client()
    : jack::client(), _ffilter(), _volume(1.0f), _current_mode(Mode::Stopped),
      _tx_modulation(ModulationScheme::SSB_USB),
      _rx_modulation(ModulationScheme::SSB_USB), _tx_carrier_freq(1000.0f),
      _rx_carrier_freq(1000.0f), _processing_active(false),
      _osc_y_n_minus_1(0.0f), _osc_y_n_minus_2(1.0f), _osc_a1(0.0f),
      _osc_sin_y_n_minus_1(1.0f), _osc_sin_y_n_minus_2(0.0f), _osc_sin_a1(0.0f),
      _hilbert_filter(nullptr), _lpf_demod(nullptr), _message_delay(0),
      _demod_delay(0), _power(0.0f), _wav_buffer_index(0),
      _fsk4_block_counter(0), _fsk4_current_buffer(0), _modulation_gain(20.0f)
      // , _rx_controller(std::make_unique<DigitalRxController>())
      ,
      _recovered_audio_buffer(48000) // Buffer de 1 segundo
      ,
      _symbol_counter(0) {
  // Initialize queue indices
  _queue_head.store(0);
  _queue_tail.store(0);

  // Inicializar frecuencias FSK por defecto
  _tx_fsk4_frequencies = {1000.0f, 2000.0f, 3000.0f, 4000.0f};
  _rx_fsk4_frequencies = {1000.0f, 2000.0f, 3000.0f, 4000.0f};
  _fsk4_frequencies = _tx_fsk4_frequencies;

  // Inicializar buffers FSK
  for (int i = 0; i < 2; ++i) {
    _fsk4_buffer[i].magnitudes.fill(0.0f);
    _fsk4_buffer[i].magnitudes_sq.fill(0.0f);
    _fsk4_buffer[i].strongest_index = 0;
  }

  std::cout << "[DSP Client] Inicializado con soporte FSK-4 completo"
            << std::endl;
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
                                          const std::vector<float> &proto) {
  _data.clear();
  _data.resize(cnt, proto);
  _idx = 0;
}

std::vector<float> &dsp_client::past_buffers_t::back() { return _data[_idx]; }

const std::vector<float> &dsp_client::past_buffers_t::back() const {
  return _data[_idx];
}

void dsp_client::past_buffers_t::push_back() {
  _idx = (_idx + 1) % _data.size();
}

bool dsp_client::init_subclass() {
  // Setup impulse response for filter
  std::unique_ptr<float[]> delta(new float[this->buffer_size()]);
  memset(delta.get(), 0, this->buffer_size() * sizeof(float));
  delta[0] = 1.0f;

  _ffilter.set_block_size(buffer_size());
  _ffilter.set_filter(delta.get(), buffer_size(), 2 * buffer_size() - 1);
  _power = 0.0f;

  // Initialize past buffers ring buffer
  const std::size_t total_buffers =
      static_cast<std::size_t>(std::ceil(0.5 * sample_rate() / buffer_size()));

  _past_buffers.allocate(total_buffers,
                         std::vector<float>(buffer_size(), float()));
  _past_buffers.push_back();

  // NEW: Initialize Hilbert filter (with optimization)
  _hilbert_filter = new FIRFilter(
      hilbert::HILBERT_COEFFS, hilbert::FILTER_LENGTH, FIRFilterType::HILBERT);

  /*
   // NEW: Initialize Hilbert filter (with optimization)
    _hilbert_filter = new FIRFilter(HILBERT_COEFFS, FILTER_LENGTH,
   FIRFilterType::HILBERT);
  */

  // NEW: Initialize Low-Pass Filter for demodulation (standard FIR)
  _lpf_demod = new FIRFilter(lpf_demod::LPF_COEFFS, lpf_demod::FILTER_LENGTH,
                             FIRFilterType::STANDARD);

  // NEW: Initialize message delay buffer (for group delay compensation)
  _message_delay.set_capacity(hilbert::GROUP_DELAY);
  for (size_t i = 0; i < hilbert::GROUP_DELAY; ++i) {
    _message_delay.push_back(0.0f);
  }

  /*
    // NEW: Initialize message delay buffer (for group delay compensation)
    _message_delay.set_capacity(GROUP_DELAY);
    for (size_t i = 0; i < GROUP_DELAY; ++i) {
        _message_delay.push_back(0.0f);
    }
  */

  // NEW: Initialize demodulation delay buffer (for LPF group delay
  // compensation)
  _demod_delay.set_capacity(lpf_demod::GROUP_DELAY);
  for (size_t i = 0; i < lpf_demod::GROUP_DELAY; ++i) {
    _demod_delay.push_back(0.0f);
  }

  // NEW: Initialize 4-FSK detector
  init_fsk4();

  // NEW: Initialize separate TX and RX FSK detectors
  _tx_fsk4_detector = std::make_unique<fsk4_detector>(
      _tx_fsk4_frequencies[0], _tx_fsk4_frequencies[1], _tx_fsk4_frequencies[2],
      _tx_fsk4_frequencies[3], sample_rate(), buffer_size(), 10);

  _rx_fsk4_detector = std::make_unique<fsk4_detector>(
      _rx_fsk4_frequencies[0], _rx_fsk4_frequencies[1], _rx_fsk4_frequencies[2],
      _rx_fsk4_frequencies[3], sample_rate(), buffer_size(), 10);

  std::cout << "TX FSK frequencies: " << _tx_fsk4_frequencies[0] << ", "
            << _tx_fsk4_frequencies[1] << ", " << _tx_fsk4_frequencies[2]
            << ", " << _tx_fsk4_frequencies[3] << " Hz\n";

  std::cout << "RX FSK frequencies: " << _rx_fsk4_frequencies[0] << ", "
            << _rx_fsk4_frequencies[1] << ", " << _rx_fsk4_frequencies[2]
            << ", " << _rx_fsk4_frequencies[3] << " Hz\n";

  return true;
}

/*
void dsp_client::init_fsk4() {
  _fsk4_detector = std::make_unique<fsk4_detector>(
    _fsk4_frequencies[0],
    _fsk4_frequencies[1],
    _fsk4_frequencies[2],
    _fsk4_frequencies[3],
    sample_rate(),
    buffer_size(),
    10  // Keep last 10 magnitude measurements
  );

  // Configure FSK transmitter
  _fsk_tx.samples_per_symbol = buffer_size();  // 1 symbol per block
  _fsk_tx.reset();

  std::cout << "4-FSK initialized: "
            << _fsk4_frequencies[0] << ", "
            << _fsk4_frequencies[1] << ", "
            << _fsk4_frequencies[2] << ", "
            << _fsk4_frequencies[3] << " Hz\n";
}
*/

void dsp_client::init_fsk4() {
  std::cout << "[DSP] Inicializando detectores FSK-4" << std::endl;

  // Asegurar que las frecuencias est�n configuradas
  if (_tx_fsk4_frequencies[0] == 0) {
    // Valores por defecto si no se han configurado
    _tx_fsk4_frequencies = {1000.0f, 2000.0f, 3000.0f, 4000.0f};
  }
  if (_rx_fsk4_frequencies[0] == 0) {
    // Deben coincidir con TX para loopback
    _rx_fsk4_frequencies = _tx_fsk4_frequencies;
  }

  // Inicializar detector de RX
  _rx_fsk4_detector = std::make_unique<fsk4_detector>(
      _rx_fsk4_frequencies[0], _rx_fsk4_frequencies[1], _rx_fsk4_frequencies[2],
      _rx_fsk4_frequencies[3], static_cast<float>(sample_rate()), buffer_size(),
      10 // buffer depth
  );

  // Inicializar detector de TX (para monitoreo)
  _tx_fsk4_detector = std::make_unique<fsk4_detector>(
      _tx_fsk4_frequencies[0], _tx_fsk4_frequencies[1], _tx_fsk4_frequencies[2],
      _tx_fsk4_frequencies[3], static_cast<float>(sample_rate()), buffer_size(),
      10);

  std::cout << "[DSP] FSK-4 inicializado - Frecuencias RX: ["
            << _rx_fsk4_frequencies[0] << ", " << _rx_fsk4_frequencies[1]
            << ", " << _rx_fsk4_frequencies[2] << ", "
            << _rx_fsk4_frequencies[3] << "] Hz" << std::endl;
}

bool dsp_client::process(jack_nframes_t nframes, const sample_t *const in,
                         sample_t *const out) {

  const sample_t *inptr = in;
  const sample_t *endptr = in + nframes;
  sample_t *outptr = out;

  // DEBUG: Verificar conexi�n TX->RX
  static int connection_check = 0;
  if (connection_check++ % 100 == 0) {
    std::cout << "[DSP] Modo: " << static_cast<int>(_current_mode)
              << ", TX->RX: out[0]=" << out[0] << ", in[0]=" << in[0]
              << ", nframes=" << nframes << std::endl;
  }

  // Process based on current mode - everything inline
  switch (_current_mode) {
  case Mode::Passthrough:
    // Direct passthrough
    {
      const sample_t *inptr = in;
      sample_t *outptr = out;
      const sample_t *const endptr = in + nframes;
      while (inptr != endptr) {
        *outptr++ = *inptr++ * _volume;
      }
    }
    break;

  case Mode::Transmit:
    if (_processing_active) {
      // Switch on transmit modulation scheme
      switch (_tx_modulation) {

      case ModulationScheme::FSK_4:
        // NEW: 4-FSK TRANSMIT
        process_fsk4_tx(out, nframes);
        break;

      case ModulationScheme::SSB_USB:
      case ModulationScheme::SSB_USB_SC:
        // SSB USB MODULATION
        // Input: message signal from WAV file
        // Output: SSB USB modulated signal
        while (inptr != endptr) {
          // Step 1: Get message sample
          float message = *inptr;

          // Step 2: Apply Hilbert transform to get m̂(t)
          float message_hilbert = _hilbert_filter->process(message);

          // Step 3: Delay original message by group delay to synchronize
          _message_delay.push_back(message);
          float message_delayed = _message_delay.front();

          // Step 4: Generate carrier signals using difference equation
          // oscillators Cosine: cos(ωc*t)
          float carrier_cos = -_osc_a1 * _osc_y_n_minus_1 - _osc_y_n_minus_2;
          _osc_y_n_minus_2 = _osc_y_n_minus_1;
          _osc_y_n_minus_1 = carrier_cos;

          // Sine: sin(ωc*t) - uses separate oscillator with 90° phase shift
          float carrier_sin =
              -_osc_sin_a1 * _osc_sin_y_n_minus_1 - _osc_sin_y_n_minus_2;
          _osc_sin_y_n_minus_2 = _osc_sin_y_n_minus_1;
          _osc_sin_y_n_minus_1 = carrier_sin;

          // Step 5: USB modulation
          // USB: s(t) = m(t)*cos(ωc*t) - m̂(t)*sin(ωc*t)
          float modulated =
              message_delayed * carrier_cos - message_hilbert * carrier_sin;

          // Step 6: Apply modulation gain and volume
          *outptr = modulated * _modulation_gain * _volume;

          ++inptr;
          ++outptr;
        }
        break;

      case ModulationScheme::SSB_LSB:
      case ModulationScheme::SSB_LSB_SC:
        // SSB LSB MODULATION
        // LSB: s(t) = m(t)*cos(ωc*t) + m̂(t)*sin(ωc*t)
        while (inptr != endptr) {
          float message = *inptr;
          float message_hilbert = _hilbert_filter->process(message);
          _message_delay.push_back(message);
          float message_delayed = _message_delay.front();

          float carrier_cos = -_osc_a1 * _osc_y_n_minus_1 - _osc_y_n_minus_2;
          _osc_y_n_minus_2 = _osc_y_n_minus_1;
          _osc_y_n_minus_1 = carrier_cos;

          float carrier_sin =
              -_osc_sin_a1 * _osc_sin_y_n_minus_1 - _osc_sin_y_n_minus_2;
          _osc_sin_y_n_minus_2 = _osc_sin_y_n_minus_1;
          _osc_sin_y_n_minus_1 = carrier_sin;

          // LSB uses + instead of -
          float modulated =
              message_delayed * carrier_cos + message_hilbert * carrier_sin;
          *outptr = modulated * _modulation_gain * _volume;

          ++inptr;
          ++outptr;
        }
        break;

      default:
        // Unknown modulation scheme - output silence
        memset(out, 0, nframes * sizeof(sample_t));
        break;
      }
    } else {
      // Not processing: output silence
      std::fill(out, out + nframes, 0.0f);
    }
    break;

  case Mode::Receive:
    // NEW: For FSK-4, do TX+RX simultaneously for loopback testing
    if (_rx_modulation == ModulationScheme::FSK_4 && _processing_active) {
      // First, generate TX signal (Loopback)
      process_fsk4_tx(out, nframes);

      // INTERNAL LOOPBACK: Use 'out' (TX signal) directly as input for RX
      // This bypasses JACK input latency and connection issues
      process_fsk4_rx(out, nframes);
    } else if (_processing_active) {
      // Original RX-only code for SSB modes
      switch (_rx_modulation) {
      case ModulationScheme::FSK_4:
        // Procesar deteccin FSK
        process_fsk4_rx(in, nframes);

        // Reproducir audio recuperado si est� disponible
        if (!_recovered_audio_buffer.empty()) {
          size_t samples_to_copy = std::min(static_cast<size_t>(nframes),
                                            _recovered_audio_buffer.size());

          for (size_t i = 0; i < samples_to_copy; ++i) {
            out[i] = _recovered_audio_buffer.front() * _volume;
            _recovered_audio_buffer.pop_front();
          }

          // Rellenar con ceros si no hay suficientes muestras
          for (size_t i = samples_to_copy; i < nframes; ++i) {
            out[i] = 0.0f;
          }

          // Debug cada cierto tiempo
          static int counter = 0;
          if (++counter % 100 == 0) {
            std::cout << "[RX] Reproduciendo audio recuperado, buffer: "
                      << _recovered_audio_buffer.size() << " muestras"
                      << std::endl;
          }
        } else {
          // No hay audio recuperado a�n, silencio
          std::fill(out, out + nframes, 0.0f);
        }
        break;

      case ModulationScheme::SSB_USB:
      case ModulationScheme::SSB_USB_SC:
        // SSB USB DEMODULATION
        // Input: received SSB USB signal from microphone
        // Output: demodulated message signal
        while (inptr != endptr) {
          // Step 1: Get received SSB signal
          float received = *inptr;

          // Step 2: Generate local oscillator (coherent carrier)
          float local_cos = -_osc_a1 * _osc_y_n_minus_1 - _osc_y_n_minus_2;
          _osc_y_n_minus_2 = _osc_y_n_minus_1;
          _osc_y_n_minus_1 = local_cos;

          // Step 3: Coherent detection (multiply by 2x carrier)
          // This shifts the SSB signal back to baseband plus a high-freq
          // component
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
        while (inptr != endptr) {
          // Step 1: Get received SSB signal
          float received = *inptr;

          // Step 2: Generate local oscillator (coherent carrier)
          float local_cos = -_osc_a1 * _osc_y_n_minus_1 - _osc_y_n_minus_2;
          _osc_y_n_minus_2 = _osc_y_n_minus_1;
          _osc_y_n_minus_1 = local_cos;

          // Step 3: Coherent detection (multiply by 2x carrier)
          // This shifts the SSB signal back to baseband plus a high-freq
          // component
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
  const sample_t *energy_ptr = out;
  const sample_t *energy_end = out + nframes;
  while (energy_ptr != energy_end) {
    float sample = *energy_ptr;
    block_energy += sample * sample;
    ++energy_ptr;
  }
  block_energy /= nframes;

  // Apply low pass filter to block energy
  constexpr float c = 0.6f;
  _power = std::max(block_energy * c + (1.0f - c) * _power, block_energy);

  // Save output for GUI display
  _past_buffers.push_back();
  memcpy(_past_buffers.back().data(), out, nframes * sizeof(sample_t));

  return true;
}

// ============================================================================
// NEW: FSK RECEIVE - Uses RX frequencies and RX detector
// ============================================================================

/*
void dsp_client::process_fsk4_rx(const sample_t* in, std::size_t nframes) {
  if (!_rx_fsk4_detector) {
    return;
  }

  // Detectar frecuencias FSK
  _rx_fsk4_detector->process_block(in, nframes);

  // Obtener magnitudes y s�mbolo m�s fuerte
  auto mags = _rx_fsk4_detector->get_current_magnitudes_squared();
  int strongest = _rx_fsk4_detector->get_strongest_frequency_index();

  // Actualizar buffer at�mico para visualizaci�n
  int write_idx = 1 - _fsk4_current_buffer.load();
  _fsk4_buffer[write_idx].magnitudes =
_rx_fsk4_detector->get_current_magnitudes();
  _fsk4_buffer[write_idx].magnitudes_sq = mags;
  _fsk4_buffer[write_idx].strongest_index = strongest;
  _fsk4_current_buffer.store(write_idx);
  _fsk4_block_counter.fetch_add(1);

  // A�ADIR: Enviar s�mbolo al controlador de recepci�n
  if (_rx_controller) {
    _rx_controller->add_symbol(strongest);

    // Verificar si hay audio recuperado
    if (_rx_controller->has_audio_ready()) {
      std::vector<float> recovered;
      _rx_controller->get_audio(recovered);

      // A�adir al buffer circular
      for (float sample : recovered) {
        _recovered_audio_buffer.push_back(sample);
      }

      std::cout << "[RX] Audio recuperado: " << recovered.size()
                << " muestras, buffer: " << _recovered_audio_buffer.size() <<
std::endl;
    }
  }
}
*/

/*
void dsp_client::process_fsk4_rx(const sample_t *in, std::size_t nframes) {
  for (std::size_t i = 0; i < nframes; ++i) {
    // float sample = in[i];
    if (_current_mode == Mode::Receive) {
      _fsk4_detector->process_block(in, nframes);
    }
  }
}
*/

void dsp_client::process_fsk4_rx(const sample_t *in, std::size_t nframes) {
  if (_current_mode != Mode::Receive || !_rx_fsk4_detector) {
    return;
  }

  // Procesar UNA vez el bloque completo
  _rx_fsk4_detector->process_block(in, nframes);

  // NUEVO: Obtener símbolo detectado y actualizar buffer atómico
  auto mags = _rx_fsk4_detector->get_current_magnitudes_squared();
  int strongest = _rx_fsk4_detector->get_strongest_frequency_index();

  // Actualizar buffer atómico para GUI (lock-free)
  int write_idx = 1 - _fsk4_current_buffer.load();
  _fsk4_buffer[write_idx].magnitudes =
      _rx_fsk4_detector->get_current_magnitudes();
  _fsk4_buffer[write_idx].magnitudes_sq = mags;
  _fsk4_buffer[write_idx].strongest_index = strongest;
  _fsk4_current_buffer.store(write_idx);
  _fsk4_block_counter.fetch_add(1);

  // NEW: Push to symbol queue
  size_t head = _queue_head.load(std::memory_order_relaxed);
  size_t next_head = (head + 1) % SYMBOL_QUEUE_SIZE;

  if (next_head != _queue_tail.load(std::memory_order_acquire)) {
    _symbol_queue[head] = static_cast<uint8_t>(strongest);
    _queue_head.store(next_head, std::memory_order_release);
  } else {
    // Queue full - drop symbol (should warn?)
  }
}

std::vector<uint8_t> dsp_client::get_rx_symbols() {
  std::vector<uint8_t> symbols;
  size_t tail = _queue_tail.load(std::memory_order_relaxed);
  size_t head = _queue_head.load(std::memory_order_acquire);

  if (tail == head) {
    return symbols;
  }

  // Calculate size
  size_t count =
      (head >= tail) ? (head - tail) : (SYMBOL_QUEUE_SIZE - tail + head);
  symbols.reserve(count);

  while (tail != head) {
    symbols.push_back(_symbol_queue[tail]);
    tail = (tail + 1) % SYMBOL_QUEUE_SIZE;
  }

  _queue_tail.store(tail, std::memory_order_release);
  return symbols;
}

// ============================================================================
// NEW: FSK TRANSMIT - Uses TX frequencies
// ============================================================================

void dsp_client::process_fsk4_tx(sample_t *out, std::size_t nframes) {
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

void dsp_client::set_tx_fsk4_frequencies(float f1, float f2, float f3,
                                         float f4) {
  _tx_fsk4_frequencies = {f1, f2, f3, f4};

  // Update TX detector if it exists
  if (_tx_fsk4_detector) {
    _tx_fsk4_detector->set_frequencies(f1, f2, f3, f4);
  }

  std::cout << "TX FSK frequencies updated: " << f1 << ", " << f2 << ", " << f3
            << ", " << f4 << " Hz" << std::endl;
}

void dsp_client::set_rx_fsk4_frequencies(float f1, float f2, float f3,
                                         float f4) {
  _rx_fsk4_frequencies = {f1, f2, f3, f4};

  // Update RX detector if it exists
  if (_rx_fsk4_detector) {
    _rx_fsk4_detector->set_frequencies(f1, f2, f3, f4);
  }

  std::cout << "RX FSK frequencies updated: " << f1 << ", " << f2 << ", " << f3
            << ", " << f4 << " Hz" << std::endl;
}

void dsp_client::set_fsk_symbol(int symbol) {
  if (symbol >= 0 && symbol < 4) {
    // Only reset if symbol actually changed
    if (_fsk_tx.current_symbol != symbol) {
      _fsk_tx.current_symbol = symbol;
      _fsk_tx.samples_in_symbol = 0; // CRITICAL: Reset counter for new symbol
      // Note: We don't reset phase to maintain phase continuity (CPM)
    }
  }
}

// ============================================================================
// NEW: RX Controller Access Methods
// ============================================================================

void dsp_client::play_sine(float freq, float amplitude) {
  // Initialize BOTH cosine and sine oscillators
  const float norm_freq = freq / sample_rate();
  const float avg_signal_power = (1.0f / (2.0f * sample_rate() + 1.0f) * 1000);

  // Coefficient: a1 = -2*cos(2��*f/Fs)
  _osc_a1 = -2.0f * std::cos(2.0f * std::numbers::pi_v<float> * norm_freq);
  _osc_sin_a1 = _osc_a1; // Same coefficient for both

  // Initialize cosine oscillator: cos(��n)
  // y(-2) = cos(-2��) = cos(2��)
  // y(-1) = cos(-��) = cos(��)
  _osc_y_n_minus_2 = amplitude * std::sqrt(avg_signal_power) *
                     std::cos(2.0f * std::numbers::pi_v<float> * norm_freq);
  _osc_y_n_minus_1 = amplitude * std::sqrt(avg_signal_power) *
                     std::cos(std::numbers::pi_v<float> * norm_freq);

  // Initialize sine oscillator: sin(��n) with 90° phase shift
  // y(-2) = sin(-2��) = -sin(2��)
  // y(-1) = sin(-��) = -sin(��)
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

  std::cout << "Modulation gain set to: " << _modulation_gain << "x"
            << std::endl;
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
  float freq =
      (_current_mode == Mode::Transmit) ? _tx_carrier_freq : _rx_carrier_freq;
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

const std::vector<dsp_client::sample_t> &dsp_client::last_buffer() {
  return _past_buffers.back();
}

float dsp_client::power() const { return _power; }

void dsp_client::set_volume(float vol) { _volume = vol * vol; }

float dsp_client::volume() const { return std::sqrt(_volume); }

// ============================================================================
// WAV Buffer Management
// ============================================================================

void dsp_client::set_wav_buffer(const float *samples, size_t num_samples) {
  _current_wav_buffer.assign(samples, samples + num_samples);
  _wav_buffer_index = 0;
  std::cout << "WAV buffer loaded with " << num_samples << " samples"
            << std::endl;
}

void dsp_client::clear_wav_buffer() {
  _current_wav_buffer.clear();
  _wav_buffer_index = 0;
  std::cout << "WAV buffer cleared" << std::endl;
}

// ============================================================================
// Recovered Audio Access
// ============================================================================

bool dsp_client::get_recovered_audio(std::vector<float> &audio) {
  if (_recovered_audio_buffer.empty()) {
    return false;
  }

  // Copy available audio from circular buffer
  audio.clear();
  audio.reserve(_recovered_audio_buffer.size());

  for (const auto &sample : _recovered_audio_buffer) {
    audio.push_back(sample);
  }

  std::cout << "Retrieved " << audio.size() << " samples of recovered audio"
            << std::endl;
  return true;
}

// ============================================================================
// Legacy FSK Configuration (for backward compatibility)
// ============================================================================

void dsp_client::set_fsk4_frequencies(float f1, float f2, float f3, float f4) {
  // Set both TX and RX to same frequencies for backward compatibility
  set_tx_fsk4_frequencies(f1, f2, f3, f4);
  set_rx_fsk4_frequencies(f1, f2, f3, f4);

  // Also update the legacy member variable
  _fsk4_frequencies = {f1, f2, f3, f4};

  std::cout << "Legacy FSK frequencies updated (both TX/RX): " << f1 << ", "
            << f2 << ", " << f3 << ", " << f4 << " Hz" << std::endl;
}

// ============================================================================

bool dsp_client::connect_ports(const std::string &source,
                               const std::string &destination) {
  jack_client_t *jack_client = get_jack_client();
  if (!jack_client) {
    std::cout << "[JACK] Error: Cliente JACK no inicializado" << std::endl;
    return false;
  }

  int result = jack_connect(jack_client, source.c_str(), destination.c_str());
  if (result == 0) {
    std::cout << "[JACK] Puertos conectados: " << source << " -> "
              << destination << std::endl;
    return true;
  } else if (result == EEXIST) {
    std::cout << "[JACK] Puertos ya conectados: " << source << " -> "
              << destination << std::endl;
    return true;
  } else {
    std::cout << "[JACK] Error conectando puertos: " << source << " -> "
              << destination << " (error: " << result << ")" << std::endl;
    return false;
  }
}

bool dsp_client::is_connected() const { return get_jack_client() != nullptr; }

void dsp_client::reset_rx_controller() {
  // No-op
}
