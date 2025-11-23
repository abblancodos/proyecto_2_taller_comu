#include "digital_link.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>

namespace comm {

DigitalLink::DigitalLink()
    : _sample_rate(48000), _channels(1), _bits_per_sample(16),
      _tx_symbol_index(0), _tx_ready(false), _frame_complete(false),
      _rx_sample_rate(48000), _rx_channels(1) {}

// ============================================================
// TX Implementation
// ============================================================

bool DigitalLink::load_wav(const std::string &path) {
  SF_INFO info;
  std::memset(&info, 0, sizeof(info));

  SNDFILE *sf = sf_open(path.c_str(), SFM_READ, &info);
  if (!sf) {
    std::cerr << "[DigitalLink] Error opening WAV: " << sf_strerror(NULL)
              << "\n";
    return false;
  }

  _sample_rate = info.samplerate;
  _channels = info.channels;

  const size_t MAX_FRAMES = (size_t)_sample_rate * 10; // 10 seconds max REAL
  size_t frames_to_read = std::min((size_t)info.frames, MAX_FRAMES);

  // Leer como float (sirve para int16, int24, float32, etc.)
  std::vector<float> temp_buf(frames_to_read * info.channels);
  sf_count_t items_read = sf_read_float(sf, temp_buf.data(), temp_buf.size());
  sf_close(sf);

  if (items_read <= 0) {
    std::cerr << "[DigitalLink] No samples read from WAV!\n";
    return false;
  }

  size_t frames_read = (size_t)items_read / info.channels;

  // Convertir a mono
  _pcm_samples.resize(frames_read);

  if (info.channels == 1) {
    for (size_t i = 0; i < frames_read; ++i) {
      float x = temp_buf[i];
      // Clamp y escala a int16
      x = std::max(-1.0f, std::min(1.0f, x));
      _pcm_samples[i] = (int16_t)std::lrintf(x * 32767.0f);
    }
  } else {
    for (size_t i = 0; i < frames_read; ++i) {
      // promedio de canales
      float sum = 0.0f;
      for (int ch = 0; ch < info.channels; ++ch)
        sum += temp_buf[i * info.channels + ch];
      float x = sum / info.channels;
      x = std::max(-1.0f, std::min(1.0f, x));
      _pcm_samples[i] = (int16_t)std::lrintf(x * 32767.0f);
    }
  }

  // Debug snapshots
  std::cout << "[DigitalLink] Loaded " << _pcm_samples.size() << " samples ("
            << (double)frames_read / _sample_rate << "s)\n";
  std::cout << "  WAV format: " << info.format << "  channels=" << info.channels
            << "  sr=" << info.samplerate << "\n";

  // imprimir algunas muestras y pico
  int16_t peak = 0;
  for (size_t i = 0; i < std::min<size_t>(frames_read, 2000); ++i)
    peak = std::max<int16_t>(peak, std::abs(_pcm_samples[i]));

  std::cout << "  First 10 PCM samples: ";
  for (int i = 0; i < 10 && i < (int)_pcm_samples.size(); ++i)
    std::cout << _pcm_samples[i] << " ";
  std::cout << "\n";
  std::cout << "  Peak(abs) in first 2000 samples: " << peak << "\n";

  // DEBUG PCM stats
  int16_t mn = INT16_MAX, mx = INT16_MIN;
  long long sum_abs = 0;
  size_t nonzero = 0;

  for (size_t i = 0; i < _pcm_samples.size(); ++i) {
    int16_t v = _pcm_samples[i];
    mn = std::min(mn, v);
    mx = std::max(mx, v);
    sum_abs += std::abs((int)v);
    if (v != 0)
      nonzero++;
  }

  std::cout << "[DEBUG PCM] min=" << mn << " max=" << mx
            << " mean_abs=" << (double)sum_abs / _pcm_samples.size()
            << " nonzero%=" << (100.0 * nonzero / _pcm_samples.size()) << "%\n";

  // snapshot primeros 20 samples
  std::cout << "[DEBUG PCM] first 20: ";
  for (int i = 0; i < 20 && i < (int)_pcm_samples.size(); ++i)
    std::cout << _pcm_samples[i] << " ";
  std::cout << "\n";

  return true;
}

// Debug version + sanity checks
void DigitalLink::pcm_to_bits() {
  _tx_bits.clear();
  _tx_bits.reserve(_pcm_samples.size() * 16);

  const int DEBUG_SAMPLES = 5;
  int sample_index = 0;

  for (int16_t sample : _pcm_samples) {
    uint16_t val = static_cast<uint16_t>(sample);

    if (sample_index < DEBUG_SAMPLES) {
      std::cout << "\n[PCM->BITS] Sample " << sample_index
                << " (value = " << sample << ") -> bits: ";
    }

    for (int i = 0; i < 16; ++i) {
      uint8_t bit = (val >> i) & 1;
      _tx_bits.push_back(bit);

      if (sample_index < DEBUG_SAMPLES) {
        std::cout << (int)bit;
      }
    }

    if (sample_index < DEBUG_SAMPLES) {
      std::cout << "  (LSB→MSB)" << std::endl;
    }

    sample_index++;
  }

  std::cout << "[PCM->BITS] Total bits: " << _tx_bits.size() << std::endl;

  // ======================================================
  // SANITY CHECKS
  // ======================================================

  std::cout << "[DEBUG BITS] First 64 bits: ";
  int limit = std::min((int)_tx_bits.size(), 64);
  for (int i = 0; i < limit; ++i) {
    std::cout << (int)_tx_bits[i];
  }
  std::cout << std::endl;

  size_t ones = 0;
  for (uint8_t b : _tx_bits) {
    if (b)
      ones++;
  }
  double ones_pct = (_tx_bits.empty()) ? 0.0 : (100.0 * ones / _tx_bits.size());

  std::cout << "[DEBUG BITS] ones% = " << ones_pct << "%  "
            << "(zeros% = " << (100.0 - ones_pct) << "%)" << std::endl;

  if (ones == 0) {
    std::cout << "[DEBUG BITS] WARNING: all bits are ZERO -> PCM maybe all "
                 "zero or conversion broken\n";
  } else if (ones == _tx_bits.size()) {
    std::cout << "[DEBUG BITS] WARNING: all bits are ONE -> something is very "
                 "wrong\n";
  }
}

bool DigitalLink::prepare_tx_payload() {
  if (_pcm_samples.empty())
    return false;

  // RESET test
  _tx_ready = false;
  _tx_symbol_index = 0;
  _tx_bits.clear();
  _tx_bits_enc.clear();
  _tx_symbols.clear();

  // 1. PCM -> Bits
  pcm_to_bits();

  // 2. FEC Encoding
  _enc.encode_block(_tx_bits, _tx_bits_enc);

  size_t ones_enc = 0;
  for (auto b : _tx_bits_enc)
    if (b)
      ones_enc++;
  double ones_pct_enc = 100.0 * ones_enc / _tx_bits_enc.size();

  std::cout << "[DEBUG ENC] Encoded bits ones% = " << ones_pct_enc
            << "% size=" << _tx_bits_enc.size() << "\n";

  std::cout << "[DEBUG ENC] First 64 enc bits: ";
  for (int i = 0; i < 64 && i < (int)_tx_bits_enc.size(); ++i)
    std::cout << (int)_tx_bits_enc[i];
  std::cout << "\n";

  // 3. Create Packet Frame  (devuelve SÍMBOLOS 0..3)
  std::vector<uint8_t> frame_symbols =
      PacketFrame::create_frame(_tx_bits_enc, _sample_rate, _channels);

  // 4. Store as symbols to transmit
  _tx_symbols = frame_symbols;

  // DEBUG: print some symbols
  auto print_sym_at = [&](size_t idx) {
    if (idx < _tx_symbols.size())
      std::cout << "[DEBUG SYM] sym[" << idx << "]=" << (int)_tx_symbols[idx]
                << "\n";
  };

  print_sym_at(0);
  print_sym_at(10);
  print_sym_at(50);
  print_sym_at(60);
  print_sym_at(100);
  print_sym_at(1000);
  print_sym_at(10000);
  print_sym_at(100000);

  // DEBUG: symbol histogram (ahora sí sobre _tx_symbols real)
  size_t c0 = 0, c1 = 0, c2 = 0, c3 = 0;
  for (uint8_t s : _tx_symbols) {
    if (s == 0)
      c0++;
    else if (s == 1)
      c1++;
    else if (s == 2)
      c2++;
    else if (s == 3)
      c3++;
    else {
      std::cout << "[DEBUG SYM] INVALID symbol: " << (int)s << "\n";
    }
  }

  std::cout << "[DEBUG SYM] Histogram: "
            << "0=" << c0 << " 1=" << c1 << " 2=" << c2 << " 3=" << c3
            << " total=" << _tx_symbols.size() << "\n";

  std::cout << "[DEBUG SYM] First 40 symbols: ";
  for (int i = 0; i < 40 && i < (int)_tx_symbols.size(); ++i) {
    std::cout << (int)_tx_symbols[i] << " ";
  }
  std::cout << "\n";

  _tx_symbol_index = 0;
  _tx_ready = true;

  std::cout << "[DigitalLink] Payload prepared:\n"
            << "  PCM samples: " << _pcm_samples.size() << "\n"
            << "  Info bits: " << _tx_bits.size() << "\n"
            << "  Encoded bits: " << _tx_bits_enc.size() << "\n"
            << "  Total symbols (with frame): " << _tx_symbols.size() << "\n";

  return true;
}

bool DigitalLink::tx_done() const {
  return !_tx_ready || (_tx_symbol_index >= _tx_symbols.size());
}

int DigitalLink::next_tx_symbol() {
  if (tx_done())
    return -1;
  return _tx_symbols[_tx_symbol_index++];
}

void DigitalLink::reset_tx() {
  _tx_symbol_index = 0;
  _tx_ready = false;
  _pcm_samples.clear();
  _tx_bits.clear();
  _tx_bits_enc.clear();
  _tx_symbols.clear();
}

// ============================================================
// RX Implementation
// ============================================================

void DigitalLink::reset_rx() {
  _rx_symbol_buffer.clear();
  _frame_complete = false;
  _pcm_rx.clear();
}

void DigitalLink::process_rx_symbol(uint8_t symbol) {
  if (_frame_complete)
    return;

  _rx_symbol_buffer.push_back(symbol);

  // Aún no hay chance de preámbulo completo
  if (_rx_symbol_buffer.size() < PacketFrame::PREAMBLE_LENGTH)
    return;

  int preamble_idx = PacketFrame::find_preamble(_rx_symbol_buffer);
  if (preamble_idx < 0) {
    // Evitar crecimiento infinito del buffer
    if (_rx_symbol_buffer.size() > 100000) {
      _rx_symbol_buffer.erase(_rx_symbol_buffer.begin(),
                              _rx_symbol_buffer.end() - 100);
    }
    return;
  }

  // --- Tenemos preámbulo detectado ---
  size_t start = (size_t)preamble_idx;

  // 1) Ver si ya tenemos suficientes símbolos para leer header entero
  size_t header_syms = PacketFrame::HEADER_SIZE * 4; // 4 syms/byte
  size_t min_needed_for_header =
      start + PacketFrame::PREAMBLE_LENGTH + header_syms;

  if (_rx_symbol_buffer.size() < min_needed_for_header)
    return; // esperar más símbolos

  // 2) Cortar solo preámbulo+header en símbolos
  std::vector<uint8_t> pre_plus_header(_rx_symbol_buffer.begin() + start,
                                       _rx_symbol_buffer.begin() + start +
                                           PacketFrame::PREAMBLE_LENGTH +
                                           header_syms);

  // 3) Extraer header (sin CRC todavía) usando extract_payload “parcial”
  //    HACEMOS decoding manual solo para leer header bytes
  auto symbols_to_bits_fsk4 = [](const std::vector<uint8_t> &syms) {
    std::vector<uint8_t> bits;
    bits.reserve(syms.size() * 2);
    for (uint8_t s : syms) {
      bits.push_back(s & 1);
      bits.push_back((s >> 1) & 1);
    }
    return bits;
  };

  auto bits_to_bytes_lsb = [](const std::vector<uint8_t> &bits) {
    std::vector<uint8_t> bytes;
    bytes.reserve((bits.size() + 7) / 8);
    uint8_t cur = 0;
    int cnt = 0;
    for (uint8_t b : bits) {
      cur |= (b & 1) << cnt;
      cnt++;
      if (cnt == 8) {
        bytes.push_back(cur);
        cur = 0;
        cnt = 0;
      }
    }
    if (cnt != 0)
      bytes.push_back(cur);
    return bytes;
  };

  // símbolos después del preámbulo
  std::vector<uint8_t> header_syms_only(pre_plus_header.begin() +
                                            PacketFrame::PREAMBLE_LENGTH,
                                        pre_plus_header.end());

  std::vector<uint8_t> header_bits = symbols_to_bits_fsk4(header_syms_only);
  std::vector<uint8_t> header_bytes = bits_to_bytes_lsb(header_bits);

  if (header_bytes.size() < PacketFrame::HEADER_SIZE)
    return; // paranoia extra

  PacketFrame::Header header =
      PacketFrame::Header::from_bytes(header_bytes.data());

  if (!header.is_valid())
    return; // era falso preámbulo

  // 4) Calcular cuántos símbolos TOTAL tiene este frame completo
  size_t payload_syms = header.payload_length * 4;
  size_t crc_syms = PacketFrame::CRC_SIZE * 4;

  size_t total_frame_syms =
      PacketFrame::PREAMBLE_LENGTH + header_syms + payload_syms + crc_syms;

  size_t min_needed_for_full_frame = start + total_frame_syms;

  if (_rx_symbol_buffer.size() < min_needed_for_full_frame)
    return; // esperar más

  // 5) Cortar el frame EXACTO
  std::vector<uint8_t> full_frame(_rx_symbol_buffer.begin() + start,
                                  _rx_symbol_buffer.begin() + start +
                                      total_frame_syms);

  // 6) Ahora sí extraer payload real con CRC
  std::vector<uint8_t> payload_bits;
  PacketFrame::Header final_header;

  if (!PacketFrame::extract_payload(full_frame, final_header, payload_bits))
    return;

  std::cout << "[DigitalLink] Frame detected and validated!\n";

  _rx_sample_rate = final_header.sample_rate;
  _rx_channels = final_header.channels;

  // 7) FEC decode
  std::vector<uint8_t> decoded_bits;
  _dec.decode_hard(payload_bits, decoded_bits);

  // 8) Bits → PCM
  bits_to_pcm(decoded_bits);

  _frame_complete = true;
  _rx_symbol_buffer.clear();
}

void DigitalLink::bits_to_pcm(const std::vector<uint8_t> &info_bits) {
  _pcm_rx.clear();

  int bps = _bits_per_sample;
  if (bps <= 0)
    bps = 16;

  size_t num_samples = info_bits.size() / (size_t)bps;
  _pcm_rx.reserve(num_samples);

  for (size_t i = 0; i < num_samples; ++i) {
    uint16_t val = 0;
    for (int b = 0; b < bps && b < 16; ++b) {
      if (info_bits[i * (size_t)bps + b]) {
        val |= (1u << b);
      }
    }
    _pcm_rx.push_back(static_cast<int16_t>(val));
  }
}

bool DigitalLink::save_received_wav(const std::string &path) {
  if (!_frame_complete || _pcm_rx.empty())
    return false;

  SF_INFO info;
  std::memset(&info, 0, sizeof(info));
  info.samplerate = _rx_sample_rate;
  info.channels = _rx_channels;
  info.format = SF_FORMAT_WAV | SF_FORMAT_PCM_16;

  SNDFILE *sf = sf_open(path.c_str(), SFM_WRITE, &info);
  if (!sf) {
    std::cerr << "[DigitalLink] Error creating WAV: " << sf_strerror(NULL)
              << "\n";
    return false;
  }

  sf_write_short(sf, _pcm_rx.data(), _pcm_rx.size());
  sf_close(sf);

  return true;
}

} // namespace comm
