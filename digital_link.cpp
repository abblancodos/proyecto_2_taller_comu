#include "digital_link.h"
#include <algorithm>
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

  // Limit size for safety
  const size_t MAX_FRAMES = 48000 * 10; // 10 seconds max
  size_t frames_to_read = std::min((size_t)info.frames, MAX_FRAMES);

  std::vector<int16_t> temp_buf(frames_to_read * info.channels);
  sf_read_short(sf, temp_buf.data(), temp_buf.size());
  sf_close(sf);

  // Convert to mono if needed
  _pcm_samples.resize(frames_to_read);
  if (info.channels == 1) {
    _pcm_samples = temp_buf;
  } else {
    for (size_t i = 0; i < frames_to_read; ++i) {
      _pcm_samples[i] = temp_buf[i * info.channels]; // Take first channel
    }
  }

  std::cout << "[DigitalLink] Loaded " << _pcm_samples.size() << " samples ("
            << (double)frames_to_read / _sample_rate << "s)\n";
  return true;
}

/*
*
* Functional version
*
void DigitalLink::pcm_to_bits() {
  _tx_bits.clear();
  _tx_bits.reserve(_pcm_samples.size() * 16);

  for (int16_t sample : _pcm_samples) {
    // Little-endian transmission
    uint16_t val = static_cast<uint16_t>(sample);
    for (int i = 0; i < 16; ++i) {
      _tx_bits.push_back((val >> i) & 1);
    }
  }
}
*/

// Debug version
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
}

bool DigitalLink::prepare_tx_payload() {
  if (_pcm_samples.empty())
    return false;

  // 1. PCM -> Bits
  pcm_to_bits();

  // 2. FEC Encoding
  _enc.encode_block(_tx_bits, _tx_bits_enc);

  // 3. Create Packet Frame
  std::vector<uint8_t> frame_symbols =
      PacketFrame::create_frame(_tx_bits_enc, _sample_rate, _channels);

  // 4. Store as symbols to transmit
  _tx_symbols = frame_symbols;

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

  // Add symbol to buffer
  _rx_symbol_buffer.push_back(symbol);

  // Check for preamble if buffer is large enough
  if (_rx_symbol_buffer.size() >= PacketFrame::PREAMBLE_LENGTH) {
    int preamble_idx = PacketFrame::find_preamble(_rx_symbol_buffer);

    if (preamble_idx >= 0) {
      // Found preamble! Now check if we have enough data for header
      // size_t header_start = preamble_idx + PacketFrame::PREAMBLE_LENGTH;

      // We need at least preamble + header size (8 bytes * 4 symbols/byte = 32
      // symbols) Header is 8 bytes. Each byte is 4 symbols (FSK-4 encodes 2
      // bits/symbol -> 4 symbols/byte) Wait... PacketFrame::create_frame packs
      // bits into symbols. Let's check PacketFrame implementation. Assuming
      // standard packing.

      // Actually, let's just try to extract the payload if we have enough data
      // The extract_payload method handles the parsing

      PacketFrame::Header header;
      std::vector<uint8_t> payload;

      // We pass the whole buffer. The extract_payload should handle finding the
      // frame But wait, extract_payload expects the frame to start at index 0
      // or we need to slice it

      // Let's slice the buffer from the preamble start
      std::vector<uint8_t> potential_frame(
          _rx_symbol_buffer.begin() + preamble_idx, _rx_symbol_buffer.end());

      if (PacketFrame::extract_payload(potential_frame, header, payload)) {
        std::cout << "[DigitalLink] Frame detected and validated!\n";

        // Get metadata
        _rx_sample_rate = header.sample_rate;
        _rx_channels = header.channels;

        // Decode FEC
        std::vector<uint8_t> decoded_bits;
        _dec.decode_hard(payload, decoded_bits);

        // Bits -> PCM
        bits_to_pcm(decoded_bits);

        _frame_complete = true;
        _rx_symbol_buffer.clear(); // Clear buffer after successful decode
      }
    }

    // Prevent buffer from growing indefinitely if no preamble found
    if (_rx_symbol_buffer.size() > 100000) {
      // Keep last 100 symbols just in case preamble is being received
      std::vector<uint8_t> new_buf(_rx_symbol_buffer.end() - 100,
                                   _rx_symbol_buffer.end());
      _rx_symbol_buffer = new_buf;
    }
  }
}

void DigitalLink::bits_to_pcm(const std::vector<uint8_t> &info_bits) {
  _pcm_rx.clear();
  // 16 bits per sample
  size_t num_samples = info_bits.size() / 16;
  _pcm_rx.reserve(num_samples);

  for (size_t i = 0; i < num_samples; ++i) {
    uint16_t val = 0;
    for (int b = 0; b < 16; ++b) {
      if (info_bits[i * 16 + b]) {
        val |= (1 << b);
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
