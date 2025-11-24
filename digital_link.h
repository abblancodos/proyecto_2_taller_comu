#ifndef DIGITAL_LINK_H
#define DIGITAL_LINK_H

#include "conv_codec.h"
#include "packet_frame.h"
#include <cstdint>
#include <sndfile.h>
#include <string>
#include <vector>

namespace comm {

class DigitalLink {
public:
  DigitalLink();

  // ========================
  //        TX  (WAV -> FSK)
  // ========================

  /// Load WAV file (mono/stereo -> mono)
  bool load_wav(const std::string &path);

  /// Load ANY binary file (TXT, PDF, images, etc.)
  bool load_binary_file(const std::string &path);

  /// Prepare payload: PCM -> bits -> FEC -> Frame (Preamble+Header+Payload+CRC)
  /// -> Symbols
  bool prepare_tx_payload();

  /// Is payload ready?
  bool tx_ready() const { return _tx_ready; }

  /// Is transmission done?
  bool tx_done() const;

  /// Get next symbol (0..3)
  int next_tx_symbol();

  /// Reset TX state
  void reset_tx();

  /// Get total symbols to transmit
  size_t get_tx_symbol_count() const { return _tx_symbols.size(); }

  // ========================
  //        RX  (FSK -> WAV)
  // ========================

  /// Reset RX state
  void reset_rx();

  /// Process received symbol (0..3)
  /// Returns true if a complete frame was detected and processed
  void process_rx_symbol(uint8_t symbol);

  /// Check if a frame has been successfully received and decoded
  bool frame_complete() const { return _frame_complete; }

  /// Save received audio to WAV
  bool save_received_wav(const std::string &path);

  /// Save received binary file (TXT, PDF, images, etc.)
  bool save_received_binary(const std::string &path);

  /// Getters for received metadata
  int get_sample_rate() const { return _rx_sample_rate; }
  int get_channels() const { return _rx_channels; }

  /// Get received binary data (for non-WAV files)
  const std::vector<uint8_t> &get_received_data() const {
    return _rx_binary_data;
  }

private:
  // Helpers
  void pcm_to_bits();
  void bits_to_pcm(const std::vector<uint8_t> &info_bits);

  // State
  int _sample_rate;
  int _channels;
  int _bits_per_sample;

  std::vector<int16_t> _pcm_samples;
  std::vector<int16_t> _pcm_rx;

  // TX
  std::vector<uint8_t> _tx_bits;
  std::vector<uint8_t> _tx_bits_enc;
  std::vector<uint8_t> _tx_symbols;
  size_t _tx_symbol_index;
  bool _tx_ready;

  // RX
  std::vector<uint8_t> _rx_symbol_buffer; // Buffer for incoming symbols
  bool _frame_complete;
  int _rx_sample_rate;
  int _rx_channels;
  std::vector<uint8_t> _rx_binary_data; // NEW: For non-WAV files

  // FEC
  conv::ConvolutionalEncoder _enc;
  conv::ViterbiDecoder _dec;
};

} // namespace comm

#endif // DIGITAL_LINK_H
