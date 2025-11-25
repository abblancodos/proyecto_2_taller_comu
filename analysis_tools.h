#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace comm {

/**
 * Bit Error Rate Calculator
 * Compares transmitted vs received bits to compute BER
 */
class BERCalculator {
public:
  BERCalculator() : _total_bits(0), _error_bits(0) {}

  // Add transmitted bits (ground truth)
  void add_tx_bits(const std::vector<uint8_t> &bits) {
    _tx_bits.insert(_tx_bits.end(), bits.begin(), bits.end());
  }

  // Add received bits and compare with TX
  void add_rx_bits(const std::vector<uint8_t> &bits) {
    size_t start = _rx_bits.size();
    _rx_bits.insert(_rx_bits.end(), bits.begin(), bits.end());

    // Compare overlapping region
    for (size_t i = start; i < _rx_bits.size() && i < _tx_bits.size(); ++i) {
      _total_bits++;
      if (_rx_bits[i] != _tx_bits[i]) {
        _error_bits++;
      }
    }
  }

  float get_ber() const {
    return (_total_bits > 0) ? (float)_error_bits / _total_bits : 0.0f;
  }

  size_t get_total_bits() const { return _total_bits; }
  size_t get_error_bits() const { return _error_bits; }

  void reset() {
    _tx_bits.clear();
    _rx_bits.clear();
    _total_bits = 0;
    _error_bits = 0;
  }

private:
  std::vector<uint8_t> _tx_bits;
  std::vector<uint8_t> _rx_bits;
  size_t _total_bits;
  size_t _error_bits;
};

/**
 * Eye Diagram Buffer
 * Stores multiple symbol traces for visualization
 */
class EyeDiagram {
public:
  static constexpr size_t MAX_TRACES = 100;

  EyeDiagram(size_t samples_per_symbol)
      : _samples_per_symbol(samples_per_symbol), _trace_index(0) {
    _traces.resize(MAX_TRACES);
    for (auto &trace : _traces) {
      trace.resize(samples_per_symbol);
    }
  }

  void add_symbol_trace(const float *samples) {
    for (size_t i = 0; i < _samples_per_symbol; ++i) {
      _traces[_trace_index][i] = samples[i];
    }
    _trace_index = (_trace_index + 1) % MAX_TRACES;
  }

  const std::vector<std::vector<float>> &get_traces() const { return _traces; }
  size_t get_samples_per_symbol() const { return _samples_per_symbol; }

private:
  std::vector<std::vector<float>> _traces;
  size_t _samples_per_symbol;
  size_t _trace_index;
};

/**
 * Data Rate Calculator
 * Tracks transmission statistics
 */
class DataRateCalculator {
public:
  DataRateCalculator()
      : _start_time(std::chrono::steady_clock::now()), _total_symbols(0),
        _total_bits(0) {}

  void start() { _start_time = std::chrono::steady_clock::now(); }

  void add_symbols(size_t count) {
    _total_symbols += count;
    _total_bits += count * 2; // FSK-4: 2 bits per symbol
  }

  float get_symbol_rate() const {
    auto now = std::chrono::steady_clock::now();
    float elapsed = std::chrono::duration<float>(now - _start_time).count();
    return (elapsed > 0) ? _total_symbols / elapsed : 0.0f;
  }

  float get_data_rate_bps() const {
    auto now = std::chrono::steady_clock::now();
    float elapsed = std::chrono::duration<float>(now - _start_time).count();
    return (elapsed > 0) ? _total_bits / elapsed : 0.0f;
  }

  size_t get_total_symbols() const { return _total_symbols; }
  size_t get_total_bits() const { return _total_bits; }

private:
  std::chrono::steady_clock::time_point _start_time;
  size_t _total_symbols;
  size_t _total_bits;
};

} // namespace comm
