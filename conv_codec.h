#pragma once

#include <cstdint>
#include <vector>

namespace conv {

// Convolutional code: rate 1/2, K=7, polynomials (171,133) octal
class ConvolutionalEncoder {
public:
  using bit_t = uint8_t;

  ConvolutionalEncoder();
  void reset();

  void encode_block(const std::vector<bit_t> &in_bits,
                    std::vector<bit_t> &out_bits, bool add_tail_bits = true);

  void encode_block(const bit_t *in_bits, std::size_t n_bits,
                    std::vector<bit_t> &out_bits, bool add_tail_bits = true);

private:
  static constexpr bit_t G0 = 0b1111001;
  static constexpr bit_t G1 = 0b1011011;
  static constexpr int K = 7;
  static constexpr int M = K - 1;
  static constexpr int NSTATES = 1 << M;

  struct TrellisTables {
    uint8_t next_state[NSTATES][2];
    uint8_t out_sym[NSTATES][2];
    TrellisTables();
  };

  static const TrellisTables &trellis();
  uint8_t _state;
};

// Viterbi decoder (hard-decision)
class ViterbiDecoder {
public:
  using bit_t = uint8_t;

  ViterbiDecoder() = default;

  void decode_hard(const std::vector<bit_t> &rx_bits,
                   std::vector<bit_t> &out_bits, bool terminated = true) const;

private:
  static constexpr bit_t G0 = 0b1111001;
  static constexpr bit_t G1 = 0b1011011;
  static constexpr int K = 7;
  static constexpr int M = K - 1;
  static constexpr int NSTATES = 1 << M;

  struct TrellisTables {
    uint8_t next_state[NSTATES][2];
    uint8_t out_sym[NSTATES][2];
    TrellisTables();
  };

  static const TrellisTables &trellis();
};

/**
 * Clase simple para llevar conteo de BER (observabilidad).
 */
class BerCounter {
public:
  using bit_t = uint8_t;

  void reset() {
    _errors = 0;
    _total = 0;
  }

  void update(bit_t ref, bit_t est) {
    if (ref != est)
      ++_errors;
    ++_total;
  }

  void update_block(const std::vector<bit_t> &ref,
                    const std::vector<bit_t> &est, std::size_t n);

  std::uint64_t errors() const { return _errors; }
  std::uint64_t total() const { return _total; }

  double ber() const {
    return (_total > 0) ? double(_errors) / double(_total) : 0.0;
  }

private:
  std::uint64_t _errors{0};
  std::uint64_t _total{0};
};

} // namespace conv
