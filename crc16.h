#pragma once

#include <cstdint>
#include <vector>

namespace comm {

// CRC-16-CCITT for error detection beyond FEC
class CRC16 {
public:
  static uint16_t compute(const uint8_t *data, std::size_t length);
  static uint16_t compute(const std::vector<uint8_t> &data);

  static bool verify(const uint8_t *data, std::size_t length);
  static bool verify(const std::vector<uint8_t> &data);

  static void append(std::vector<uint8_t> &data);

private:
  static constexpr uint16_t POLYNOMIAL = 0x1021;
  static constexpr uint16_t INITIAL_VALUE = 0xFFFF;

  // Lookup table for fast CRC computation
  static const uint16_t crc_table[256];

  // Initialize lookup table
  static uint16_t make_crc_table_entry(uint8_t index);
};

} // namespace comm
