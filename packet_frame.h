#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace comm {

// Frame: [Preamble: 32 symbols] [Header: 8 bytes] [Payload: N bytes] [CRC: 2
// bytes]
class PacketFrame {
public:
  static constexpr uint8_t MAGIC_NUMBER = 0xA5;
  static constexpr std::size_t PREAMBLE_LENGTH = 32;
  static constexpr std::size_t HEADER_SIZE = 8;
  static constexpr std::size_t CRC_SIZE = 2;
  static constexpr std::array<uint8_t, 4> PREAMBLE_PATTERN = {0, 1, 2, 3};

  struct Header {
    uint8_t magic;
    uint16_t payload_length;
    uint32_t sample_rate;
    uint8_t channels;

    Header()
        : magic(MAGIC_NUMBER), payload_length(0), sample_rate(0), channels(0) {}
    Header(uint16_t len, uint32_t sr, uint8_t ch)
        : magic(MAGIC_NUMBER), payload_length(len), sample_rate(sr),
          channels(ch) {}

    std::vector<uint8_t> to_bytes() const;
    static Header from_bytes(const uint8_t *data);
    bool is_valid() const { return magic == MAGIC_NUMBER; }
  };

  static std::vector<uint8_t> generate_preamble();
  static std::vector<uint8_t> create_frame(const std::vector<uint8_t> &payload,
                                           uint32_t sample_rate,
                                           uint8_t channels);
  static bool extract_payload(const std::vector<uint8_t> &frame, Header &header,
                              std::vector<uint8_t> &payload);
  static bool is_preamble(const std::vector<uint8_t> &symbols,
                          std::size_t offset = 0);
  static int find_preamble(const std::vector<uint8_t> &symbols);
};

} // namespace comm
