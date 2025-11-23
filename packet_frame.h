#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace comm {

class PacketFrame {
public:
  static constexpr uint8_t MAGIC = 0xA5;
  static constexpr size_t PREAMBLE_LENGTH = 32;
  static constexpr size_t CRC_SIZE = 2;

  // NUEVO tamaño: magic(1) + payload_len(4) + sample_rate(4) + channels(1) = 10
  static constexpr size_t HEADER_SIZE = 10;

  static constexpr std::array<uint8_t, 4> PREAMBLE_PATTERN = {0, 1, 2, 3};
  struct Header {
    uint8_t magic = MAGIC;
    uint32_t payload_length = 0;
    uint32_t sample_rate = 0;
    uint8_t channels = 0;

    Header() = default;
    Header(uint32_t len, uint32_t sr, uint8_t ch)
        : magic(MAGIC), payload_length(len), sample_rate(sr), channels(ch) {}

    std::vector<uint8_t> to_bytes() const;
    static Header from_bytes(const uint8_t *data);

    bool is_valid() const {
      return magic == MAGIC && channels >= 1 && channels <= 2 &&
             sample_rate > 8000 && sample_rate <= 192000;
    }
  };

  static std::vector<uint8_t> generate_preamble();
  static std::vector<uint8_t>
  create_frame(const std::vector<uint8_t> &payload_bytes, uint32_t sample_rate,
               uint8_t channels);

  static bool extract_payload(const std::vector<uint8_t> &frame, Header &header,
                              std::vector<uint8_t> &payload_bytes);

  static bool is_preamble(const std::vector<uint8_t> &symbols,
                          std::size_t offset);
  static int find_preamble(const std::vector<uint8_t> &symbols);
};

} // namespace comm
