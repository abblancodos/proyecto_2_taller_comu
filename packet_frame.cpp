#include "packet_frame.h"
#include "crc16.h"

namespace comm {

std::vector<uint8_t> PacketFrame::Header::to_bytes() const {
  std::vector<uint8_t> bytes(HEADER_SIZE);

  bytes[0] = magic;
  bytes[1] = static_cast<uint8_t>(payload_length >> 8);
  bytes[2] = static_cast<uint8_t>(payload_length & 0xFF);
  bytes[3] = static_cast<uint8_t>(sample_rate >> 24);
  bytes[4] = static_cast<uint8_t>((sample_rate >> 16) & 0xFF);
  bytes[5] = static_cast<uint8_t>((sample_rate >> 8) & 0xFF);
  bytes[6] = static_cast<uint8_t>(sample_rate & 0xFF);
  bytes[7] = channels;

  return bytes;
}

PacketFrame::Header PacketFrame::Header::from_bytes(const uint8_t *data) {
  Header h;

  h.magic = data[0];
  h.payload_length =
      (static_cast<uint16_t>(data[1]) << 8) | static_cast<uint16_t>(data[2]);
  h.sample_rate = (static_cast<uint32_t>(data[3]) << 24) |
                  (static_cast<uint32_t>(data[4]) << 16) |
                  (static_cast<uint32_t>(data[5]) << 8) |
                  static_cast<uint32_t>(data[6]);
  h.channels = data[7];

  return h;
}

std::vector<uint8_t> PacketFrame::generate_preamble() {
  std::vector<uint8_t> preamble;
  preamble.reserve(PREAMBLE_LENGTH);

  for (std::size_t i = 0; i < PREAMBLE_LENGTH; ++i) {
    preamble.push_back(PREAMBLE_PATTERN[i % PREAMBLE_PATTERN.size()]);
  }

  return preamble;
}

std::vector<uint8_t>
PacketFrame::create_frame(const std::vector<uint8_t> &payload,
                          uint32_t sample_rate, uint8_t channels) {
  std::vector<uint8_t> frame;

  // 1. Add preamble
  auto preamble = generate_preamble();
  frame.insert(frame.end(), preamble.begin(), preamble.end());

  // 2. Create and add header
  Header header(static_cast<uint16_t>(payload.size()), sample_rate, channels);
  auto header_bytes = header.to_bytes();
  frame.insert(frame.end(), header_bytes.begin(), header_bytes.end());

  // 3. Add payload
  frame.insert(frame.end(), payload.begin(), payload.end());

  // 4. Compute and append CRC over header + payload
  // CRC is computed starting after preamble
  std::vector<uint8_t> data_for_crc(frame.begin() + PREAMBLE_LENGTH,
                                    frame.end());
  CRC16::append(data_for_crc);

  // Replace frame data with CRC-appended version
  frame.resize(PREAMBLE_LENGTH);
  frame.insert(frame.end(), data_for_crc.begin(), data_for_crc.end());

  return frame;
}

bool PacketFrame::extract_payload(const std::vector<uint8_t> &frame,
                                  Header &header,
                                  std::vector<uint8_t> &payload) {
  // Minimum frame size: preamble + header + CRC
  if (frame.size() < PREAMBLE_LENGTH + HEADER_SIZE + CRC_SIZE) {
    return false;
  }

  // Extract data after preamble (header + payload + CRC)
  std::vector<uint8_t> data_with_crc(frame.begin() + PREAMBLE_LENGTH,
                                     frame.end());

  // Verify CRC
  if (!CRC16::verify(data_with_crc)) {
    return false;
  }

  // Extract header
  header = Header::from_bytes(data_with_crc.data());

  // Validate header
  if (!header.is_valid()) {
    return false;
  }

  // Check if payload length matches
  std::size_t expected_size = HEADER_SIZE + header.payload_length + CRC_SIZE;
  if (data_with_crc.size() != expected_size) {
    return false;
  }

  // Extract payload (between header and CRC)
  payload.assign(data_with_crc.begin() + HEADER_SIZE,
                 data_with_crc.end() - CRC_SIZE);

  return true;
}

bool PacketFrame::is_preamble(const std::vector<uint8_t> &symbols,
                              std::size_t offset) {
  if (offset + PREAMBLE_LENGTH > symbols.size()) {
    return false;
  }

  for (std::size_t i = 0; i < PREAMBLE_LENGTH; ++i) {
    uint8_t expected = PREAMBLE_PATTERN[i % PREAMBLE_PATTERN.size()];
    if (symbols[offset + i] != expected) {
      return false;
    }
  }

  return true;
}

int PacketFrame::find_preamble(const std::vector<uint8_t> &symbols) {
  if (symbols.size() < PREAMBLE_LENGTH) {
    return -1;
  }

  // Search for preamble pattern
  for (std::size_t i = 0; i <= symbols.size() - PREAMBLE_LENGTH; ++i) {
    if (is_preamble(symbols, i)) {
      return static_cast<int>(i);
    }
  }

  return -1;
}

} // namespace comm
