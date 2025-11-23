#include "packet_frame.h"
#include "crc16.h"
#include <algorithm>
#include <iostream>

namespace comm {

// Definición del patrón (asumiendo que lo declaraste en el .h como:
// static constexpr std::array<uint8_t, 4> PREAMBLE_PATTERN;)
// constexpr std::array<uint8_t, 4> PacketFrame::PREAMBLE_PATTERN{{0, 1, 2, 3}};

std::vector<uint8_t> PacketFrame::Header::to_bytes() const {
  std::vector<uint8_t> bytes(HEADER_SIZE);

  bytes[0] = magic;

  // payload_length (uint32_t) big-endian
  bytes[1] = static_cast<uint8_t>(payload_length >> 24);
  bytes[2] = static_cast<uint8_t>((payload_length >> 16) & 0xFF);
  bytes[3] = static_cast<uint8_t>((payload_length >> 8) & 0xFF);
  bytes[4] = static_cast<uint8_t>(payload_length & 0xFF);

  // sample_rate (uint32_t) big-endian
  bytes[5] = static_cast<uint8_t>(sample_rate >> 24);
  bytes[6] = static_cast<uint8_t>((sample_rate >> 16) & 0xFF);
  bytes[7] = static_cast<uint8_t>((sample_rate >> 8) & 0xFF);
  bytes[8] = static_cast<uint8_t>(sample_rate & 0xFF);

  bytes[9] = channels;

  return bytes;
}

PacketFrame::Header PacketFrame::Header::from_bytes(const uint8_t *data) {
  Header h;

  h.magic = data[0];

  h.payload_length = (static_cast<uint32_t>(data[1]) << 24) |
                     (static_cast<uint32_t>(data[2]) << 16) |
                     (static_cast<uint32_t>(data[3]) << 8) |
                     static_cast<uint32_t>(data[4]);

  h.sample_rate = (static_cast<uint32_t>(data[5]) << 24) |
                  (static_cast<uint32_t>(data[6]) << 16) |
                  (static_cast<uint32_t>(data[7]) << 8) |
                  static_cast<uint32_t>(data[8]);

  h.channels = data[9];

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

  // =========================
  // Helpers locales (LSB-first)
  // =========================
  auto bits_to_bytes_lsb = [](const std::vector<uint8_t> &bits) {
    std::vector<uint8_t> bytes;
    bytes.reserve((bits.size() + 7) / 8);

    uint8_t cur = 0;
    int cnt = 0;

    for (uint8_t b : bits) {
      cur |= (b & 1) << cnt; // LSB-first
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

  auto bytes_to_bits_lsb = [](const std::vector<uint8_t> &bytes) {
    std::vector<uint8_t> bits;
    bits.reserve(bytes.size() * 8);
    for (uint8_t v : bytes) {
      for (int i = 0; i < 8; ++i) {
        bits.push_back((v >> i) & 1); // LSB-first
      }
    }
    return bits;
  };

  auto bits_to_symbols_fsk4 = [](const std::vector<uint8_t> &bits) {
    std::vector<uint8_t> syms;
    syms.reserve((bits.size() + 1) / 2);

    for (size_t i = 0; i < bits.size(); i += 2) {
      uint8_t b0 = bits[i] & 1;
      uint8_t b1 = (i + 1 < bits.size()) ? (bits[i + 1] & 1) : 0;
      uint8_t sym = b0 | (b1 << 1); // 2 bits -> símbolo 0..3 (LSB-first)
      syms.push_back(sym);
    }
    return syms;
  };

  // =========================
  // DEBUG de entrada
  // =========================
  std::cout << "[PacketFrame] create_frame() ENTER\n";
  std::cout << "  payload(bits).size() = " << payload.size() << "\n";
  if (!payload.empty()) {
    std::cout << "  payload first 32 bits: ";
    for (int i = 0; i < 32 && i < (int)payload.size(); ++i) {
      std::cout << (int)payload[i] << " ";
    }
    std::cout << "\n";
  }

  // 0) Payload bits -> payload bytes (para CRC/length)
  std::vector<uint8_t> payload_bytes = bits_to_bytes_lsb(payload);

  // 1) Add preamble (ya son símbolos 0..3)
  auto preamble = generate_preamble();
  frame.insert(frame.end(), preamble.begin(), preamble.end());

  std::cout << "[PacketFrame] After preamble\n";
  std::cout << "  preamble.size() = " << preamble.size() << "\n";
  std::cout << "  frame.size()    = " << frame.size() << "\n";

  // 2) Header en bytes (payload_length en BYTES)
  Header header(static_cast<uint32_t>(payload_bytes.size()), sample_rate,
                channels);
  auto header_bytes = header.to_bytes();
  std::vector<uint8_t> data_bytes;
  data_bytes.reserve(header_bytes.size() + payload_bytes.size() + CRC_SIZE);

  data_bytes.insert(data_bytes.end(), header_bytes.begin(), header_bytes.end());
  data_bytes.insert(data_bytes.end(), payload_bytes.begin(),
                    payload_bytes.end());

  std::cout << "[PacketFrame] After header\n";
  std::cout << "  header_bytes.size() = " << header_bytes.size() << "\n";
  std::cout << "  header bytes: ";
  for (int i = 0; i < (int)header_bytes.size(); ++i) {
    std::cout << (int)header_bytes[i] << " ";
  }
  std::cout << "\n";
  std::cout << "  data_bytes.size() (header+payload) = " << data_bytes.size()
            << "\n";

  // 3) CRC sobre bytes (header + payload)
  std::cout << "[PacketFrame] Before CRC append\n";
  std::cout << "  data_bytes.size() = " << data_bytes.size() << "\n";

  CRC16::append(data_bytes);

  std::cout << "[PacketFrame] After CRC append\n";
  std::cout << "  data_bytes.size() = " << data_bytes.size() << "\n";
  std::cout << "  last 8 values of data_bytes: ";
  for (int i = std::max(0, (int)data_bytes.size() - 8);
       i < (int)data_bytes.size(); ++i) {
    std::cout << (int)data_bytes[i] << " ";
  }
  std::cout << "\n";

  // 4) Bytes -> bits -> símbolos FSK4
  std::vector<uint8_t> data_bits = bytes_to_bits_lsb(data_bytes);
  std::vector<uint8_t> data_syms = bits_to_symbols_fsk4(data_bits);

  frame.insert(frame.end(), data_syms.begin(), data_syms.end());

  std::cout << "[PacketFrame] Final frame built\n";
  std::cout << "  frame.size() = " << frame.size() << "\n";

  // DEBUG histograma del frame final (ya debería ser solo 0..3)
  {
    size_t c0 = 0, c1 = 0, c2 = 0, c3 = 0, c_other = 0;
    for (uint8_t v : frame) {
      if (v == 0)
        c0++;
      else if (v == 1)
        c1++;
      else if (v == 2)
        c2++;
      else if (v == 3)
        c3++;
      else
        c_other++;
    }

    std::cout << "[PacketFrame DEBUG] value histogram: "
              << "0=" << c0 << " 1=" << c1 << " 2=" << c2 << " 3=" << c3
              << " other=" << c_other << " total=" << frame.size() << "\n";

    std::cout << "[PacketFrame DEBUG] first 60 values: ";
    for (int i = 0; i < 60 && i < (int)frame.size(); ++i)
      std::cout << (int)frame[i] << " ";
    std::cout << "\n";
  }

  return frame;
}

bool PacketFrame::extract_payload(const std::vector<uint8_t> &frame,
                                  Header &header,
                                  std::vector<uint8_t> &payload_bits) {
  // Helpers locales
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

  auto bytes_to_bits_lsb = [](const std::vector<uint8_t> &bytes) {
    std::vector<uint8_t> bits;
    bits.reserve(bytes.size() * 8);
    for (uint8_t v : bytes) {
      for (int i = 0; i < 8; ++i)
        bits.push_back((v >> i) & 1);
    }
    return bits;
  };

  // =========================
  // 1) Comprobar tamaño mínimo
  // =========================
  const size_t min_syms = PREAMBLE_LENGTH + (HEADER_SIZE + CRC_SIZE) * 4;

  if (frame.size() < min_syms)
    return false;

  // =========================
  // 2) Validar que frame empieza EXACTAMENTE en preámbulo
  // =========================
  if (!is_preamble(frame, 0))
    return false;

  // =========================
  // 3) Extraer todos los símbolos después del preámbulo
  // =========================
  std::vector<uint8_t> data_syms(frame.begin() + PREAMBLE_LENGTH, frame.end());

  // =========================
  // 4) Convertir símbolos → bits → bytes
  // =========================
  std::vector<uint8_t> data_bits = symbols_to_bits_fsk4(data_syms);
  std::vector<uint8_t> data_bytes_with_crc = bits_to_bytes_lsb(data_bits);

  if (data_bytes_with_crc.size() < HEADER_SIZE + CRC_SIZE)
    return false;

  // =========================
  // 5) Validar CRC
  // =========================
  if (!CRC16::verify(data_bytes_with_crc))
    return false;

  // =========================
  // 6) Parsear header
  // =========================
  header = Header::from_bytes(data_bytes_with_crc.data());
  if (!header.is_valid())
    return false;

  // =========================
  // 7) Validar tamaño total exacto
  // =========================
  size_t expected_bytes = HEADER_SIZE + header.payload_length + CRC_SIZE;

  if (data_bytes_with_crc.size() != expected_bytes)
    return false;

  // =========================
  // 8) Extraer payload BYTES
  // =========================
  std::vector<uint8_t> payload_bytes(data_bytes_with_crc.begin() + HEADER_SIZE,
                                     data_bytes_with_crc.end() - CRC_SIZE);

  // =========================
  // 9) Convertir BYTES → BITS (LSB-first)
  // =========================
  payload_bits = bytes_to_bits_lsb(payload_bytes);

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
