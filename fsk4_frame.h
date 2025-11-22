/**
 * fsk4_frame.h
 *
 * FSK-4 Frame structure and utilities for digital communications
 * 
 * Frame Format:
 * [PREAMBLE] [HEADER] [PAYLOAD] [CRC16]
 * 
 * - PREAMBLE: Alternating pattern for synchronization (configurable length)
 * - HEADER: 12 bits (configuration/state/id)
 * - PAYLOAD: Variable length data
 * - CRC16: 16-bit CRC for error detection
 *
 * Copyright (C) 2024
 */

#ifndef FSK4_FRAME_H
#define FSK4_FRAME_H

#include <cstdint>
#include <cstddef>
#include <vector>
#include <array>

namespace fsk4 {

// ============================================================================
// FSK-4 Symbol Mapping
// ============================================================================

/**
 * Symbol to dibits mapping with maximum Hamming distance
 * Using Gray code for adjacent symbols to minimize bit errors
 */
enum class Symbol : uint8_t {
    S00 = 0,  // Frequency f1 -> dibits 00
    S01 = 1,  // Frequency f2 -> dibits 01
    S11 = 2,  // Frequency f3 -> dibits 11
    S10 = 3   // Frequency f4 -> dibits 10
};

// Convert dibits to symbol
inline Symbol dibits_to_symbol(uint8_t dibits) {
    return static_cast<Symbol>(dibits & 0x03);
}

// Convert symbol to dibits
inline uint8_t symbol_to_dibits(Symbol sym) {
    return static_cast<uint8_t>(sym);
}

// ============================================================================
// CRC-16 CCITT
// ============================================================================

class CRC16 {
public:
    CRC16() : _crc(0xFFFF) {}
    
    void reset() { _crc = 0xFFFF; }
    
    void update(uint8_t byte) {
        _crc ^= static_cast<uint16_t>(byte) << 8;
        for (int i = 0; i < 8; ++i) {
            if (_crc & 0x8000) {
                _crc = (_crc << 1) ^ 0x1021;
            } else {
                _crc <<= 1;
            }
        }
    }
    
    void update(const uint8_t* data, size_t length) {
        for (size_t i = 0; i < length; ++i) {
            update(data[i]);
        }
    }
    
    uint16_t get() const { return _crc; }
    
    static uint16_t calculate(const uint8_t* data, size_t length) {
        CRC16 crc;
        crc.update(data, length);
        return crc.get();
    }

private:
    uint16_t _crc;
};

// ============================================================================
// Frame Structure
// ============================================================================

struct FrameHeader {
    uint8_t frame_id : 4;      // Frame ID (0-15)
    uint8_t payload_length : 8; // Payload length in bytes (0-255)
    
    FrameHeader() : frame_id(0), payload_length(0) {}
    FrameHeader(uint8_t id, uint8_t len) : frame_id(id), payload_length(len) {}
    
    // Serialize to 12 bits (stored in 2 bytes)
    void serialize(uint8_t* out) const {
        out[0] = (frame_id << 4) | ((payload_length >> 4) & 0x0F);
        out[1] = (payload_length & 0x0F) << 4;
    }
    
    // Deserialize from 12 bits
    static FrameHeader deserialize(const uint8_t* in) {
        FrameHeader hdr;
        hdr.frame_id = (in[0] >> 4) & 0x0F;
        hdr.payload_length = ((in[0] & 0x0F) << 4) | ((in[1] >> 4) & 0x0F);
        return hdr;
    }
};

struct Frame {
    FrameHeader header;
    std::vector<uint8_t> payload;
    uint16_t crc;
    
    Frame() : crc(0) {}
    
    // Build complete frame with CRC
    void build() {
        CRC16 crc_calc;
        
        // Calculate CRC over header + payload
        uint8_t hdr_bytes[2];
        header.serialize(hdr_bytes);
        crc_calc.update(hdr_bytes, 2);
        crc_calc.update(payload.data(), payload.size());
        
        crc = crc_calc.get();
    }
    
    // Verify frame CRC
    bool verify() const {
        CRC16 crc_calc;
        
        uint8_t hdr_bytes[2];
        header.serialize(hdr_bytes);
        crc_calc.update(hdr_bytes, 2);
        crc_calc.update(payload.data(), payload.size());
        
        return crc_calc.get() == crc;
    }
    
    // Get total frame length in dibits (2 bits per symbol)
    size_t get_length_in_dibits() const {
        // Preamble handled separately
        // Header: 12 bits = 6 dibits
        // Payload: N bytes = N*8 bits = N*4 dibits
        // CRC: 16 bits = 8 dibits
        return 6 + (payload.size() * 4) + 8;
    }
};

// ============================================================================
// Preamble Pattern
// ============================================================================

/**
 * Generate preamble pattern for synchronization
 * Alternating 01 pattern provides good clock recovery
 */
inline std::vector<Symbol> generate_preamble(size_t length_in_symbols) {
    std::vector<Symbol> preamble;
    preamble.reserve(length_in_symbols);
    
    for (size_t i = 0; i < length_in_symbols; ++i) {
        // Alternating pattern: 01 01 01 01...
        preamble.push_back((i % 2) ? Symbol::S01 : Symbol::S00);
    }
    
    return preamble;
}

// ============================================================================
// Bit Packing Utilities
// ============================================================================

/**
 * Pack bytes into dibits (2-bit symbols)
 */
inline std::vector<uint8_t> bytes_to_dibits(const uint8_t* data, size_t length) {
    std::vector<uint8_t> dibits;
    dibits.reserve(length * 4); // Each byte = 4 dibits
    
    for (size_t i = 0; i < length; ++i) {
        uint8_t byte = data[i];
        // Extract 4 dibits from each byte (MSB first)
        dibits.push_back((byte >> 6) & 0x03);
        dibits.push_back((byte >> 4) & 0x03);
        dibits.push_back((byte >> 2) & 0x03);
        dibits.push_back((byte >> 0) & 0x03);
    }
    
    return dibits;
}

/**
 * Unpack dibits back to bytes
 */
inline std::vector<uint8_t> dibits_to_bytes(const std::vector<uint8_t>& dibits) {
    std::vector<uint8_t> bytes;
    bytes.reserve(dibits.size() / 4);
    
    for (size_t i = 0; i + 3 < dibits.size(); i += 4) {
        uint8_t byte = 0;
        byte |= (dibits[i + 0] & 0x03) << 6;
        byte |= (dibits[i + 1] & 0x03) << 4;
        byte |= (dibits[i + 2] & 0x03) << 2;
        byte |= (dibits[i + 3] & 0x03) << 0;
        bytes.push_back(byte);
    }
    
    return bytes;
}

/**
 * Convert frame to complete symbol sequence
 * [PREAMBLE] [HEADER] [PAYLOAD] [CRC]
 */
inline std::vector<Symbol> frame_to_symbols(const Frame& frame, size_t preamble_length = 16) {
    std::vector<Symbol> symbols;
    
    // 1. Add preamble
    auto preamble = generate_preamble(preamble_length);
    symbols.insert(symbols.end(), preamble.begin(), preamble.end());
    
    // 2. Add header (12 bits = 6 dibits)
    uint8_t hdr_bytes[2];
    frame.header.serialize(hdr_bytes);
    auto hdr_dibits = bytes_to_dibits(hdr_bytes, 2);
    // Only take first 6 dibits (12 bits)
    for (size_t i = 0; i < 6; ++i) {
        symbols.push_back(dibits_to_symbol(hdr_dibits[i]));
    }
    
    // 3. Add payload
    auto payload_dibits = bytes_to_dibits(frame.payload.data(), frame.payload.size());
    for (auto dibit : payload_dibits) {
        symbols.push_back(dibits_to_symbol(dibit));
    }
    
    // 4. Add CRC (16 bits = 8 dibits)
    uint8_t crc_bytes[2] = {
        static_cast<uint8_t>(frame.crc >> 8),
        static_cast<uint8_t>(frame.crc & 0xFF)
    };
    auto crc_dibits = bytes_to_dibits(crc_bytes, 2);
    for (auto dibit : crc_dibits) {
        symbols.push_back(dibits_to_symbol(dibit));
    }
    
    return symbols;
}

} // namespace fsk4

#endif // FSK4_FRAME_H