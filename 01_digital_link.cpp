#include "digital_link.h"
#include "dsp_client.h"
#include <iostream>
#include <random>

DigitalTxController::DigitalTxController(dsp_client& client)
    : _client(client)
{
}

void DigitalTxController::prepare_test_payload() {
    _tx_bits.clear();
    _tx_bits_enc.clear();
    _symbols.clear();
    _symbol_index = 0;

    // --- 1) Generar un patron de prueba en bis ---
    // const char* msg = "XYZ7P9";

    // New: Generating 32 bytes -- random
    std::mt19937 rng(12345);
    std::uniform_int_distribution<uint8_t> dist(0, 255);

    const int N_BYTES = 32;
    for (int i = 0; i < N_BYTES; ++i) {
	uint8_t b = dist(rng);
	for (int k = 7; k >= 0; --k) {
	    _tx_bits.push_back((b >> k) & 1);
	}
    }

    // Old loop causing a bug
    /*
    for (const char* p = msg; *p != '\0'; ++p) {
        std::uint8_t b = static_cast<std::uint8_t>(*p);
        for (int i = 7; i >= 0; --i) {
            std::uint8_t bit = (b >> i) & 0x01;
            _tx_bits.push_back(bit);
        }
    }
    */

    // --- 2) Codificación convolucional (FEC) ---
    _enc.encode_block(_tx_bits, _tx_bits_enc, /*add_tail_bits=*/true);

    // --- 3) Bits codificados -> símbolos 4-FSK (2 bits por símbolo) ---
    for (std::size_t i = 0; i + 1 < _tx_bits_enc.size(); i += 2) {
        std::uint8_t b0 = _tx_bits_enc[i];
        std::uint8_t b1 = _tx_bits_enc[i + 1];
        std::uint8_t sym = (b0 << 1) | b1; // 00->0, 01->1, 10->2, 11->3
        sym &= 0x3; // asegurarse que está en [0,3]
        _symbols.push_back(sym);
    }

    std::cout << "[DigitalTxController] Payload preparado: "
              << _tx_bits.size() << " bits -> "
              << _tx_bits_enc.size() << " bits codificados -> "
              << _symbols.size() << " símbolos FSK"
              << std::endl;

    _symbol_index = 0;
}

void DigitalTxController::tick() {
    if (_symbol_index < _symbols.size()) {
        int s = static_cast<int>(_symbols[_symbol_index]);
        _client.set_fsk_symbol(s);
        ++_symbol_index;
    } else {
        // Si ya terminamos, dejar fijo un símbolo "idle" (por ejemplo 0)
        _client.set_fsk_symbol(0);
    }
}

bool DigitalTxController::done() const {
    return _symbol_index >= _symbols.size();
}

