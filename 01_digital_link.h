#pragma once

#include <vector>
#include <cstdint>

#include "conv_codec.h"  // Codificador/decodificador convolucional

class dsp_client;

/**
 * Controlador simple de transmisión digital:
 * - Genera un patrón de prueba en bits
 * - Lo codifica con FEC (ConvolutionalEncoder)
 * - Lo mapea a símbolos 4-FSK (0..3)
 * - En cada tick() llama a set_fsk_symbol() en dsp_client
 */
class DigitalTxController {
public:
    explicit DigitalTxController(dsp_client& client);

    // Prepara un payload de prueba (bits -> FEC -> símbolos)
    void prepare_test_payload();

    // Llamar periódicamente (por ejemplo, desde un QTimer)
    // para avanzar al siguiente símbolo FSK
    void tick();

    // Saber si ya se enviaron todos los símbolos
    bool done() const;

private:
    dsp_client& _client;
    conv::ConvolutionalEncoder _enc;

    std::vector<std::uint8_t> _tx_bits;      // bits originales
    std::vector<std::uint8_t> _tx_bits_enc;  // bits codificados
    std::vector<std::uint8_t> _symbols;      // símbolos 0..3

    std::size_t _symbol_index = 0;
};

