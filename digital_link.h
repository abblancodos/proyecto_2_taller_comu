#pragma once

#include <vector>
#include <cstdint>
#include "conv_codec.h"

class dsp_client;

/**
 * Controlador de transmisión digital:
 * - Procesa audio WAV o datos de prueba
 * - Codifica con FEC (ConvolutionalEncoder)
 * - Mapea a símbolos 4-FSK (0..3)
 */
class DigitalTxController {
public:
    explicit DigitalTxController(dsp_client& client);

    // Procesar audio WAV para transmisión
    void prepare_wav_payload(const float* audio_samples, size_t num_samples);
    
    // Mantener método de prueba existente
    void prepare_test_payload();

    // Llamar periódicamente para avanzar al siguiente símbolo
    void tick();

    // Verificar si terminó la transmisión
    bool done() const;
    
    // Resetear el controlador
    void reset();

private:
    dsp_client& _client;
    conv::ConvolutionalEncoder _enc;

    std::vector<std::uint8_t> _tx_bits;      // bits originales
    std::vector<std::uint8_t> _tx_bits_enc;  // bits codificados
    std::vector<std::uint8_t> _symbols;      // símbolos 0..3
    std::size_t _symbol_index = 0;
};

/**
 * Controlador de recepción digital:
 * - Recibe símbolos FSK detectados
 * - Decodifica con Viterbi
 * - Reconstruye audio
 */
class DigitalRxController {
public:
    DigitalRxController();
    
    // Añadir un símbolo detectado
    void add_symbol(std::uint8_t symbol);
    
    // Procesar bloques acumulados
    void process_block();
    
    // Verificar si hay audio recuperado
    bool has_audio_ready() const;
    
    // Obtener audio recuperado
    void get_audio(std::vector<float>& audio);
    
    // Resetear el controlador
    void reset();
    
    // Obtener estadísticas
    std::size_t get_symbols_received() const { return _total_symbols; }
    std::size_t get_bits_decoded() const { return _total_bits_decoded; }

private:
    conv::ViterbiDecoder _dec;
    std::vector<std::uint8_t> _rx_bits;
    std::vector<std::uint8_t> _rx_symbols;
    std::vector<std::uint8_t> _decoded_bits;
    std::vector<float> _recovered_audio;
    
    std::size_t _total_symbols;
    std::size_t _total_bits_decoded;
    
    static constexpr std::size_t SYMBOLS_PER_BLOCK = 512;
};
