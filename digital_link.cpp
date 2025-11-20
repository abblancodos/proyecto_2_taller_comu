#include "digital_link.h"
#include "dsp_client.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <random>

// ============================================================================
// DigitalTxController
// ============================================================================

DigitalTxController::DigitalTxController(dsp_client& client)
    : _client(client)
{
}

void DigitalTxController::reset() {
    _tx_bits.clear();
    _tx_bits_enc.clear();
    _symbols.clear();
    _symbol_index = 0;
}

void DigitalTxController::prepare_wav_payload(const float* audio_samples, size_t num_samples) {
    reset();
    
    std::cout << "[DigitalTxController] Procesando " << num_samples << " muestras de audio WAV" << std::endl;
    
    // Convertir audio float [-1,1] a bits usando cuantización de 8 bits
    for (size_t i = 0; i < num_samples; ++i) {
        // Limitar el rango a [-1, 1]
        float sample = std::max(-1.0f, std::min(1.0f, audio_samples[i]));
        
        // Convertir a 8-bit unsigned [0, 255]
        uint8_t byte = static_cast<uint8_t>((sample + 1.0f) * 127.5f);
        
        // Convertir byte a 8 bits
        for (int b = 7; b >= 0; --b) {
            _tx_bits.push_back((byte >> b) & 1);
        }
    }
    
    std::cout << "[DigitalTxController] Convertidos " << num_samples 
              << " samples a " << _tx_bits.size() << " bits" << std::endl;
    
    // Codificar con FEC convolucional
    _enc.reset();
    _enc.encode_block(_tx_bits, _tx_bits_enc, true);
    
    std::cout << "[DigitalTxController] Codificación FEC: " << _tx_bits.size() 
              << " bits -> " << _tx_bits_enc.size() << " bits codificados" << std::endl;
    
    // Mapear bits codificados a símbolos 4-FSK (2 bits por símbolo)
    for (size_t i = 0; i + 1 < _tx_bits_enc.size(); i += 2) {
        uint8_t b0 = _tx_bits_enc[i];
        uint8_t b1 = _tx_bits_enc[i + 1];
        uint8_t sym = (b0 << 1) | b1;  // 00->0, 01->1, 10->2, 11->3
        _symbols.push_back(sym & 0x3);
    }
    
    std::cout << "[DigitalTxController] Generados " << _symbols.size() 
              << " símbolos FSK-4 listos para transmitir" << std::endl;
    
    _symbol_index = 0;
}

void DigitalTxController::prepare_test_payload() {
    reset();
    
    // Generar 32 bytes aleatorios para prueba
    std::mt19937 rng(12345);
    std::uniform_int_distribution<uint8_t> dist(0, 255);
    
    const int N_BYTES = 32;
    for (int i = 0; i < N_BYTES; ++i) {
        uint8_t b = dist(rng);
        for (int k = 7; k >= 0; --k) {
            _tx_bits.push_back((b >> k) & 1);
        }
    }
    
    // Codificar con FEC
    _enc.encode_block(_tx_bits, _tx_bits_enc, true);
    
    // Mapear a símbolos FSK
    for (size_t i = 0; i + 1 < _tx_bits_enc.size(); i += 2) {
        uint8_t b0 = _tx_bits_enc[i];
        uint8_t b1 = _tx_bits_enc[i + 1];
        uint8_t sym = (b0 << 1) | b1;
        _symbols.push_back(sym & 0x3);
    }
    
    std::cout << "[DigitalTxController] Test payload: "
              << _tx_bits.size() << " bits -> "
              << _tx_bits_enc.size() << " bits codificados -> "
              << _symbols.size() << " símbolos FSK"
              << std::endl;
    
    _symbol_index = 0;
}

/*
 * Bug founded in this structure
 *
void DigitalTxController::tick() {
    if (_symbol_index < _symbols.size()) {
        int s = static_cast<int>(_symbols[_symbol_index]);
        _client.set_fsk_symbol(s);
        ++_symbol_index;
        
        // Log cada 100 símbolos
        if (_symbol_index % 100 == 0) {
            std::cout << "[TX] Transmitiendo símbolo " << _symbol_index 
                      << "/" << _symbols.size() << std::endl;
        }
    } else {
        // Transmisión completada, enviar símbolo idle
        _client.set_fsk_symbol(0);
    }
}
*/

bool DigitalTxController::done() const {
    return _symbol_index >= _symbols.size();
}

void DigitalTxController::tick() {
    if (_symbol_index < _symbols.size()) {
        int s = static_cast<int>(_symbols[_symbol_index]);
        _client.set_fsk_symbol(s);
        ++_symbol_index;

        if (_symbol_index % 100 == 0) {
            std::cout << "[TX] Transmitiendo símbolo " << _symbol_index
                      << "/" << _symbols.size() << std::endl;
        }
    } else if (_symbol_index == _symbols.size()) {
        // Solo imprimir una vez cuando se completa
        std::cout << "[TX] Transmisión completada - " << _symbols.size() 
                  << " símbolos transmitidos" << std::endl;
        _symbol_index++; // Para que no se imprima repetidamente
    }
    // No hacer nada si ya completó
}

// ============================================================================
// DigitalRxController
// ============================================================================

DigitalRxController::DigitalRxController() 
    : _total_symbols(0)
    , _total_bits_decoded(0)
{
    reset();
}

void DigitalRxController::reset() {
    _rx_bits.clear();
    _rx_symbols.clear();
    _decoded_bits.clear();
    _recovered_audio.clear();
    _total_symbols = 0;
    _total_bits_decoded = 0;
}

void DigitalRxController::add_symbol(uint8_t symbol) {
    _rx_symbols.push_back(symbol & 0x3);
    _total_symbols++;
    
    // Procesar cuando tengamos suficientes símbolos
    if (_rx_symbols.size() >= SYMBOLS_PER_BLOCK) {
        process_block();
    }
}

void DigitalRxController::process_block() {
    if (_rx_symbols.empty()) return;
    
    std::cout << "[DigitalRxController] Procesando bloque de " 
              << _rx_symbols.size() << " símbolos" << std::endl;
    
    // Convertir símbolos a bits (2 bits por símbolo)
    _rx_bits.clear();
    for (uint8_t sym : _rx_symbols) {
        _rx_bits.push_back((sym >> 1) & 1);  // bit más significativo
        _rx_bits.push_back(sym & 1);          // bit menos significativo
    }
    
    std::cout << "[DigitalRxController] Convertidos a " << _rx_bits.size() << " bits" << std::endl;
    
    // Decodificar con Viterbi
    _decoded_bits.clear();
    _dec.decode_hard(_rx_bits, _decoded_bits, true);
    
    std::cout << "[DigitalRxController] Decodificados " << _decoded_bits.size() 
              << " bits con Viterbi" << std::endl;
    
    _total_bits_decoded += _decoded_bits.size();
    
    // Convertir bits decodificados a muestras de audio
    for (size_t i = 0; i + 7 < _decoded_bits.size(); i += 8) {
        uint8_t byte = 0;
        for (int b = 0; b < 8; ++b) {
            byte |= (_decoded_bits[i + b] << (7 - b));
        }
        
        // Convertir byte [0, 255] a float [-1, 1]
        float sample = (byte / 127.5f) - 1.0f;
        _recovered_audio.push_back(sample);
    }
    
    std::cout << "[DigitalRxController] Recuperadas " << _recovered_audio.size() 
              << " muestras de audio" << std::endl;
    
    // Limpiar buffer de símbolos para el próximo bloque
    _rx_symbols.clear();
}

bool DigitalRxController::has_audio_ready() const {
    return !_recovered_audio.empty();
}

void DigitalRxController::get_audio(std::vector<float>& audio) {
    audio = std::move(_recovered_audio);
    _recovered_audio.clear();
}
