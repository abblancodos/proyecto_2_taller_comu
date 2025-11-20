/**
 * digital_link.h
 *
 * Capa de enlace digital para transmisión de audio WAV
 * usando codificación convolucional (FEC) + modulación 4-FSK.
 *
 * Flujo TX:
 *   WAV -> muestras PCM int16 -> bits -> FEC -> bits codificados -> símbolos 4-FSK
 *
 * Flujo RX:
 *   símbolos 4-FSK -> bits -> Viterbi -> bits info -> muestras PCM int16 -> WAV
 */

#ifndef DIGITAL_LINK_H
#define DIGITAL_LINK_H

#include <vector>
#include <cstdint>
#include <string>
#include <sndfile.h>       // libsndfile
#include "conv_codec.h"    // ConvolutionalEncoder / ViterbiDecoder

class DigitalLink {
public:
    DigitalLink();

    // ========================
    //        TX  (WAV -> FSK)
    // ========================

    /// Carga un archivo WAV completo (solo mono o toma canal 0 si es estéreo).
    /// Devuelve true si todo bien.
    bool load_wav(const std::string& path);

    /// Prepara el payload para TX:
    ///   PCM -> bits -> FEC -> símbolos 4-FSK
    /// Devuelve false si no hay WAV cargado.
    bool prepare_tx_payload();

    /// ¿Hay payload listo para transmitir?
    bool tx_ready() const { return _tx_ready; }

    /// ¿Ya se han transmitido todos los símbolos del payload?
    bool tx_done() const;

    /// Devuelve el siguiente símbolo 4-FSK (0..3).
    /// Si ya terminó, por simplicidad sigue devolviendo 0.
    int next_tx_symbol();

    /// Resetea el estado TX (para volver a preparar otro WAV).
    void reset_tx();

    // ========================
    //        RX  (FSK -> WAV)
    // ========================

    /// Prepara buffers RX (se puede llamar antes de iniciar la recepción).
    void reset_rx();

    /// Agrega un símbolo detectado en el receptor (0..3).
    void push_rx_symbol(int symbol);

    /// Intenta decodificar lo recibido y escribir un WAV de salida.
    /// Devuelve true si logró decodificar y escribir el archivo.
    ///
    /// Caso ideal (para laboratorio): se usa la misma instancia de DigitalLink
    /// tanto en TX como en RX, de forma que conoce:
    ///   - cuántos bits de información originales había (_tx_bits.size())
    ///   - cuántos bits codificados se esperaban (_tx_bits_enc.size())
    bool decode_to_wav(const std::string& out_path);

    // ========================
    //     Parámetros varios
    // ========================

    /// Profundidad de bits por muestra PCM (por defecto 16).
    void set_bits_per_sample(int bits) { _bits_per_sample = bits; }

    int sample_rate() const { return _sample_rate; }

private:
    // -------- helpers internos --------

    void pcm_to_bits();
    void bits_to_pcm(const std::vector<uint8_t>& info_bits);

    void bits_to_symbols(const std::vector<uint8_t>& bits_enc);
    void symbols_to_bits(std::vector<uint8_t>& bits_out) const;

    bool write_wav(const std::string& path);

    // ========================
    //         Estado WAV
    // ========================
    int _sample_rate;
    int _channels;
    int _bits_per_sample;             // normalmente 16

    std::vector<int16_t> _pcm_samples; // audio original
    std::vector<int16_t> _pcm_rx;      // audio reconstruido

    // ========================
    //        Estado TX
    // ========================
    std::vector<uint8_t> _tx_bits;      // bits de info
    std::vector<uint8_t> _tx_bits_enc;  // bits codificados
    std::vector<uint8_t> _tx_symbols;   // símbolos 0..3

    size_t _tx_symbol_index;
    bool   _tx_ready;

    // ========================
    //        Estado RX
    // ========================
    std::vector<uint8_t> _rx_symbols;   // símbolos recibidos
    std::vector<uint8_t> _rx_bits;      // bits codificados reconstruidos
    std::vector<uint8_t> _rx_bits_dec;  // bits info decodificados

    // ========================
    //         FEC
    // ========================
    conv::ConvolutionalEncoder _enc;
    conv::ViterbiDecoder        _dec;
};

#endif // DIGITAL_LINK_H

