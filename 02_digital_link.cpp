/**
 * digital_link.cpp
 *
 * Implementación de la capa de enlace digital:
 *   WAV <-> bits <-> FEC <-> símbolos 4-FSK
 */

#include "digital_link.h"
#include <stdexcept>
#include <iostream>
#include <cmath>

DigitalLink::DigitalLink()
  : _sample_rate(0)
  , _channels(0)
  , _bits_per_sample(16)
  , _tx_symbol_index(0)
  , _tx_ready(false)
{
}

// ========================
//          TX
// ========================

bool DigitalLink::load_wav(const std::string& path) {
    SF_INFO info{};
    SNDFILE* snd = sf_open(path.c_str(), SFM_READ, &info);
    if (!snd) {
        std::cerr << "[DigitalLink] Error abriendo WAV: " << path << "\n";
        return false;
    }

    _sample_rate = info.samplerate;
    _channels    = info.channels;

    if (_channels <= 0) {
        std::cerr << "[DigitalLink] WAV con canales inválidos\n";
        sf_close(snd);
        return false;
    }

    // Leemos todo el archivo en float
    std::vector<float> tmp(info.frames * _channels);
    sf_count_t read_frames = sf_readf_float(snd, tmp.data(), info.frames);
    sf_close(snd);

    if (read_frames <= 0) {
        std::cerr << "[DigitalLink] No se pudieron leer frames\n";
        return false;
    }

    tmp.resize(read_frames * _channels);

    // Convertir a PCM int16 mono (tomamos canal 0)
    _pcm_samples.clear();
    _pcm_samples.reserve(read_frames);

    for (sf_count_t n = 0; n < read_frames; ++n) {
        float s = tmp[n * _channels + 0]; // canal 0
        // Clampear
        if (s > 1.0f)  s = 1.0f;
        if (s < -1.0f) s = -1.0f;
        int16_t v = static_cast<int16_t>(std::round(s * 32767.0f));
        _pcm_samples.push_back(v);
    }

    std::cout << "[DigitalLink] WAV cargado: " << path
              << " frames=" << read_frames
              << " sr=" << _sample_rate
              << " ch=" << _channels << "\n";

    return true;
}

void DigitalLink::pcm_to_bits() {
    _tx_bits.clear();
    _tx_bits.reserve(_pcm_samples.size() * _bits_per_sample);

    for (int16_t sample : _pcm_samples) {
        uint16_t v = static_cast<uint16_t>(sample);
        for (int b = 0; b < _bits_per_sample; ++b) {
            uint8_t bit = (v >> b) & 0x1;
            _tx_bits.push_back(bit);
        }
    }

    std::cout << "[DigitalLink] pcm_to_bits: "
              << _pcm_samples.size() << " muestras -> "
              << _tx_bits.size() << " bits\n";
}

void DigitalLink::bits_to_symbols(const std::vector<uint8_t>& bits_enc) {
    _tx_symbols.clear();
    _tx_symbols.reserve((bits_enc.size() + 1) / 2);

    for (size_t i = 0; i < bits_enc.size(); i += 2) {
        uint8_t b0 = bits_enc[i];
        uint8_t b1 = (i + 1 < bits_enc.size()) ? bits_enc[i + 1] : 0;
        uint8_t symbol = (b1 << 1) | (b0 & 0x1); // 00,01,10,11 -> 0..3
        _tx_symbols.push_back(symbol);
    }

    std::cout << "[DigitalLink] bits_to_symbols: "
              << bits_enc.size() << " bits -> "
              << _tx_symbols.size() << " símbolos\n";
}

bool DigitalLink::prepare_tx_payload() {
    if (_pcm_samples.empty()) {
        std::cerr << "[DigitalLink] No hay WAV cargado para preparar TX\n";
        return false;
    }

    // 1) PCM -> bits
    pcm_to_bits();

    // 2) FEC: codificar con ConvolutionalEncoder
    _tx_bits_enc.clear();
    _enc.reset();
    _enc.encode_block(_tx_bits, _tx_bits_enc, true); // con tail bits

    std::cout << "[DigitalLink] FEC: "
              << _tx_bits.size() << " bits -> "
              << _tx_bits_enc.size() << " bits codificados\n";

    // 3) bits codificados -> símbolos 4-FSK
    bits_to_symbols(_tx_bits_enc);

    _tx_symbol_index = 0;
    _tx_ready = true;

    std::cout << "[DigitalLink] Payload preparado: "
              << _tx_bits.size()     << " bits info -> "
              << _tx_bits_enc.size() << " bits codif -> "
              << _tx_symbols.size()  << " símbolos FSK\n";

    return true;
}

bool DigitalLink::tx_done() const {
    if (!_tx_ready) return true;
    return _tx_symbol_index >= _tx_symbols.size();
}

int DigitalLink::next_tx_symbol() {
    if (!_tx_ready || _tx_symbols.empty()) {
        return 0;
    }
    if (_tx_symbol_index >= _tx_symbols.size()) {
        // Payload terminado: por simplicidad devolvemos 0
        return 0;
    }
    int sym = _tx_symbols[_tx_symbol_index];
    ++_tx_symbol_index;
    return sym;
}

void DigitalLink::reset_tx() {
    _tx_bits.clear();
    _tx_bits_enc.clear();
    _tx_symbols.clear();
    _tx_symbol_index = 0;
    _tx_ready = false;
}

// ========================
//          RX
// ========================

void DigitalLink::reset_rx() {
    _rx_symbols.clear();
    _rx_bits.clear();
    _rx_bits_dec.clear();
    _pcm_rx.clear();
}

void DigitalLink::push_rx_symbol(int symbol) {
    if (symbol < 0 || symbol > 3) {
        return;
    }
    _rx_symbols.push_back(static_cast<uint8_t>(symbol));
}

void DigitalLink::symbols_to_bits(std::vector<uint8_t>& bits_out) const {
    bits_out.clear();
    bits_out.reserve(_rx_symbols.size() * 2);

    for (uint8_t sym : _rx_symbols) {
        uint8_t b0 = sym & 0x1;
        uint8_t b1 = (sym >> 1) & 0x1;
        bits_out.push_back(b0);
        bits_out.push_back(b1);
    }
}

void DigitalLink::bits_to_pcm(const std::vector<uint8_t>& info_bits) {
    _pcm_rx.clear();
    if (info_bits.empty()) return;

    // Usamos el mismo número de bits que en TX (si está disponible)
    size_t total_bits = info_bits.size();
    size_t bits_per_sample = static_cast<size_t>(_bits_per_sample);

    size_t num_samples = total_bits / bits_per_sample;
    _pcm_rx.reserve(num_samples);

    size_t idx = 0;
    for (size_t n = 0; n < num_samples; ++n) {
        uint16_t v = 0;
        for (size_t b = 0; b < bits_per_sample; ++b) {
            uint8_t bit = info_bits[idx++];
            v |= (static_cast<uint16_t>(bit & 0x1) << b);
        }
        int16_t sample = static_cast<int16_t>(v);
        _pcm_rx.push_back(sample);
    }

    std::cout << "[DigitalLink] bits_to_pcm: "
              << info_bits.size() << " bits -> "
              << _pcm_rx.size()   << " muestras\n";
}

bool DigitalLink::write_wav(const std::string& path) {
    if (_pcm_rx.empty() || _sample_rate <= 0) {
        std::cerr << "[DigitalLink] No hay audio RX o sample_rate inválido\n";
        return false;
    }

    SF_INFO info{};
    info.frames     = static_cast<sf_count_t>(_pcm_rx.size());
    info.samplerate = _sample_rate;
    info.channels   = 1; // siempre mono en la reconstrucción
    info.format     = SF_FORMAT_WAV | SF_FORMAT_PCM_16;

    SNDFILE* snd = sf_open(path.c_str(), SFM_WRITE, &info);
    if (!snd) {
        std::cerr << "[DigitalLink] Error abriendo WAV para escritura: "
                  << path << "\n";
        return false;
    }

    sf_count_t written = sf_writef_short(snd, _pcm_rx.data(), info.frames);
    sf_close(snd);

    if (written != info.frames) {
        std::cerr << "[DigitalLink] No se pudieron escribir todos los frames\n";
        return false;
    }

    std::cout << "[DigitalLink] WAV RX escrito: " << path
              << " frames=" << written
              << " sr=" << info.samplerate << "\n";
    return true;
}

bool DigitalLink::decode_to_wav(const std::string& out_path) {
    if (_rx_symbols.empty()) {
        std::cerr << "[DigitalLink] No hay símbolos RX para decodificar\n";
        return false;
    }

    // 1) símbolos -> bits codificados (2 bits/símbolo)
    symbols_to_bits(_rx_bits);

    std::cout << "[DigitalLink] RX: "
              << _rx_symbols.size() << " símbolos -> "
              << _rx_bits.size()    << " bits codificados\n";

    if (_rx_bits.empty()) {
        return false;
    }

    // 2) FEC: Viterbi
    _rx_bits_dec.clear();
    _dec.reset();

    // Caso ideal de laboratorio:
    // si se usó la misma instancia, sabemos cuántos bits codificados se esperaban
    size_t needed_coded = _tx_bits_enc.empty()
                            ? _rx_bits.size()
                            : std::min(_rx_bits.size(), _tx_bits_enc.size());

    std::vector<uint8_t> rx_cropped(_rx_bits.begin(),
                                    _rx_bits.begin() + needed_coded);

    _dec.decode_hard(rx_cropped, _rx_bits_dec, true);

    std::cout << "[DigitalLink] Viterbi: "
              << rx_cropped.size()  << " bits codificados -> "
              << _rx_bits_dec.size() << " bits info\n";

    if (_rx_bits_dec.empty()) {
        return false;
    }

    // 3) bits info -> PCM
    bits_to_pcm(_rx_bits_dec);

    // 4) escribir WAV
    return write_wav(out_path);
}

