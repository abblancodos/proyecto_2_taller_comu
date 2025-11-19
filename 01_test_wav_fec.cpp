#include "conv_codec.h"

#include <sndfile.h>
#include <cstdint>
#include <vector>
#include <iostream>
#include <random>
#include <algorithm>
#include <cstring>  // memcpy

using conv::ConvolutionalEncoder;
using conv::ViterbiDecoder;
using conv::BerCounter;
using std::uint8_t;

// Convierte bytes -> bits (MSB primero)
void bytes_to_bits(const std::vector<uint8_t>& bytes,
                   std::vector<uint8_t>& bits) {
    bits.clear();
    bits.reserve(bytes.size() * 8);
    for (uint8_t b : bytes) {
        for (int i = 0; i < 8; ++i) {
            uint8_t bit = (b >> (7 - i)) & 1u;
            bits.push_back(bit);
        }
    }
}

// Convierte bits -> bytes (usa grupos de 8 bits, ignora el resto)
void bits_to_bytes(const std::vector<uint8_t>& bits,
                   std::vector<uint8_t>& bytes) {
    std::size_t nbytes = bits.size() / 8;
    bytes.assign(nbytes, 0);
    for (std::size_t i = 0; i < nbytes; ++i) {
        uint8_t b = 0;
        for (int j = 0; j < 8; ++j) {
            b |= (bits[8 * i + j] & 1u) << (7 - j);
        }
        bytes[i] = b;
    }
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Uso: " << argv[0] << " entrada.wav salida.wav\n";
        return 1;
    }

    const char* in_path  = argv[1];
    const char* out_path = argv[2];

    // --- Leer WAV de entrada ---
    SF_INFO info{};
    SNDFILE* in_file = sf_open(in_path, SFM_READ, &info);
    if (!in_file) {
        std::cerr << "Error abriendo WAV de entrada: " << in_path << "\n";
        return 1;
    }

    sf_count_t frames = info.frames;
    int channels = info.channels;

    std::vector<int16_t> samples(frames * channels);
    sf_count_t read = sf_readf_short(in_file, samples.data(), frames);
    sf_close(in_file);

    if (read != frames) {
        std::cerr << "Advertencia: se esperaban " << frames
                  << " frames pero se leyeron " << read << "\n";
    }

    // --- Interpretar las muestras como bytes crudos ---
    std::size_t n_bytes = samples.size() * sizeof(int16_t);
    std::vector<uint8_t> bytes_in(n_bytes);
    std::memcpy(bytes_in.data(), samples.data(), n_bytes);

    // --- bytes -> bits ---
    std::vector<uint8_t> bits_in;
    bytes_to_bits(bytes_in, bits_in);

    // --- Codificación convolucional ---
    ConvolutionalEncoder enc;
    enc.reset();

    std::vector<uint8_t> bits_enc;
    enc.encode_block(bits_in, bits_enc, /*add_tail_bits=*/true);

    // --- Canal simulado (BSC) ---
    std::vector<uint8_t> bits_rx = bits_enc;

    double p = 0.0;  // por ahora SIN errores. Luego podés subirlo.
    std::mt19937 rng(1234);
    std::bernoulli_distribution flip(p);

    for (auto& b : bits_rx) {
        if (flip(rng)) {
            b ^= 1u;
        }
    }

    // --- Decodificación Viterbi ---
    ViterbiDecoder dec;
    std::vector<uint8_t> bits_dec;
    dec.decode_hard(bits_rx, bits_dec, /*terminated=*/true);

    // Viterbi devuelve N_info + M bits, a nosotros solo nos interesan los N originales
    if (bits_dec.size() > bits_in.size()) {
        bits_dec.resize(bits_in.size());
    }

    // --- Calcular BER entre bits_in y bits_dec ---
    BerCounter ber;
    ber.update_block(bits_in, bits_dec, bits_in.size());

    std::cout << "BER = " << ber.ber()
              << " (" << ber.errors() << " / " << ber.total() << ")\n";

    // --- bits -> bytes ---
    std::vector<uint8_t> bytes_out;
    bits_to_bytes(bits_dec, bytes_out);

    if (bytes_out.size() != bytes_in.size()) {
        std::cerr << "Advertencia: bytes_out.size() = " << bytes_out.size()
                  << " != bytes_in.size() = " << bytes_in.size() << "\n";
    }

    // --- Reconstruir muestras de salida ---
    std::vector<int16_t> samples_out(samples.size());
    std::size_t copy_bytes =
        std::min(bytes_out.size(), samples.size() * sizeof(int16_t));

    std::memcpy(samples_out.data(), bytes_out.data(), copy_bytes);

    // --- Escribir WAV de salida ---
    SF_INFO out_info = info;
    SNDFILE* out_file = sf_open(out_path, SFM_WRITE, &out_info);
    if (!out_file) {
        std::cerr << "Error abriendo WAV de salida: " << out_path << "\n";
        return 1;
    }

    sf_count_t written = sf_writef_short(out_file, samples_out.data(), frames);
    sf_close(out_file);

    if (written != frames) {
        std::cerr << "Advertencia: se esperaban escribir " << frames
                  << " frames pero se escribieron " << written << "\n";
    }

    std::cout << "Listo. Archivo de salida: " << out_path << "\n";
    return 0;
}

