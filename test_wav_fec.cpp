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
    if (bits.size() < 8) {
        bytes.clear();
        return;
    }
    std::size_t nbytes = bits.size() / 8;  // piso
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

    std::vector<int16_t> samples(static_cast<std::size_t>(frames) * channels);
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

    std::cout << "bits_in.size() = " << bits_in.size() << "\n";

    // --- Parámetros de FEC / canal ---
    const std::size_t block_bits = 50000;   // tamaño de bloque para Viterbi
    const std::size_t total_bits = bits_in.size();

    double p = 0.005;                         // prob de error de bit en el canal
    std::mt19937 rng(1234);
    std::bernoulli_distribution flip(p);

    ConvolutionalEncoder enc;
    ViterbiDecoder dec;
    BerCounter ber;

    std::vector<uint8_t> bits_dec_all;
    bits_dec_all.reserve(total_bits);

    // --- Procesar por bloques ---
    for (std::size_t off = 0; off < total_bits; off += block_bits) {
        std::size_t n = std::min(block_bits, total_bits - off);

        // 1) Bloque de entrada u
        std::vector<uint8_t> u(bits_in.begin() + off,
                               bits_in.begin() + off + n);

        // 2) Codificar bloque
        enc.reset();
        std::vector<uint8_t> c;
        enc.encode_block(u, c, /*add_tail_bits=*/true);

        // 3) Canal BSC sobre bits codificados
        std::vector<uint8_t> r = c;
        for (auto &b : r) {
            if (flip(rng)) b ^= 1u;
        }

        // 4) Decodificar bloque
        std::vector<uint8_t> uhat;
        dec.decode_hard(r, uhat, /*terminated=*/true);

        // Puede devolver más o menos bits; usamos la parte válida común
        std::size_t len = std::min(u.size(), uhat.size());
        if (len == 0) {
            std::cerr << "Bloque con len=0 en off=" << off << ", n=" << n << "\n";
            continue;
        }

        uhat.resize(len);

        // 5) BER por bloque (acumulado)
        ber.update_block(u, uhat, len);

        // 6) Acumular bits decodificados
        bits_dec_all.insert(bits_dec_all.end(), uhat.begin(), uhat.end());
    }

    std::cout << "BER total = " << ber.ber()
              << " (" << ber.errors() << " / " << ber.total() << ")\n";

    // --- Reconstrucción: bits -> bytes ---
    // Redondear a múltiplo de 8 para reconstrucción limpia
    std::size_t n_info_bits_trunc = (bits_dec_all.size() / 8) * 8;
    if (n_info_bits_trunc == 0) {
        std::cerr << "Muy pocos bits decodificados para reconstruir.\n";
        return 1;
    }

    std::vector<uint8_t> bits_dec_trunc(bits_dec_all.begin(),
                                        bits_dec_all.begin() + n_info_bits_trunc);

    std::vector<uint8_t> bytes_out;
    bits_to_bytes(bits_dec_trunc, bytes_out);

    std::cout << "bytes_in.size() = " << bytes_in.size()
              << ", bytes_out.size() = " << bytes_out.size() << "\n";

    // --- Reconstruir muestras de salida ---
    std::vector<int16_t> samples_out(samples.size());
    std::size_t max_copy_bytes = samples.size() * sizeof(int16_t);
    std::size_t copy_bytes = std::min(bytes_out.size(), max_copy_bytes);

    std::memcpy(samples_out.data(), bytes_out.data(), copy_bytes);

    if (copy_bytes < max_copy_bytes) {
        std::cerr << "Advertencia: solo se copiaron "
                  << copy_bytes << " bytes de " << max_copy_bytes << "\n";
    }

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

