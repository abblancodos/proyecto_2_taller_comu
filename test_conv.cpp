#include "conv_codec.h"
#include <iostream>
#include <random>

int main() {
  using namespace conv;

  ConvolutionalEncoder enc;
  ViterbiDecoder       dec;
  BerCounter           ber;

  // 1) Generar bits aleatorios
  std::size_t N = 100000;
  std::vector<uint8_t> u(N);
  std::mt19937 rng(1234);
  std::bernoulli_distribution bitgen(0.5);

  for (auto& b : u) {
    b = static_cast<uint8_t>(bitgen(rng));
  }

  // 2) Codificar (con tail bits)
  std::vector<uint8_t> c;
  enc.reset();
  enc.encode_block(u, c, /*add_tail_bits=*/true);

  // 3) Simular canal BSC simple (voltear bits con prob. p)
  double p = 0.001; // probabilidad mas realista
  // double p = 0.15;
  std::bernoulli_distribution flip(p);
  std::vector<uint8_t> r = c;
  for (auto& b : r) {
    if (flip(rng)) {
      b ^= 1u;
    }
  }

  // 4) Decodificar
  std::vector<uint8_t> uhat;
  dec.decode_hard(r, uhat, /*terminated=*/true);

  // uhat incluye los M tail bits al final: comparamos solo los primeros N
  ber.update_block(u, uhat, N);

  std::cout << "BER = " << ber.ber()
            << "  (" << ber.errors() << " / " << ber.total() << ")\n";

  return 0;
}

