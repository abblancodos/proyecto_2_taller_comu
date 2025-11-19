#include "conv_codec.h"

#include <algorithm> // std::fill
#include <limits> // std::numeric_limits

namespace conv {

	static inline uint8_t parity7(uint8_t x) {
		// 7-bits parity
		return static_cast<uint8_t>(__builtin_parity(static_cast<unsigned>(x)));
}

ConvolutionalEncoder::TrellisTables::TrellisTables() {
	for (int s = 0; s < NSTATES; ++s) {
		for (int u = 0; u <=1; ++u) {
			uint8_t reg7 = static_cast<uint8_t>((s << 1) | (u & 1));
			uint8_t y0 = parity7(reg7 & G0);
			uint8_t y1 = parity7(reg7 & G1);

			// (y0, y1) como 2 bits empaquetados
      			out_sym[s][u] = static_cast<uint8_t>((y0 << 1) | y1);

      			// Nuevo estado = desplazar a la izquierda e insertar u
      			uint8_t ns = static_cast<uint8_t>(((s << 1) | u) & ((1 << M) - 1));
      			next_state[s][u] = ns;
		}
	}

}

const ViterbiDecoder::TrellisTables&
ViterbiDecoder::trellis() {
  static const TrellisTables T;
  return T;
}

void ViterbiDecoder::decode_hard(const std::vector<bit_t>& rx_bits,
                                 std::vector<bit_t>& out_bits,
                                 bool terminated) const
{
  const auto& T = trellis();

  if (rx_bits.size() % 2 != 0) {
    out_bits.clear();
    return;
  }

  const std::size_t Nsteps = rx_bits.size() / 2;

  const int INF = std::numeric_limits<int>::max() / 4;

  std::vector<int> pm(NSTATES, INF), pm_new(NSTATES, INF);
  std::vector<uint8_t> prev_state(NSTATES * Nsteps);
  std::vector<uint8_t> prev_bit(NSTATES * Nsteps);

  // Estado inicial = 0
  pm[0] = 0;

  // Forward pass
  for (std::size_t t = 0; t < Nsteps; ++t) {
    uint8_t r = static_cast<uint8_t>((rx_bits[2*t] << 1) |
                                      rx_bits[2*t + 1]);
    std::fill(pm_new.begin(), pm_new.end(), INF);

    for (int s = 0; s < NSTATES; ++s) {
      if (pm[s] >= INF) continue;

      for (int u = 0; u <= 1; ++u) {
        uint8_t ns = T.next_state[s][u];
        uint8_t sy = T.out_sym[s][u];
        int bm = hamming2(sy, r);
        int cand = pm[s] + bm;

        if (cand < pm_new[ns]) {
          pm_new[ns] = cand;
          prev_state[t*NSTATES + ns] = static_cast<uint8_t>(s);
          prev_bit[t*NSTATES + ns]   = static_cast<uint8_t>(u);
        }
      }
    }
    pm.swap(pm_new);
  }

  // Estado final
  int best_state = 0;

  if (!terminated) {
    int best = INF;
    for (int s = 0; s < NSTATES; ++s) {
      if (pm[s] < best) {
        best = pm[s];
        best_state = s;
      }
    }
  }

  // Traceback
  out_bits.assign(Nsteps, 0);
  int s = best_state;

  for (int t = static_cast<int>(Nsteps) - 1; t >= 0; --t) {
    uint8_t u = prev_bit[t*NSTATES + s];
    out_bits[t] = static_cast<bit_t>(u);
    s = prev_state[t*NSTATES + s];
  }
}

// ==========================================================
// BerCounter
// ==========================================================

void BerCounter::update_block(const std::vector<bit_t>& ref,
                              const std::vector<bit_t>& est,
                              std::size_t n)
{
  std::size_t N = std::min({n, ref.size(), est.size()});
  for (std::size_t i = 0; i < N; ++i) {
    update(ref[i], est[i]);
  }
}

} // namespace conv

