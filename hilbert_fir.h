/**
 * hilbert_fir.h
 *
 * Simple FIR Hilbert filter with circular buffer for real-time processing
 */

#ifndef HILBERT_FIR_H
#define HILBERT_FIR_H

#include <cstddef>
#include <cstring>

/**
 * Optimized FIR Hilbert Filter
 * Uses circular buffer and exploits the fact that even coefficients are ~0
 */
class HilbertFIR {
public:
    /**
     * Constructor
     * @param coeffs Pointer to filter coefficients
     * @param length Filter length (must be odd for Hilbert)
     */
    HilbertFIR(const float* coeffs, size_t length) 
        : _coeffs(coeffs)
        , _length(length)
        , _delay_idx(0)
    {
        _delay_line = new float[length];
        std::memset(_delay_line, 0, length * sizeof(float));
    }

    ~HilbertFIR() {
        delete[] _delay_line;
    }

    /**
     * Process one sample through the filter
     * Optimized to skip even-indexed coefficients (they're ~0 for Hilbert)
     */
    inline float process(float input) {
        // Insert new sample
        _delay_line[_delay_idx] = input;

        // Compute convolution (only odd indices)
        float output = 0.0f;
        size_t idx = _delay_idx;
        
        // Start at index 1 (first odd coefficient) and step by 2
        for (size_t i = 1; i < _length; i += 2) {
            // Move back one position
            idx = (idx == 0) ? (_length - 1) : (idx - 1);
            output += _coeffs[i] * _delay_line[idx];
            
            // Move back another position (skip even coefficient)
            idx = (idx == 0) ? (_length - 1) : (idx - 1);
        }

        // Update circular buffer index
        _delay_idx = (_delay_idx + 1) % _length;

        return output;
    }

    /**
     * Reset filter state
     */
    void reset() {
        std::memset(_delay_line, 0, _length * sizeof(float));
        _delay_idx = 0;
    }

private:
    const float* _coeffs;
    size_t _length;
    float* _delay_line;
    size_t _delay_idx;
};

#endif // HILBERT_FIR_H