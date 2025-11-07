/**
 * fir_filter.h
 *
 * Generic FIR filter with circular buffer for real-time processing
 * Supports optimization for Hilbert filters (skips near-zero even coefficients)
 */
#ifndef FIR_FILTER_H
#define FIR_FILTER_H

#include <cstddef>
#include <cstring>

/**
 * Filter type enumeration
 */
enum class FIRFilterType {
    STANDARD,   // Standard FIR (processes all coefficients)
    HILBERT     // Hilbert FIR (skips even-indexed coefficients which are ~0)
};

/**
 * Generic FIR Filter with optional Hilbert optimization
 * Uses circular buffer for efficient real-time processing
 */
class FIRFilter {
public:
    /**
     * Constructor
     * @param coeffs Pointer to filter coefficients
     * @param length Filter length (number of taps)
     * @param type Filter type (STANDARD or HILBERT for optimization)
     */
    FIRFilter(const float* coeffs, size_t length, FIRFilterType type = FIRFilterType::STANDARD) 
        : _coeffs(coeffs)
        , _length(length)
        , _type(type)
        , _delay_idx(0)
    {
        _delay_line = new float[length];
        std::memset(_delay_line, 0, length * sizeof(float));
    }

    ~FIRFilter() {
        delete[] _delay_line;
    }

    /**
     * Process one sample through the filter
     * Automatically uses optimized path for Hilbert filters
     */
    inline float process(float input) {
        if (_type == FIRFilterType::HILBERT) {
            return process_hilbert(input);
        } else {
            return process_standard(input);
        }
    }

    /**
     * Reset filter state (clear delay line)
     */
    void reset() {
        std::memset(_delay_line, 0, _length * sizeof(float));
        _delay_idx = 0;
    }

    /**
     * Get filter length
     */
    size_t length() const {
        return _length;
    }

    /**
     * Get group delay (for linear phase FIR)
     */
    size_t group_delay() const {
        return (_length - 1) / 2;
    }

    /**
     * Get filter type
     */
    FIRFilterType type() const {
        return _type;
    }

private:
    /**
     * Standard FIR processing - uses all coefficients
     */
    inline float process_standard(float input) {
        // Insert new sample into circular buffer
        _delay_line[_delay_idx] = input;
        
        // Compute convolution: y[n] = sum(h[k] * x[n-k])
        float output = 0.0f;
        size_t idx = _delay_idx;
        
        // Process all coefficients
        for (size_t i = 0; i < _length; ++i) {
            output += _coeffs[i] * _delay_line[idx];
            
            // Move back one position in circular buffer
            idx = (idx == 0) ? (_length - 1) : (idx - 1);
        }
        
        // Update circular buffer index
        _delay_idx = (_delay_idx + 1) % _length;
        
        return output;
    }

    /**
     * Optimized Hilbert FIR processing - skips even coefficients
     * Even-indexed coefficients in Hilbert filters are approximately 0
     * This reduces computational load by ~50%
     */
    inline float process_hilbert(float input) {
        // Insert new sample into circular buffer
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

    const float* _coeffs;    // Filter coefficients
    size_t _length;          // Number of taps
    FIRFilterType _type;     // Filter type (for optimization selection)
    float* _delay_line;      // Circular delay line buffer
    size_t _delay_idx;       // Current position in circular buffer
};

#endif // FIR_FILTER_H