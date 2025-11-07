/**
 * freq_filter_goertzel.h
 *
 * Enhanced frequency filter with Goertzel algorithm for 4-FSK detection
 * Based on the original freq_filter by Pablo Alvarado
 * Extended to support efficient frequency detection using the Goertzel algorithm
 *
 * Copyright (C) 2023-2024  Pablo Alvarado
 * EL5805 Procesamiento Digital de Señales
 * Escuela de Ingeniería Electrónica
 * Tecnológico de Costa Rica
 */

#ifndef _FREQ_FILTER_GOERTZEL_H
#define _FREQ_FILTER_GOERTZEL_H

#include <fftw3.h>
#include <cstddef>
#include <memory>
#include <atomic>
#include <vector>
#include <array>
#include <deque>
#include <cmath>

/**
 * Goertzel filter for detecting a single frequency
 * 
 * This implements the second-order Goertzel algorithm from equations (3.8) and (3.9):
 * v_k(n) = 2*cos(2πk/N)*v_k(n-1) - v_k(n-2) + x(n)
 * y_k(n) = v_k(n) - W_N^k * v_k(n-1)
 */
class goertzel_filter {
public:
  /**
   * Constructor
   * @param target_freq Target frequency in Hz
   * @param sample_rate Sampling rate in Hz
   * @param block_size Number of samples per block (N in Proakis)
   */
  goertzel_filter(float target_freq, float sample_rate, std::size_t block_size);
  
  /**
   * Reset the filter state
   */
  void reset();
  
  /**
   * Process a block of samples and return the magnitude squared at the target frequency
   * @param in Input samples buffer
   * @param nframes Number of samples in the buffer
   * @return Magnitude squared of the DFT at target frequency
   */
  float process_block(const float* in, std::size_t nframes);
  
  /**
   * Get the magnitude (sqrt of magnitude squared)
   */
  float get_magnitude() const { return std::sqrt(_magnitude_squared); }
  
  /**
   * Get the magnitude squared (more efficient, avoids sqrt)
   */
  float get_magnitude_squared() const { return _magnitude_squared; }
  
  /**
   * Update parameters (useful for dynamic frequency changes)
   */
  void set_parameters(float target_freq, float sample_rate, std::size_t block_size);

private:
  float _coeff;              // 2*cos(2πk/N)
  float _wr;                 // Real part of W_N^k
  float _wi;                 // Imaginary part of W_N^k
  float _v_n_minus_1;        // v_k(n-1)
  float _v_n_minus_2;        // v_k(n-2)
  float _magnitude_squared;  // |X(k)|^2
  std::size_t _block_size;   // N samples
  
  void compute_coefficients(float target_freq, float sample_rate, std::size_t block_size);
};

/**
 * 4-FSK detector using Goertzel algorithm
 * 
 * Efficiently detects 4 frequencies simultaneously and maintains a buffer
 * of recent magnitude measurements for symbol detection
 */
class fsk4_detector {
public:
  /**
   * Constructor
   * @param f1 First frequency (Hz)
   * @param f2 Second frequency (Hz)
   * @param f3 Third frequency (Hz)
   * @param f4 Fourth frequency (Hz)
   * @param sample_rate Sampling rate (Hz)
   * @param block_size Samples per block
   * @param buffer_depth Number of past blocks to store
   */
  fsk4_detector(float f1, float f2, float f3, float f4,
                float sample_rate, std::size_t block_size,
                std::size_t buffer_depth = 10);
  
  /**
   * Process a block and update magnitude buffers
   * @param in Input samples
   * @param nframes Number of samples
   */
  void process_block(const float* in, std::size_t nframes);
  
  /**
   * Get the current magnitudes (most recent block)
   * @return Array of 4 magnitude values [f1, f2, f3, f4]
   */
  const std::array<float, 4>& get_current_magnitudes() const { 
    return _current_magnitudes; 
  }
  
  /**
   * Get magnitude squared values (more efficient)
   * @return Array of 4 magnitude squared values
   */
  const std::array<float, 4>& get_current_magnitudes_squared() const {
    return _current_magnitudes_squared;
  }
  
  /**
   * Get the index of the strongest frequency (0-3)
   */
  int get_strongest_frequency_index() const;
  
  /**
   * Get the magnitude buffer for a specific frequency
   * @param freq_index Index of frequency (0-3)
   * @return Deque of past magnitude measurements
   */
  const std::deque<float>& get_magnitude_buffer(std::size_t freq_index) const {
    return _magnitude_buffers[freq_index];
  }
  
  /**
   * Reset all filters
   */
  void reset();
  
  /**
   * Update frequencies dynamically
   */
  void set_frequencies(float f1, float f2, float f3, float f4);
  
private:
  std::array<goertzel_filter, 4> _filters;
  std::array<float, 4> _current_magnitudes;
  std::array<float, 4> _current_magnitudes_squared;
  std::array<std::deque<float>, 4> _magnitude_buffers;
  std::size_t _buffer_depth;
  float _sample_rate;
  std::size_t _block_size;
};

/**
 * Enhanced filtering operation in the frequency domain with Goertzel support
 *
 * This extends the original freq_filter class with Goertzel algorithm
 * capabilities for efficient single-frequency or multi-frequency detection.
 */
class freq_filter {
public:
  /**
   * Create a temporary freq_filter
   */
  freq_filter();
  
  /**
   * Constructor
   *
   * @param block_size size of the data blocks to be filtered.  This
   *                   should be the size of the buffers that Jack's process()
   *                   handles.
   */
  freq_filter(std::size_t block_size);

  /**
   * Destructor
   */
  ~freq_filter();

  /**
   * Change the block size.  
   * 
   * The block size is the number of "samples" processed in the frequency domain.
   * It corresponds to the 'L', in Proakis & Manolakis' DSP book.
   *
   * This MUST be called before set_filter.
   */
  void set_block_size(const std::size_t block_size);
  
  /**
   * Set the complex frequency response of the filter Hw.
   *
   * The filter size must be at least the block_size plus the size of the filter
   * impuse response, i.e. the size of the given filter frequency response
   * shall already consider the zero padded impulse response, in such a way
   * that the product of the response and the transformed input signal do not
   * alias the temporal signal.
   *
   * @param Hw pointer to the complex frequency response.  The pointed data
   *           will be copied in a memory block managed by this instance.  The
   *           owner of that data is responsible of deallocating it.
   * @param Hw_size total number of complex numbers in Hw.  It must be greater
   *               or equal than the sum of  block_size (provided at
   *                construction time) plus the length of the impulse response
   *                minus 1.
   * @param hn_size length of the impulse response used.
   */
  void set_filter(const fftwf_complex* Hw,
                  std::size_t Hw_size,
                  std::size_t hn_size);

  /**
   * Set the filter impulse response
   *
   * The filter size must be at least the processing block_size (L)
   * plus the size of the filter impuse response, i.e. the size of the
   * given filter frequency response should already have considered
   * the zero padded impulse response to be able to hold the result of
   * the convolution without aliasing.
   *
   * @param hn samples of the impulse response.  It must be already zero 
   *        padded.  The memory is assumed to be managed elsewhere, hence, 
            it will just be used here.
   * @param hn_size length of the hn array
   * @param Hw_size size of the complete frequency response, that must allow
   *        to hold at least the convolution result of hn_size + block_size-1.
   */
  void set_filter(const float* hn,std::size_t hn_size,std::size_t Hw_size);

  /**
   * Filter the input block of the block_size given at construction
   * time and produce the output of the same size considering past
   * evaluations.
   *
   * This filtering transforms the input in to the frequency domain,
   * multiplies that spectrum with the frequency response of the filter
   * and transforms back.  
   *
   * The provided implementation uses the overlap-save method.
   */
  void process(const float* in,float* out);

  /**
   * Reset
   *
   * Set all internal state data to zero
   */
  void reset();
  
  /**
   * Initialize 4-FSK detector
   * @param f1 First frequency (Hz)
   * @param f2 Second frequency (Hz)
   * @param f3 Third frequency (Hz)
   * @param f4 Fourth frequency (Hz)
   * @param sample_rate Sampling rate (Hz)
   * @param buffer_depth Number of past magnitude measurements to keep
   */
  void init_fsk4_detector(float f1, float f2, float f3, float f4,
                          float sample_rate, std::size_t buffer_depth = 10);
  
  /**
   * Get the FSK4 detector (nullptr if not initialized)
   */
  fsk4_detector* get_fsk4_detector() { return _fsk4_detector.get(); }
  
  /**
   * Process block through FSK4 detector (in addition to normal filtering)
   * @param in Input samples
   */
  void process_fsk4(const float* in);

private:
  /**
   * Block size (i.e. Jack's process() input and output buffer sizes)
   */
  std::size_t _block_size;

  /**
   * Frequency response size
   */
  std::size_t _Hw_size;

  /**
   * Impulse respones size
   */
  std::size_t _hn_size;

  /**
   * fftw3 library plan for direct transform
   */
  fftwf_plan _fft;

  /**
   * fftw3 library plan for inverse transform
   */
  fftwf_plan _ifft;

  /**
   * Construct the fftw plans according to the indicated lenghts.
   */
  void make_fftw_plans(std::size_t HwSize,std::size_t hn_size);

  /**
   * Helper deleters for the unique pointers
   */
  //@{
  struct fftw_complex_deleter {
    void operator()(::fftwf_complex* ptr);
  };
  
  struct fftw_float_deleter {
    void operator()(float* ptr);
  };
  //@}

  typedef std::unique_ptr<fftwf_complex,fftw_complex_deleter> complex_ptr;
  typedef std::unique_ptr<float,fftw_float_deleter> float_ptr;
  
  /**
   * Buffer used for frequency domain filter response
   */
  complex_ptr _Hw;

  /**
   * Buffer used for the frequency domain input
   */
  complex_ptr _Xw;

  /**
   * Buffer used for the input in the discrete time domain
   */
  float_ptr _xn;

  /**
   * Buffer used for the output in the discrete time domain
   */
  float_ptr _yn;

  /**
   * Multiply two complex numbers and return the result in place:
   *
   * X = X*H
   */
  inline void mul(fftwf_complex& X,
                  const fftwf_complex& H) const;

  /**
   * Safeguards for process and set_filter (lock-free atomic)
   */
  static_assert(std::atomic<bool>::is_always_lock_free);
  
  std::atomic<bool> _processing;
  std::atomic<bool> _changing_filter;
  
  /**
   * Optional 4-FSK detector using Goertzel algorithm
   */
  std::unique_ptr<fsk4_detector> _fsk4_detector;
  float _sample_rate;
};

#endif // _FREQ_FILTER_GOERTZEL_H