/**
 * freq_filter_goertzel.cpp
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

#include "freq_filter_goertzel.h"
#include <cstring>
#include <thread>
#include <chrono>
#include <numbers>
#include <algorithm>

#undef _DSP_DEBUG
#define _DSP_DEBUG

#ifdef _DSP_DEBUG
#include <iostream>
#define _debug(x) std::cerr << x
#else
#define _debug(x)
#endif

// ============================================================================
// goertzel_filter implementation
// ============================================================================

goertzel_filter::goertzel_filter(float target_freq, float sample_rate, std::size_t block_size)
  : _v_n_minus_1(0.0f)
  , _v_n_minus_2(0.0f)
  , _magnitude_squared(0.0f)
  , _block_size(block_size)
{
  compute_coefficients(target_freq, sample_rate, block_size);
}

void goertzel_filter::compute_coefficients(float target_freq, float sample_rate, std::size_t block_size) {
  _block_size = block_size;
  
  // Compute the normalized frequency bin k
  // k = round(f_target * N / f_s)
  float k = std::round(target_freq * block_size / sample_rate);
  
  // Compute coefficient: 2*cos(2πk/N)
  float omega = 2.0f * std::numbers::pi_v<float> * k / static_cast<float>(block_size);
  _coeff = 2.0f * std::cos(omega);
  
  // Compute W_N^k = e^(-j*2πk/N) for final computation
  _wr = std::cos(omega);   // Real part
  _wi = -std::sin(omega);  // Imaginary part (note the negative sign)
  
  _debug("Goertzel filter initialized: f=" << target_freq 
         << " Hz, k=" << k 
         << ", coeff=" << _coeff 
         << ", wr=" << _wr 
         << ", wi=" << _wi << std::endl);
}

void goertzel_filter::reset() {
  _v_n_minus_1 = 0.0f;
  _v_n_minus_2 = 0.0f;
  _magnitude_squared = 0.0f;
}

float goertzel_filter::process_block(const float* in, std::size_t nframes) {
  // Reset state variables for new block
  _v_n_minus_1 = 0.0f;
  _v_n_minus_2 = 0.0f;
  
  // Equation (3.8): Iterate the difference equation for n = 0, 1, ..., N-1
  // v_k(n) = 2*cos(2πk/N)*v_k(n-1) - v_k(n-2) + x(n)
  for (std::size_t n = 0; n < nframes; ++n) {
    float v_n = _coeff * _v_n_minus_1 - _v_n_minus_2 + in[n];
    _v_n_minus_2 = _v_n_minus_1;
    _v_n_minus_1 = v_n;
  }
  
  // Equation (3.9): Compute y_k(N) = v_k(N) - W_N^k * v_k(N-1)
  // This gives us the complex DFT value at frequency k
  // X(k) = y_k(N)
  float real_part = _v_n_minus_1 - _wr * _v_n_minus_2;
  float imag_part = -_wi * _v_n_minus_2;
  
  // Compute magnitude squared: |X(k)|^2 = real^2 + imag^2
  _magnitude_squared = real_part * real_part + imag_part * imag_part;
  
  return _magnitude_squared;
}

void goertzel_filter::set_parameters(float target_freq, float sample_rate, std::size_t block_size) {
  compute_coefficients(target_freq, sample_rate, block_size);
  reset();
}

// ============================================================================
// fsk4_detector implementation
// ============================================================================

fsk4_detector::fsk4_detector(float f1, float f2, float f3, float f4,
                             float sample_rate, std::size_t block_size,
                             std::size_t buffer_depth)
  : _filters{{
      goertzel_filter(f1, sample_rate, block_size),
      goertzel_filter(f2, sample_rate, block_size),
      goertzel_filter(f3, sample_rate, block_size),
      goertzel_filter(f4, sample_rate, block_size)
    }}
  , _current_magnitudes{0.0f, 0.0f, 0.0f, 0.0f}
  , _current_magnitudes_squared{0.0f, 0.0f, 0.0f, 0.0f}
  , _buffer_depth(buffer_depth)
  , _sample_rate(sample_rate)
  , _block_size(block_size)
{
  _debug("4-FSK detector initialized: f1=" << f1 
         << " Hz, f2=" << f2 
         << " Hz, f3=" << f3 
         << " Hz, f4=" << f4 
         << " Hz" << std::endl);
}

void fsk4_detector::process_block(const float* in, std::size_t nframes) {
  // Process each frequency with its Goertzel filter
  for (std::size_t i = 0; i < 4; ++i) {
    _current_magnitudes_squared[i] = _filters[i].process_block(in, nframes);
    _current_magnitudes[i] = _filters[i].get_magnitude();
    
    // Add to buffer (maintain fixed depth)
    _magnitude_buffers[i].push_back(_current_magnitudes[i]);
    if (_magnitude_buffers[i].size() > _buffer_depth) {
      _magnitude_buffers[i].pop_front();
    }
  }
  
  _debug("FSK4 magnitudes: [" 
         << _current_magnitudes[0] << ", "
         << _current_magnitudes[1] << ", "
         << _current_magnitudes[2] << ", "
         << _current_magnitudes[3] << "]" << std::endl);
}

int fsk4_detector::get_strongest_frequency_index() const {

  // Debug purpouses -- added 2025.11.20
  std::cout << "FSK4 Magnitudes: [" 
            << _current_magnitudes[0] << ", " 
            << _current_magnitudes[1] << ", " 
            << _current_magnitudes[2] << ", " 
            << _current_magnitudes[3] << "]" << std::endl;
  
  // Find the index with maximum magnitude
  auto max_it = std::max_element(_current_magnitudes.begin(), _current_magnitudes.end());
  int detected_index = std::distance(_current_magnitudes.begin(), max_it);
 
  // Debug purpouses -- added 2025.11.20 
  std::cout << "S�mbolo detectado: " << detected_index << std::endl;
  
  return detected_index;
}
void fsk4_detector::reset() {
  for (auto& filter : _filters) {
    filter.reset();
  }
  _current_magnitudes.fill(0.0f);
  _current_magnitudes_squared.fill(0.0f);
  for (auto& buffer : _magnitude_buffers) {
    buffer.clear();
  }
}

void fsk4_detector::set_frequencies(float f1, float f2, float f3, float f4) {
  _filters[0].set_parameters(f1, _sample_rate, _block_size);
  _filters[1].set_parameters(f2, _sample_rate, _block_size);
  _filters[2].set_parameters(f3, _sample_rate, _block_size);
  _filters[3].set_parameters(f4, _sample_rate, _block_size);
  reset();
}

// ============================================================================
// freq_filter implementation (enhanced with Goertzel)
// ============================================================================

// fftw provides alloc_real for doubles and alloc_complex, but nothing
// for float
static float* fftwf_alloc_float(std::size_t n) {
  return reinterpret_cast<float*>(fftwf_malloc(sizeof(float)*n));
}

inline void freq_filter::mul(fftwf_complex& X,
                             const fftwf_complex& H) const {
  float re = X[0]*H[0] - X[1]*H[1];
  float im = X[0]*H[1] + X[1]*H[0];

  X[0] = re;
  X[1] = im;
}

freq_filter::freq_filter()
  : _block_size(0)
  , _Hw_size(0)
  , _hn_size(0)
  , _processing(false)
  , _changing_filter(true)
  , _sample_rate(0.0f)
{
}

freq_filter::freq_filter(std::size_t block_size)
  : _block_size(block_size)
  , _Hw_size(0)
  , _hn_size(0)
  , _processing(false)
  , _changing_filter(true)
  , _sample_rate(0.0f)
{
}

freq_filter::~freq_filter() {
  _block_size = 0;
  _Hw_size = 0;
  _hn_size = 0;

  if (_Hw_size > 0) {
    fftwf_destroy_plan(_fft);
    fftwf_destroy_plan(_ifft);
  }

  _Hw.reset();
  _Xw.reset();
  _xn.reset();
  _yn.reset();
}

void freq_filter::set_block_size(const std::size_t block_size) {
  _changing_filter = true;
  while(_processing) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  _block_size = block_size;
}

void freq_filter::make_fftw_plans(std::size_t Hw_size, std::size_t hn_size) {
  if ((Hw_size != _Hw_size) || (hn_size != _hn_size)) {
    _debug("  set-up plans and memory arrays" << std::endl);

    if (_Hw_size > 0) {
      fftwf_destroy_plan(_fft);
      fftwf_destroy_plan(_ifft);
    }

    _Hw_size = Hw_size; // This is L+M-1 (in Proakis' notation)
    _hn_size = hn_size; // This is M

    // Both _Hw and _Xw must share the same size
    _Hw.reset(fftwf_alloc_complex(_Hw_size));
    _Xw.reset(fftwf_alloc_complex(_Hw_size));
    memset(_Xw.get(), 0, sizeof(fftwf_complex)*_Hw_size);

    // Even if the size of h(n) is _hn_size, we use Hw_size because zero
    // padding must be performed
    _xn.reset(fftwf_alloc_float(_Hw_size));
    _yn.reset(fftwf_alloc_float(_Hw_size));
    memset(_xn.get(), 0, sizeof(float)*_Hw_size);
    memset(_yn.get(), 0, sizeof(float)*_Hw_size);

    _fft  = fftwf_plan_dft_r2c_1d(_Hw_size, _xn.get(), _Xw.get(), FFTW_MEASURE);
    _ifft = fftwf_plan_dft_c2r_1d(_Hw_size, _Xw.get(), _yn.get(), FFTW_MEASURE);
  }
}

void freq_filter::set_filter(const fftwf_complex* Hw,
                             std::size_t Hw_size,
                             std::size_t hn_size) {
  _changing_filter = true;
  while(_processing) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  
  make_fftw_plans(Hw_size, hn_size);

  // The FFTW does not automatically normalize the inverse transform.
  // We force the normalization inserting the normalization factor into the
  // filter itself

#if 1 // set to zero to avoid dividing by _Hw_size

  const fftwf_complex* src = Hw;
  const fftwf_complex *const src_end = src + _Hw_size;
  fftwf_complex* dest = _Hw.get();

  while (src != src_end) {
    (*dest)[0] = (*src)[0] / _Hw_size;
    (*dest)[1] = (*src)[1] / _Hw_size;

    ++src;
    ++dest;
  }

#else

  // debug line avoiding normalization
  memcpy(_Hw.get(), Hw, sizeof(fftwf_complex)*_Hw_size);

#endif
  _changing_filter = false;
}

void freq_filter::set_filter(const float* hn,
                             std::size_t hn_size,
                             std::size_t Hw_size) {
  
  _debug(" freq_filter::set_filter()" << std::endl);

  _changing_filter = true;

  while(_processing) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // Build the plans.  hn_size is used there just to detect if plan
  // adjustments are necessary.  The plans will operate with Hw_size
  make_fftw_plans(Hw_size, hn_size);
  
  _debug("  computing frequency response of given impulse response\n");

  // We use now the FFT to compute H(w) from h[n]
  memset(_xn.get(), 0, sizeof(float)*_Hw_size); // zero padding

  // first move the impulse response to x(n)
  std::copy(hn, hn + _hn_size, _xn.get());

  // Compute the frequency response
  fftwf_execute(_fft);

  // The FFTW does not automatically normalize the inverse transform.
  // We force the normalization inserting the normalization factor into the
  // filter itself

#if 1 // set to zero to avoid dividing by _Hw_size

  const fftwf_complex* src = _Xw.get();
  const fftwf_complex *const src_end = src + _Hw_size;
  fftwf_complex* dest = _Hw.get();

  while (src != src_end) {
    (*dest)[0] = (*src)[0] / _Hw_size;
    (*dest)[1] = (*src)[1] / _Hw_size;

    ++src;
    ++dest;
  }

#else

  // debug line avoiding normalization
  memcpy(_Hw.get(), _Xw.get(), sizeof(fftwf_complex)*_Hw_size);

#endif

  // Since the size of the filter can differ from the size of x[n], we
  // reset here the buffer, so the padding fits the processing steps.
  memset(_xn.get(), 0, sizeof(float)*_Hw_size); // zero padding
  
  _changing_filter = false;
}

void freq_filter::process(const float* in, float* out) {
  // use the fastest memory_order for the atomics...
  if (_changing_filter.load(std::memory_order_relaxed) || (_block_size == 0)) {
    // Abort!  Configuration of the filters is being changed
    return;
  }

  // Flag everywhere we are using the filters.
  _processing.store(true, std::memory_order_relaxed);

  // we use overlap-save method

  // the save-part first:
  const std::size_t hn_size1 = (_hn_size - 1);
  
  // copy the last _hn_size-1 samples from the end of the last response to
  // the very beginning
  std::copy(_xn.get() + _block_size, _xn.get() + _block_size + hn_size1, _xn.get());  
  // now copy the input block after the saved block
  std::copy(in, in + _block_size, _xn.get() + hn_size1);

  // when the filter was set, the rest was set to zero.

  fftwf_execute(_fft); // input to the frequency domain

  fftwf_complex* Xw_ptr = _Xw.get();
  fftwf_complex *const Xw_end = Xw_ptr + _Hw_size;
  const fftwf_complex* Hw_ptr = _Hw.get();
  
  // multiply _Xw and _Hw
  for (; Xw_ptr != Xw_end; ++Xw_ptr, ++Hw_ptr) {
    mul(*Xw_ptr, *Hw_ptr);
  }

  // return to the time domain
  fftwf_execute(_ifft);

  // and the last step: move the data to the output array
  std::copy(_yn.get() + hn_size1, _yn.get() + hn_size1 + _block_size, out);

  // we use "active" waiting outside this thread, to avoid time overhead here
  _processing.store(false, std::memory_order_relaxed);
}

void freq_filter::reset() {
  if (_Hw_size > 0) {
    memset(_xn.get(), 0, sizeof(float)*_Hw_size);
    memset(_yn.get(), 0, sizeof(float)*_Hw_size);
  }
  
  if (_fsk4_detector) {
    _fsk4_detector->reset();
  }
}

void freq_filter::init_fsk4_detector(float f1, float f2, float f3, float f4,
                                     float sample_rate, std::size_t buffer_depth) {
  _sample_rate = sample_rate;
  _fsk4_detector = std::make_unique<fsk4_detector>(f1, f2, f3, f4, 
                                                    sample_rate, _block_size, 
                                                    buffer_depth);
}

void freq_filter::process_fsk4(const float* in) {
  if (_fsk4_detector) {
    _fsk4_detector->process_block(in, _block_size);
  }
}

void freq_filter::fftw_complex_deleter::operator()(::fftwf_complex* ptr) {
  if (ptr == nullptr) {
    return;
  }
  ::fftwf_free(ptr);
}

void freq_filter::fftw_float_deleter::operator()(float* ptr) {
  ::fftwf_free(ptr);
}
