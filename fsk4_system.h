/**
 * fsk4_system.h
 *
 * FSK-4 System with Circular Buffers Architecture
 * 
 * Architecture:
 * - Transmitter: Continuously fills TX circular buffer with modulated samples
 * - Receiver: Reads from RX circular buffer and decodes when synchronized
 * - dsp_client: Just moves data 1024 samples at a time between buffers and JACK
 *
 * Copyright (C) 2024
 */

#ifndef FSK4_SYSTEM_H
#define FSK4_SYSTEM_H

#include "fsk4_frame.h"
#include "freq_filter_goertzel.h"
#include <boost/circular_buffer.hpp>
#include <cmath>
#include <numbers>
#include <array>
#include <queue>
#include <mutex>
#include <atomic>
#include <thread>
#include <iostream>

namespace fsk4 {

// ============================================================================
// FSK-4 Transmitter with Circular Buffer
// ============================================================================

class FSK4TransmitterBuffer {
public:
    /**
     * Constructor
     * @param f1-f4 Frequencies for symbols 00, 01, 11, 10 (Hz)
     * @param baudrate Symbol rate (symbols per second)
     * @param sample_rate Audio sample rate (Hz)
     * @param buffer_size Size of circular buffer (in samples)
     * @param preamble_symbols Number of preamble symbols
     */
    FSK4TransmitterBuffer(float f1, float f2, float f3, float f4,
                          float baudrate, float sample_rate,
                          size_t buffer_size = 65536,
                          size_t preamble_symbols = 16)
        : _frequencies{f1, f2, f3, f4}
        , _baudrate(baudrate)
        , _sample_rate(sample_rate)
        , _preamble_symbols(preamble_symbols)
        , _samples_per_symbol(static_cast<size_t>(sample_rate / baudrate))
        , _tx_buffer(buffer_size)
        , _phase(0.0f)
        , _is_generating(false)
        , _stop_requested(false)
        , _frame_id(0)
    {
        std::cout << "[TX] FSK-4 Transmitter Buffer initialized:" << std::endl;
        std::cout << "     Frequencies: " << f1 << ", " << f2 << ", " 
                  << f3 << ", " << f4 << " Hz" << std::endl;
        std::cout << "     Baudrate: " << baudrate << " symbols/s" << std::endl;
        std::cout << "     Samples per symbol: " << _samples_per_symbol << std::endl;
        std::cout << "     Buffer size: " << buffer_size << " samples" << std::endl;
    }
    
    ~FSK4TransmitterBuffer() {
        stop_generator();
    }
    
    /**
     * Queue a frame for transmission
     * This will start the generator thread if not running
     */
    void queue_frame(const uint8_t* payload, size_t payload_length) {
        std::lock_guard<std::mutex> lock(_queue_mutex);
        
        // Create frame
        Frame frame;
        frame.header.frame_id = _frame_id++;
        frame.header.payload_length = static_cast<uint8_t>(payload_length);
        frame.payload.assign(payload, payload + payload_length);
        frame.build(); // Calculate CRC
        
        // Convert to symbols
        auto symbols = frame_to_symbols(frame, _preamble_symbols);
        
        std::cout << "[TX] Frame queued: ID=" << static_cast<int>(frame.header.frame_id)
                  << ", Payload=" << payload_length << " bytes"
                  << ", Symbols=" << symbols.size()
                  << ", CRC=0x" << std::hex << frame.crc << std::dec << std::endl;
        
        // Add to queue
        _symbol_queue.push(symbols);
        
        // Start generator if not running
        if (!_is_generating.load()) {
            start_generator();
        }
    }
    
    /**
     * Get samples from TX buffer for transmission
     * Called by dsp_client::process() to fill JACK output
     * @param out Output buffer
     * @param nframes Number of samples to read
     * @return Number of samples actually read
     */
    size_t get_samples(float* out, size_t nframes) {
        std::lock_guard<std::mutex> lock(_buffer_mutex);
        
        size_t available = _tx_buffer.size();
        size_t to_read = std::min(nframes, available);
        
        for (size_t i = 0; i < to_read; ++i) {
            out[i] = _tx_buffer.front();
            _tx_buffer.pop_front();
        }
        
        // Fill rest with silence if not enough samples
        if (to_read < nframes) {
            std::fill(out + to_read, out + nframes, 0.0f);
        }
        
        return to_read;
    }
    
    /**
     * Check how many samples are available in TX buffer
     */
    size_t available_samples() const {
        std::lock_guard<std::mutex> lock(_buffer_mutex);
        return _tx_buffer.size();
    }
    
    /**
     * Check if transmitter is active
     */
    bool is_transmitting() const {
        return _is_generating.load();
    }
    
    /**
     * Stop generator thread
     */
    void stop_generator() {
        _stop_requested.store(true);
        if (_generator_thread.joinable()) {
            _generator_thread.join();
        }
    }
    
    /**
     * Reset transmitter
     */
    void reset() {
        stop_generator();
        
        std::lock_guard<std::mutex> lock1(_buffer_mutex);
        std::lock_guard<std::mutex> lock2(_queue_mutex);
        
        _tx_buffer.clear();
        while (!_symbol_queue.empty()) _symbol_queue.pop();
        _phase = 0.0f;
        _frame_id = 0;
        _is_generating.store(false);
        _stop_requested.store(false);
        
        std::cout << "[TX] Transmitter reset" << std::endl;
    }
    
    /**
     * Set frequencies
     */
    void set_frequencies(float f1, float f2, float f3, float f4) {
        _frequencies = {f1, f2, f3, f4};
        std::cout << "[TX] Frequencies updated" << std::endl;
    }
    
    /**
     * Set baudrate
     */
    void set_baudrate(float baudrate) {
        _baudrate = baudrate;
        _samples_per_symbol = static_cast<size_t>(_sample_rate / baudrate);
        std::cout << "[TX] Baudrate updated: " << baudrate << " symbols/s" << std::endl;
    }

private:
    /**
     * Start generator thread that fills TX buffer
     */
    void start_generator() {
        _stop_requested.store(false);
        _is_generating.store(true);
        
        _generator_thread = std::thread(&FSK4TransmitterBuffer::generator_loop, this);
        
        std::cout << "[TX] Generator thread started" << std::endl;
    }
    
    /**
     * Generator loop - runs in separate thread
     * Continuously generates samples and fills TX buffer
     */
    void generator_loop() {
        while (!_stop_requested.load()) {
            // Check if we have symbols to transmit
            std::vector<Symbol> current_symbols;
            
            {
                std::lock_guard<std::mutex> lock(_queue_mutex);
                if (!_symbol_queue.empty()) {
                    current_symbols = _symbol_queue.front();
                    _symbol_queue.pop();
                } else {
                    // No more symbols, stop generator
                    _is_generating.store(false);
                    std::cout << "[TX] All frames transmitted, generator stopping" << std::endl;
                    break;
                }
            }
            
            // Generate samples for this frame
            std::cout << "[TX] Generating " << current_symbols.size() 
                      << " symbols..." << std::endl;
            
            for (size_t sym_idx = 0; sym_idx < current_symbols.size(); ++sym_idx) {
                Symbol symbol = current_symbols[sym_idx];
                float frequency = _frequencies[static_cast<size_t>(symbol)];
                float angular_freq = 2.0f * std::numbers::pi_v<float> * frequency / _sample_rate;
                
                // Generate samples for this symbol
                for (size_t i = 0; i < _samples_per_symbol; ++i) {
                    float sample = std::sin(_phase);
                    _phase += angular_freq;
                    
                    // Keep phase in range
                    if (_phase >= 2.0f * std::numbers::pi_v<float>) {
                        _phase -= 2.0f * std::numbers::pi_v<float>;
                    }
                    
                    // Add to TX buffer
                    {
                        std::lock_guard<std::mutex> lock(_buffer_mutex);
                        
                        // Wait if buffer is full (shouldn't happen with large buffer)
                        while (_tx_buffer.full() && !_stop_requested.load()) {
                            std::this_thread::sleep_for(std::chrono::microseconds(100));
                        }
                        
                        if (!_stop_requested.load()) {
                            _tx_buffer.push_back(sample);
                        }
                    }
                }
                
                // Progress update every 10 symbols
                if ((sym_idx + 1) % 10 == 0) {
                    std::cout << "[TX] Progress: " << (sym_idx + 1) 
                              << "/" << current_symbols.size() << " symbols" << std::endl;
                }
            }
            
            std::cout << "[TX] Frame generation complete" << std::endl;
        }
        
        _is_generating.store(false);
        std::cout << "[TX] Generator thread stopped" << std::endl;
    }

private:
    std::array<float, 4> _frequencies;
    float _baudrate;
    float _sample_rate;
    size_t _preamble_symbols;
    size_t _samples_per_symbol;
    
    // Circular buffer for TX samples
    boost::circular_buffer<float> _tx_buffer;
    mutable std::mutex _buffer_mutex;
    
    // Symbol queue
    std::queue<std::vector<Symbol>> _symbol_queue;
    std::mutex _queue_mutex;
    
    // Generator thread
    std::thread _generator_thread;
    std::atomic<bool> _is_generating;
    std::atomic<bool> _stop_requested;
    float _phase;
    uint8_t _frame_id;
};

// ============================================================================
// FSK-4 Receiver with Circular Buffer
// ============================================================================

class FSK4ReceiverBuffer {
public:
    /**
     * Constructor
     */
    FSK4ReceiverBuffer(float f1, float f2, float f3, float f4,
                       float baudrate, float sample_rate, size_t block_size,
                       size_t buffer_size = 65536)
        : _baudrate(baudrate)
        , _sample_rate(sample_rate)
        , _block_size(block_size)
        , _samples_per_symbol(static_cast<size_t>(sample_rate / baudrate))
        , _rx_buffer(buffer_size)
        , _state(RxState::IDLE)
        , _symbol_sample_counter(0)
        , _preamble_count(0)
        , _min_preamble_symbols(8)
        , _is_processing(false)
        , _stop_requested(false)
        , _last_symbol_time(std::chrono::steady_clock::now())
    {
        // Initialize Goertzel filters
        _detectors[0] = std::make_unique<goertzel_filter>(f1, sample_rate, _samples_per_symbol);
        _detectors[1] = std::make_unique<goertzel_filter>(f2, sample_rate, _samples_per_symbol);
        _detectors[2] = std::make_unique<goertzel_filter>(f3, sample_rate, _samples_per_symbol);
        _detectors[3] = std::make_unique<goertzel_filter>(f4, sample_rate, _samples_per_symbol);
        
        std::cout << "[RX] FSK-4 Receiver Buffer initialized:" << std::endl;
        std::cout << "     Frequencies: " << f1 << ", " << f2 << ", " 
                  << f3 << ", " << f4 << " Hz" << std::endl;
        std::cout << "     Baudrate: " << baudrate << " symbols/s" << std::endl;
        std::cout << "     Samples per symbol: " << _samples_per_symbol << std::endl;
        std::cout << "     Buffer size: " << buffer_size << " samples" << std::endl;
        
        // Start decoder thread
        start_decoder();
    }
    
    ~FSK4ReceiverBuffer() {
        stop_decoder();
    }
    
    /**
     * Put samples into RX buffer
     * Called by dsp_client::process() with incoming samples from JACK
     */
    void put_samples(const float* in, size_t nframes) {
        std::lock_guard<std::mutex> lock(_buffer_mutex);
        
        for (size_t i = 0; i < nframes; ++i) {
            _rx_buffer.push_back(in[i]);
        }
    }
    
    /**
     * Check if a complete frame has been received
     */
    bool has_frame() const {
        std::lock_guard<std::mutex> lock(_frame_mutex);
        return !_completed_frames.empty();
    }
    
    /**
     * Get next received frame
     */
    Frame get_frame() {
        std::lock_guard<std::mutex> lock(_frame_mutex);
        if (_completed_frames.empty()) {
            return Frame();
        }
        
        Frame frame = _completed_frames.front();
        _completed_frames.pop_front();
        return frame;
    }
    
    /**
     * Get current state
     */
    RxState get_state() const {
        return _state;
    }
    
    /**
     * Reset receiver
     */
    void reset() {
        std::lock_guard<std::mutex> lock1(_buffer_mutex);
        std::lock_guard<std::mutex> lock2(_frame_mutex);
        
        _rx_buffer.clear();
        _completed_frames.clear();
        _state = RxState::IDLE;
        _symbol_sample_counter = 0;
        _preamble_count = 0;
        _received_dibits.clear();
        
        std::cout << "[RX] Receiver reset" << std::endl;
    }
    
    /**
     * Set frequencies
     */
    void set_frequencies(float f1, float f2, float f3, float f4) {
        _detectors[0]->set_parameters(f1, _sample_rate, _samples_per_symbol);
        _detectors[1]->set_parameters(f2, _sample_rate, _samples_per_symbol);
        _detectors[2]->set_parameters(f3, _sample_rate, _samples_per_symbol);
        _detectors[3]->set_parameters(f4, _sample_rate, _samples_per_symbol);
        std::cout << "[RX] Frequencies updated" << std::endl;
    }
    
    /**
     * Set baudrate
     */
    void set_baudrate(float baudrate) {
        _baudrate = baudrate;
        _samples_per_symbol = static_cast<size_t>(_sample_rate / baudrate);
        
        // Reinitialize Goertzel filters with new block size
        for (auto& detector : _detectors) {
            // Get current frequency (would need to store these)
            // For now, just update the block size parameter
            // This is a simplification - in production you'd store frequencies
        }
        
        std::cout << "[RX] Baudrate updated: " << baudrate << " symbols/s" << std::endl;
    }

private:
    /**
     * Start decoder thread
     */
    void start_decoder() {
        _stop_requested.store(false);
        _is_processing.store(true);
        
        _decoder_thread = std::thread(&FSK4ReceiverBuffer::decoder_loop, this);
        
        std::cout << "[RX] Decoder thread started" << std::endl;
    }
    
    /**
     * Stop decoder thread
     */
    void stop_decoder() {
        _stop_requested.store(true);
        if (_decoder_thread.joinable()) {
            _decoder_thread.join();
        }
    }
    
    /**
     * Decoder loop - runs in separate thread
     * Waits for samples, detects symbols, and decodes frames
     */
    void decoder_loop() {
        std::vector<float> symbol_buffer;
        symbol_buffer.reserve(_samples_per_symbol);
        
        while (!_stop_requested.load()) {
            // Wait for enough samples
            {
                std::lock_guard<std::mutex> lock(_buffer_mutex);
                
                if (_rx_buffer.size() < _samples_per_symbol) {
                    // Not enough samples yet
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
                
                // Extract samples for one symbol
                symbol_buffer.clear();
                for (size_t i = 0; i < _samples_per_symbol && !_rx_buffer.empty(); ++i) {
                    symbol_buffer.push_back(_rx_buffer.front());
                    _rx_buffer.pop_front();
                }
            }
            
            // Detect symbol
            if (symbol_buffer.size() >= _samples_per_symbol) {
                detect_and_process_symbol(symbol_buffer.data(), symbol_buffer.size());
            }
            
            // Check for timeout
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - _last_symbol_time).count();
            
            if (_state != RxState::IDLE && elapsed > 3000) {
                std::cout << "[RX] Timeout - resetting to IDLE" << std::endl;
                reset_state();
            }
        }
        
        _is_processing.store(false);
        std::cout << "[RX] Decoder thread stopped" << std::endl;
    }
    
    /**
     * Detect symbol from samples using Goertzel
     */
    void detect_and_process_symbol(const float* samples, size_t nsamples) {
        // Run Goertzel on all frequencies
        std::array<float, 4> magnitudes;
        for (size_t i = 0; i < 4; ++i) {
            magnitudes[i] = _detectors[i]->process_block(samples, nsamples);
        }
        
        // Find maximum
        size_t max_idx = 0;
        float max_mag = magnitudes[0];
        for (size_t i = 1; i < 4; ++i) {
            if (magnitudes[i] > max_mag) {
                max_mag = magnitudes[i];
                max_idx = i;
            }
        }
        
        Symbol detected = static_cast<Symbol>(max_idx);
        _last_symbol_time = std::chrono::steady_clock::now();
        
        // Process symbol based on state
        process_symbol(detected);
    }
    
    /**
     * Process detected symbol based on state machine
     */
    void process_symbol(Symbol symbol) {
        switch (_state) {
            case RxState::IDLE:
                // Look for preamble
                if (is_preamble_symbol(symbol)) {
                    _preamble_count++;
                    if (_preamble_count >= _min_preamble_symbols) {
                        std::cout << "[RX] Preamble detected (" << _preamble_count 
                                  << " symbols), switching to HEADER" << std::endl;
                        _state = RxState::HEADER;
                        _received_dibits.clear();
                        _preamble_count = 0;
                    }
                } else {
                    _preamble_count = 0;
                }
                break;
                
            case RxState::HEADER:
                _received_dibits.push_back(symbol_to_dibits(symbol));
                if (_received_dibits.size() >= 6) {
                    decode_header();
                    _state = RxState::PAYLOAD;
                }
                break;
                
            case RxState::PAYLOAD:
                _received_dibits.push_back(symbol_to_dibits(symbol));
                size_t payload_dibits = _current_frame.header.payload_length * 4;
                if (_received_dibits.size() >= 6 + payload_dibits) {
                    _state = RxState::CRC;
                }
                break;
                
            case RxState::CRC:
                _received_dibits.push_back(symbol_to_dibits(symbol));
                if (_received_dibits.size() >= 6 + (_current_frame.header.payload_length * 4) + 8) {
                    decode_and_validate_frame();
                }
                break;
                
            case RxState::PREAMBLE_DETECT:
            case RxState::VALIDATE:
                break;
        }
    }
    
    bool is_preamble_symbol(Symbol symbol) {
        return (symbol == Symbol::S00 || symbol == Symbol::S01);
    }
    
    void decode_header() {
        if (_received_dibits.size() < 6) return;
        
        uint8_t hdr_bytes[2];
        hdr_bytes[0] = (_received_dibits[0] << 6) | (_received_dibits[1] << 4) |
                       (_received_dibits[2] << 2) | (_received_dibits[3]);
        hdr_bytes[1] = (_received_dibits[4] << 6) | (_received_dibits[5] << 4);
        
        _current_frame.header = FrameHeader::deserialize(hdr_bytes);
        
        std::cout << "[RX] Header decoded: ID=" 
                  << static_cast<int>(_current_frame.header.frame_id)
                  << ", Payload=" 
                  << static_cast<int>(_current_frame.header.payload_length) 
                  << " bytes" << std::endl;
    }
    
    void decode_and_validate_frame() {
        // Extract payload
        size_t payload_start = 6;
        size_t payload_dibits = _current_frame.header.payload_length * 4;
        
        std::vector<uint8_t> payload_dibit_vec(
            _received_dibits.begin() + payload_start,
            _received_dibits.begin() + payload_start + payload_dibits
        );
        
        _current_frame.payload = dibits_to_bytes(payload_dibit_vec);
        
        // Extract CRC
        size_t crc_start = payload_start + payload_dibits;
        uint8_t crc_bytes[2];
        crc_bytes[0] = (_received_dibits[crc_start + 0] << 6) |
                       (_received_dibits[crc_start + 1] << 4) |
                       (_received_dibits[crc_start + 2] << 2) |
                       (_received_dibits[crc_start + 3]);
        crc_bytes[1] = (_received_dibits[crc_start + 4] << 6) |
                       (_received_dibits[crc_start + 5] << 4) |
                       (_received_dibits[crc_start + 6] << 2) |
                       (_received_dibits[crc_start + 7]);
        
        _current_frame.crc = (static_cast<uint16_t>(crc_bytes[0]) << 8) | crc_bytes[1];
        
        // Validate
        if (_current_frame.verify()) {
            std::cout << "[RX] ✓ Frame received successfully! ID=" 
                      << static_cast<int>(_current_frame.header.frame_id)
                      << ", CRC=0x" << std::hex << _current_frame.crc << std::dec << std::endl;
            
            std::lock_guard<std::mutex> lock(_frame_mutex);
            _completed_frames.push_back(_current_frame);
        } else {
            std::cout << "[RX] ✗ CRC error - frame discarded" << std::endl;
        }
        
        reset_state();
    }
    
    void reset_state() {
        _state = RxState::IDLE;
        _received_dibits.clear();
        _preamble_count = 0;
        _current_frame = Frame();
    }

private:
    float _baudrate;
    float _sample_rate;
    size_t _block_size;
    size_t _samples_per_symbol;
    
    // Circular buffer for RX samples
    boost::circular_buffer<float> _rx_buffer;
    mutable std::mutex _buffer_mutex;
    
    // Goertzel detectors
    std::array<std::unique_ptr<goertzel_filter>, 4> _detectors;
    
    // State machine
    RxState _state;
    size_t _symbol_sample_counter;
    size_t _preamble_count;
    size_t _min_preamble_symbols;
    
    // Frame decoding
    std::vector<uint8_t> _received_dibits;
    Frame _current_frame;
    std::deque<Frame> _completed_frames;
    mutable std::mutex _frame_mutex;
    
    // Decoder thread
    std::thread _decoder_thread;
    std::atomic<bool> _is_processing;
    std::atomic<bool> _stop_requested;
    std::chrono::steady_clock::time_point _last_symbol_time;
};

} // namespace fsk4

#endif // FSK4_SYSTEM_H