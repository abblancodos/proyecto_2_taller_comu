/**
 * mainwindow.h
 *
 * Copyright (C) 2023-2024  Pablo Alvarado
 * EL5805 Procesamiento Digital de Señales
 * Escuela de Ingeniería Electrónica
 * Tecnológico de Costa Rica
 *
 * All rights reserved.
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <iostream>
#include <QTimer>
#include <filesystem>
#include <memory>
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>

#include "dsp_client.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    /**
     * Add file to playlist
     */
    bool add_file(const std::filesystem::path& file);

private slots:
    // File controls
    void on_fileButton_clicked();
    void on_fileEdit_returnPressed();
    
    // Volume controls
    void on_volumeDial_sliderMoved(int value);
    void on_volumeSpin_valueChanged(int value);
    
    // Playback control buttons
    void on_start_modulation_pbutton_clicked();
    void on_stop_modulation_pbutton_clicked();
    void on_passthrough_mode_pbutton_clicked();
    
    // Radio buttons for transmit/receive
    void on_transmitButton_toggled(bool checked);
    void on_receivedButton_toggled(bool checked);
    
    // Transmit controls
    void on_transmit_carrier_freq_spinbox_valueChanged(int value);
    void on_transmit_modulation_scheme_combobox_currentIndexChanged(int index);
    
    // Receive controls
    void on_receive_carrier_freq_spinbox_valueChanged(int value);
    void on_receive_modulation_scheme_combobox_currentIndexChanged(int index);
    
    // Update timer
    void on_update_timer();

      // NEW: Gain control slots
    void on_modulation_gain_dial_valueChanged(int value);
    void on_modulation_gain_spinbox_valueChanged(int value);

    // NEW: FSK carrier frequency slots - TRANSMIT
    void on_f1_tx_fsk_carrier_spinBox_valueChanged(int value);
    void on_f2_tx_fsk_carrier_spinBox_valueChanged(int value);
    void on_f3_tx_fsk_carrier_spinBox_valueChanged(int value);
    void on_f4_tx_fsk_carrier_spinBox_valueChanged(int value);

    // NEW: FSK carrier frequency slots - RECEIVE
    void on_f1_rx_fsk_carrier_spinBox_valueChanged(int value);
    void on_f2_rx_fsk_carrier_spinBox_valueChanged(int value);
    void on_f3_rx_fsk_carrier_spinBox_valueChanged(int value);
    void on_f4_rx_fsk_carrier_spinBox_valueChanged(int value);

private:
    Ui::MainWindow *ui;
    
    // The one and only client instance
    static dsp_client _client;
    
    // Selected files
    QStringList _selectedFiles;
    
    // Timer for display updates
    std::unique_ptr<QTimer> _timer;
    
    // Time values for plotting
    QVector<double> _times;

    // NEW: FSK Decoder Thread
    // ========================
    std::thread _fsk_decoder_thread;
    std::atomic<bool> _fsk_decoder_running;
    
    void fsk_decoder_loop();
    void start_fsk_decoder();
    void stop_fsk_decoder();
    
    // Decoded data handling
    struct DecodedSymbol {
        int symbol;           // 0, 1, 2, or 3
        uint64_t block_num;   // Block counter
        float snr;            // Signal quality
        std::array<float, 4> magnitudes;
    };
    
    std::queue<DecodedSymbol> _decoded_symbols;
    std::mutex _decoded_mutex;
    
    void process_decoded_symbol(const DecodedSymbol& sym);
    void update_fsk_display(const DecodedSymbol& sym);
    
    // NEW: FSK Transmit Queue
    // =======================
    std::queue<int> _tx_symbol_queue;
    std::mutex _tx_queue_mutex;
    std::unique_ptr<QTimer> _fsk_update_timer;
    
    void queue_tx_symbol(int symbol);
    void process_tx_queue();
    
    // Data buffers for FSK communication
    std::vector<uint8_t> _rx_bit_buffer;
    std::vector<uint8_t> _tx_bit_buffer;
    
    // Statistics
    uint64_t _last_block_processed;
    uint64_t _symbols_received;
    uint64_t _symbols_transmitted;
    uint64_t _errors_detected;
    
    // Helper methods
    void convert_symbol_to_bits(int symbol, uint8_t& bit1, uint8_t& bit0);
    int convert_bits_to_symbol(uint8_t bit1, uint8_t bit0);
    void update_fsk_statistics();


    // NEW: Helper methods for FSK UI visibility
    void update_tx_ui_visibility(int modulation_index);
    void update_rx_ui_visibility(int modulation_index);
    void setup_fsk_carrier_spinboxes();
    
    // NEW: Current modulation gain (avoid magic numbers)
    float _current_gain;
};

#endif // MAINWINDOW_H