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
#include <QTimer>
#include <memory>

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
};

#endif // MAINWINDOW_H