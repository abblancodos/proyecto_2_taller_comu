/**
 * mainwindow.cpp
 *
 * Copyright (C) 2023-2024  Pablo Alvarado
 * EL5805 Procesamiento Digital de Señales
 * Escuela de Ingeniería Electrónica
 * Tecnológico de Costa Rica
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the authors nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <cmath>

// The one and only client instance belongs to the main window
dsp_client MainWindow::_client;

MainWindow::MainWindow(QWidget *parent)
  : QMainWindow(parent)
  , ui(new Ui::MainWindow)
{
  if (_client.init() != jack::client_state::Running) {
    throw std::runtime_error("Could not initialize the JACK client");
  }
  
  ui->setupUi(this);

  // Setup the CustomPlot stuff
  ui->crt_plot->addGraph();

  const double ts = 1.0/_client.sample_rate();
  double time_window = double(_client.buffer_size()-1)*ts;
  ui->crt_plot->xAxis->setRange(0,time_window);
  ui->crt_plot->yAxis->setRange(-1.0,1.0);

  _times.resize(_client.buffer_size());
  for (int i=0;i<_times.size();++i) {
    _times[i]=double(i)*ts;
  }

  // Populate modulation scheme combo boxes
  QStringList modulation_schemes;
  modulation_schemes << "SSB Upper Sideband"
                     << "SSB Lower Sideband"
                     << "SSB USB Suppressed Carrier"
                     << "SSB LSB Suppressed Carrier"
                     << "4-FSK";
  
  ui->transmit_modulation_scheme_combobox->addItems(modulation_schemes);
  ui->receive_modulation_scheme_combobox->addItems(modulation_schemes);

  // Set carrier frequency spinbox ranges
  ui->transmit_carrier_freq_spinbox->setRange(100, 20000);
  ui->transmit_carrier_freq_spinbox->setValue(1000);
  ui->transmit_carrier_freq_spinbox->setSuffix(" Hz");
  
  ui->receive_carrier_freq_spinbox->setRange(100, 20000);
  ui->receive_carrier_freq_spinbox->setValue(1000);
  ui->receive_carrier_freq_spinbox->setSuffix(" Hz");

  // Initialize radio buttons - receive is default
  ui->receivedButton->setChecked(true);
  _client.set_mode(dsp_client::Mode::Receive);

  // Connect all UI signals to slots
  // File controls
  connect(ui->fileButton, SIGNAL(clicked()), 
          this, SLOT(on_fileButton_clicked()));
  connect(ui->fileEdit, SIGNAL(returnPressed()), 
          this, SLOT(on_fileEdit_returnPressed()));
  
  // Volume controls
  connect(ui->volumeDial, SIGNAL(sliderMoved(int)), 
          this, SLOT(on_volumeDial_sliderMoved(int)));
  connect(ui->volumeSpin, SIGNAL(valueChanged(int)), 
          this, SLOT(on_volumeSpin_valueChanged(int)));
  
  // Playback control buttons
  connect(ui->start_modulation_pbutton, SIGNAL(clicked()), 
          this, SLOT(on_start_modulation_pbutton_clicked()));
  connect(ui->stop_modulation_pbutton, SIGNAL(clicked()), 
          this, SLOT(on_stop_modulation_pbutton_clicked()));
  connect(ui->passthrough_mode_pbutton, SIGNAL(clicked()), 
          this, SLOT(on_passthrough_mode_pbutton_clicked()));
  
  // Radio buttons for transmit/receive
  connect(ui->transmitButton, SIGNAL(toggled(bool)), 
          this, SLOT(on_transmitButton_toggled(bool)));
  connect(ui->receivedButton, SIGNAL(toggled(bool)), 
          this, SLOT(on_receivedButton_toggled(bool)));
  
  // Transmit controls
  connect(ui->transmit_carrier_freq_spinbox, SIGNAL(valueChanged(int)), 
          this, SLOT(on_transmit_carrier_freq_spinbox_valueChanged(int)));
  connect(ui->transmit_modulation_scheme_combobox, SIGNAL(currentIndexChanged(int)), 
          this, SLOT(on_transmit_modulation_scheme_combobox_currentIndexChanged(int)));
  
  // Receive controls
  connect(ui->receive_carrier_freq_spinbox, SIGNAL(valueChanged(int)), 
          this, SLOT(on_receive_carrier_freq_spinbox_valueChanged(int)));
  connect(ui->receive_modulation_scheme_combobox, SIGNAL(currentIndexChanged(int)), 
          this, SLOT(on_receive_modulation_scheme_combobox_currentIndexChanged(int)));

  // Update timer - 30 fps for display
  _timer = std::make_unique<QTimer>(this);
  connect(_timer.get(), SIGNAL(timeout()), this, SLOT(on_update_timer()));
  _timer->start(33); // approximately 30 fps (1000ms/30 = 33.33ms)
}

MainWindow::~MainWindow()
{
    _client.stop();
}

// Play button - start processing
void MainWindow::on_start_modulation_pbutton_clicked() {
  _client.start_processing();
}

// Stop button - stop processing
void MainWindow::on_stop_modulation_pbutton_clicked() {
  _client.stop_processing();
  _client.stop_files();
}

// Passthrough button - enable passthrough mode
void MainWindow::on_passthrough_mode_pbutton_clicked() {
  ui->transmitButton->setChecked(false);
  ui->receivedButton->setChecked(false);
  _client.set_mode(dsp_client::Mode::Passthrough);
  _client.start_processing();
}

// Transmit radio button
void MainWindow::on_transmitButton_toggled(bool checked) {
  if (checked) {
    ui->receivedButton->setChecked(false);
    _client.set_mode(dsp_client::Mode::Transmit);
    
    // When transmit is active, ensure we're using the file as input
    // The file should only be audible in passthrough mode with bypass
  }
}

// Receive radio button
void MainWindow::on_receivedButton_toggled(bool checked) {
  if (checked) {
    ui->transmitButton->setChecked(false);
    _client.set_mode(dsp_client::Mode::Receive);
    
    // When receive is active, input comes from microphone
  }
}

// Transmit carrier frequency changed
void MainWindow::on_transmit_carrier_freq_spinbox_valueChanged(int value) {
  _client.set_transmit_carrier_freq(static_cast<float>(value));
}

// Receive carrier frequency changed
void MainWindow::on_receive_carrier_freq_spinbox_valueChanged(int value) {
  _client.set_receive_carrier_freq(static_cast<float>(value));
}

// Transmit modulation scheme changed
void MainWindow::on_transmit_modulation_scheme_combobox_currentIndexChanged(int index) {
  dsp_client::ModulationScheme scheme;
  switch(index) {
    case 0: scheme = dsp_client::ModulationScheme::SSB_USB; break;
    case 1: scheme = dsp_client::ModulationScheme::SSB_LSB; break;
    case 2: scheme = dsp_client::ModulationScheme::SSB_USB_SC; break;
    case 3: scheme = dsp_client::ModulationScheme::SSB_LSB_SC; break;
    case 4: scheme = dsp_client::ModulationScheme::FSK_4; break;
    default: scheme = dsp_client::ModulationScheme::SSB_USB;
  }
  _client.set_transmit_modulation(scheme);
}

// Receive modulation scheme changed
void MainWindow::on_receive_modulation_scheme_combobox_currentIndexChanged(int index) {
  dsp_client::ModulationScheme scheme;
  switch(index) {
    case 0: scheme = dsp_client::ModulationScheme::SSB_USB; break;
    case 1: scheme = dsp_client::ModulationScheme::SSB_LSB; break;
    case 2: scheme = dsp_client::ModulationScheme::SSB_USB_SC; break;
    case 3: scheme = dsp_client::ModulationScheme::SSB_LSB_SC; break;
    case 4: scheme = dsp_client::ModulationScheme::FSK_4; break;
    default: scheme = dsp_client::ModulationScheme::SSB_USB;
  }
  _client.set_receive_modulation(scheme);
}

// File selection
bool MainWindow::add_file(const std::filesystem::path& file) {
  ui->fileEdit->setText(file.c_str());
  return _client.add_file(file);
}

void MainWindow::on_fileButton_clicked() {
  _selectedFiles =
      QFileDialog::getOpenFileNames(this,
                                   "Select one or more audio files to open",
                                   ui->fileEdit->text(),
                                   "WAV Files (*.wav)");

  if (!_selectedFiles.empty()) {
    ui->fileEdit->setText(*_selectedFiles.begin());

    _client.stop_files();
    QStringList::iterator it;
    for (it=_selectedFiles.begin();it!=_selectedFiles.end();++it) {
      std::filesystem::path tmp(qPrintable(*it));
      _client.add_file(tmp.c_str());
    }
  }
}

void MainWindow::on_fileEdit_returnPressed() {
  _client.stop_files();

  std::filesystem::path tmp(qPrintable(ui->fileEdit->text()));
  if (!tmp.empty()) {
    _client.add_file(tmp.c_str());
  }
}

// Volume controls
void MainWindow::on_volumeDial_sliderMoved(int value) {
  ui->volumeSpin->setValue(value);
  _client.set_volume(float(value)/100.0f);
}

void MainWindow::on_volumeSpin_valueChanged(int value) { 
  ui->volumeDial->setValue(value);  
  _client.set_volume(float(value)/100.0f);
}

// Update timer - runs at ~30 fps
void MainWindow::on_update_timer() {
  float p = std::sqrt(_client.power());
  ui->powerBar->setValue(static_cast<int>(p*100));

  QVector<double> vals(_times.size());
  const std::vector<float>& lastvals=_client.last_buffer();
  for (int i=0;i<vals.size();++i) {
    vals[i]=double(lastvals[i]);
  }

  ui->crt_plot->graph(0)->setData(_times,vals);
  ui->crt_plot->replot(QCustomPlot::rpQueuedReplot);
}