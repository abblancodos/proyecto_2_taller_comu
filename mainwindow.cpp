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
#include <iostream>

// The one and only client instance belongs to the main window
dsp_client MainWindow::_client;

MainWindow::MainWindow(QWidget *parent)
  : QMainWindow(parent)
  , ui(new Ui::MainWindow)
  , _fsk_decoder_running(false)
  , _last_block_processed(0)
  , _symbols_received(0)
  , _symbols_transmitted(0)
  , _errors_detected(0)
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

    
  // Set gain dial and spinbox ranges (0 to 30)
  ui->modulation_gain_dial->setRange(0, 30);
  ui->modulation_gain_dial->setValue(1);  // Default gain = 1
  ui->modulation_gain_dial->setNotchesVisible(true);
  
  ui->modulation_gain_spinbox->setRange(0, 30);
  ui->modulation_gain_spinbox->setValue(1);
  ui->modulation_gain_spinbox->setSuffix("x");  // Shows "1x", "2x", etc.
  
  ui->transmit_modulation_scheme_combobox->addItems(modulation_schemes);
  ui->receive_modulation_scheme_combobox->addItems(modulation_schemes);

  // Set carrier frequency spinbox ranges
  ui->transmit_carrier_freq_spinbox->setRange(100, 20000);
  ui->transmit_carrier_freq_spinbox->setValue(1000);
  ui->transmit_carrier_freq_spinbox->setSuffix(" Hz");
  
  ui->receive_carrier_freq_spinbox->setRange(100, 20000);
  ui->receive_carrier_freq_spinbox->setValue(1000);
  ui->receive_carrier_freq_spinbox->setSuffix(" Hz");

  setup_fsk_carrier_spinboxes();

  // Initialize radio buttons - receive is default
  ui->receivedButton->setChecked(true);
  _client.set_mode(dsp_client::Mode::Receive);

  // NEW: Initially hide FSK controls (SSB is default at index 0)
  update_tx_ui_visibility(0);
  update_rx_ui_visibility(0);

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
  connect(ui->process_file_pButton, SIGNAL(clicked()),
          this, SLOT(on_process_file_pButton_clicked()));
  connect(ui->process_mic_pButton, SIGNAL(clicked()),
          this, SLOT(on_process_mic_pButton_clicked()));
  
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

  // Connect gain controls BEFORE setting initial values
  connect(ui->modulation_gain_dial, SIGNAL(valueChanged(int)),
          this, SLOT(on_modulation_gain_dial_valueChanged(int)));
  connect(ui->modulation_gain_spinbox, SIGNAL(valueChanged(int)),
          this, SLOT(on_modulation_gain_spinbox_valueChanged(int)));

  // NEW: FSK TRANSMIT carrier frequency controls
  connect(ui->f1_tx_fsk_carrier_spinBox, SIGNAL(valueChanged(int)),
          this, SLOT(on_f1_tx_fsk_carrier_spinBox_valueChanged(int)));
  connect(ui->f2_tx_fsk_carrier_spinBox, SIGNAL(valueChanged(int)),
          this, SLOT(on_f2_tx_fsk_carrier_spinBox_valueChanged(int)));
  connect(ui->f3_tx_fsk_carrier_spinBox, SIGNAL(valueChanged(int)),
          this, SLOT(on_f3_tx_fsk_carrier_spinBox_valueChanged(int)));
  connect(ui->f4_tx_fsk_carrier_spinBox, SIGNAL(valueChanged(int)),
          this, SLOT(on_f4_tx_fsk_carrier_spinBox_valueChanged(int)));

  // NEW: FSK RECEIVE carrier frequency controls
  connect(ui->f1_rx_fsk_carrier_spinBox, SIGNAL(valueChanged(int)),
          this, SLOT(on_f1_rx_fsk_carrier_spinBox_valueChanged(int)));
  connect(ui->f2_rx_fsk_carrier_spinBox, SIGNAL(valueChanged(int)),
          this, SLOT(on_f2_rx_fsk_carrier_spinBox_valueChanged(int)));
  connect(ui->f3_rx_fsk_carrier_spinBox, SIGNAL(valueChanged(int)),
          this, SLOT(on_f3_rx_fsk_carrier_spinBox_valueChanged(int)));
  connect(ui->f4_rx_fsk_carrier_spinBox, SIGNAL(valueChanged(int)),
          this, SLOT(on_f4_rx_fsk_carrier_spinBox_valueChanged(int)));


  // Update timer - 30 fps for display
  _timer = std::make_unique<QTimer>(this);
  connect(_timer.get(), SIGNAL(timeout()), this, SLOT(on_update_timer()));
  _timer->start(33); // approximately 30 fps (1000ms/30 = 33.33ms)


}

MainWindow::~MainWindow()
{
  _client.stop();
  delete ui;
}

// Play button - start processing
void MainWindow::on_start_modulation_pbutton_clicked() {
  // Determine the mode based on current radio button selection
  if (ui->transmitButton->isChecked()) {
    _client.set_mode(dsp_client::Mode::Transmit);
    std::cout << "Starting modulation in TRANSMIT mode" << std::endl;
  } else if (ui->receivedButton->isChecked()) {
    _client.set_mode(dsp_client::Mode::Receive);
    std::cout << "Starting demodulation in RECEIVE mode" << std::endl;
  } else {
    // If neither is checked, assume passthrough
    _client.set_mode(dsp_client::Mode::Passthrough);
    std::cout << "Starting in PASSTHROUGH mode" << std::endl;
  }
  
  _client.start_processing();
}

// Stop button - stop processing
void MainWindow::on_stop_modulation_pbutton_clicked() {
  _client.stop_processing();
  _client.stop_files();
  std::cout << "Processing stopped" << std::endl;
}


// Passthrough button - enable passthrough mode
void MainWindow::on_passthrough_mode_pbutton_clicked() {
  // Stop any current processing
  _client.stop_processing();
  
  // Uncheck both radio buttons to indicate passthrough mode
  ui->transmitButton->setChecked(false);
  ui->receivedButton->setChecked(false);
  
  // Set passthrough mode and start processing
  _client.set_mode(dsp_client::Mode::Passthrough);
  _client.start_processing();
  
  std::cout << "Passthrough mode activated - input passes directly to output" << std::endl;
}

// Play from file button - start playing from selected audio file
void MainWindow::on_process_file_pButton_clicked() {
  // First, make sure we have a file selected
  if (ui->fileEdit->text().isEmpty()) {
    std::cerr << "No file selected! Please select an audio file first." << std::endl;
    return;
  }
  
  // Stop any current processing
  _client.stop_processing();
  
  // Set the mode based on current radio button selection
  if (ui->transmitButton->isChecked()) {
    _client.set_mode(dsp_client::Mode::Transmit);
    std::cout << "Playing from file with TRANSMIT modulation" << std::endl;
  } else if (ui->receivedButton->isChecked()) {
    _client.set_mode(dsp_client::Mode::Receive);
    std::cout << "Playing from file with RECEIVE demodulation" << std::endl;
  } else {
    // Default to transmit if nothing is selected
    ui->transmitButton->setChecked(true);
    _client.set_mode(dsp_client::Mode::Transmit);
    std::cout << "Playing from file with TRANSMIT modulation (default)" << std::endl;
  }
  
  // Start processing from file
  _client.start_processing();
  
  std::cout << "  File: " << ui->fileEdit->text().toStdString() << std::endl;
}

// Play from microphone button - start processing microphone input
void MainWindow::on_process_mic_pButton_clicked() {
  // Stop file playback if active
  _client.stop_files();
  
  // Stop any current processing
  _client.stop_processing();
  
  // Set the mode based on current radio button selection
  if (ui->transmitButton->isChecked()) {
    _client.set_mode(dsp_client::Mode::Transmit);
    std::cout << "Processing microphone input with TRANSMIT modulation" << std::endl;
  } else if (ui->receivedButton->isChecked()) {
    _client.set_mode(dsp_client::Mode::Receive);
    std::cout << "Processing microphone input with RECEIVE demodulation" << std::endl;
  } else {
    // Default to transmit if nothing is selected
    ui->transmitButton->setChecked(true);
    _client.set_mode(dsp_client::Mode::Transmit);
    std::cout << "Processing microphone input with TRANSMIT modulation (default)" << std::endl;
  }
  
  // Start processing from microphone
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

// ============================================================================
// NEW: FSK CARRIER SPINBOX SETUP
// ============================================================================

void MainWindow::setup_fsk_carrier_spinboxes() {
  // Configure TRANSMIT FSK carrier spinboxes
  ui->f1_tx_fsk_carrier_spinBox->setRange(100, 20000);
  ui->f1_tx_fsk_carrier_spinBox->setValue(1000);
  ui->f1_tx_fsk_carrier_spinBox->setSuffix(" Hz");
  ui->f1_tx_fsk_carrier_spinBox->setPrefix("f1: ");
  
  ui->f2_tx_fsk_carrier_spinBox->setRange(100, 20000);
  ui->f2_tx_fsk_carrier_spinBox->setValue(2000);
  ui->f2_tx_fsk_carrier_spinBox->setSuffix(" Hz");
  ui->f2_tx_fsk_carrier_spinBox->setPrefix("f2: ");
  
  ui->f3_tx_fsk_carrier_spinBox->setRange(100, 20000);
  ui->f3_tx_fsk_carrier_spinBox->setValue(3000);
  ui->f3_tx_fsk_carrier_spinBox->setSuffix(" Hz");
  ui->f3_tx_fsk_carrier_spinBox->setPrefix("f3: ");
  
  ui->f4_tx_fsk_carrier_spinBox->setRange(100, 20000);
  ui->f4_tx_fsk_carrier_spinBox->setValue(4000);
  ui->f4_tx_fsk_carrier_spinBox->setSuffix(" Hz");
  ui->f4_tx_fsk_carrier_spinBox->setPrefix("f4: ");

  // Configure RECEIVE FSK carrier spinboxes
  ui->f1_rx_fsk_carrier_spinBox->setRange(100, 20000);
  ui->f1_rx_fsk_carrier_spinBox->setValue(1000);
  ui->f1_rx_fsk_carrier_spinBox->setSuffix(" Hz");
  ui->f1_rx_fsk_carrier_spinBox->setPrefix("f1: ");
  
  ui->f2_rx_fsk_carrier_spinBox->setRange(100, 20000);
  ui->f2_rx_fsk_carrier_spinBox->setValue(2000);
  ui->f2_rx_fsk_carrier_spinBox->setSuffix(" Hz");
  ui->f2_rx_fsk_carrier_spinBox->setPrefix("f2: ");
  
  ui->f3_rx_fsk_carrier_spinBox->setRange(100, 20000);
  ui->f3_rx_fsk_carrier_spinBox->setValue(3000);
  ui->f3_rx_fsk_carrier_spinBox->setSuffix(" Hz");
  ui->f3_rx_fsk_carrier_spinBox->setPrefix("f3: ");
  
  ui->f4_rx_fsk_carrier_spinBox->setRange(100, 20000);
  ui->f4_rx_fsk_carrier_spinBox->setValue(4000);
  ui->f4_rx_fsk_carrier_spinBox->setSuffix(" Hz");
  ui->f4_rx_fsk_carrier_spinBox->setPrefix("f4: ");
}

// ============================================================================
// NEW: UI VISIBILITY CONTROL
// ============================================================================

void MainWindow::update_tx_ui_visibility(int modulation_index) {
  bool is_fsk_mode = (modulation_index == 4);  // FSK is index 4
  bool is_ssb_mode = (modulation_index < 4);   // SSB modes are 0-3
  
  // Show/hide carrier frequency for SSB (not used in FSK)
  ui->label_2->setVisible(is_ssb_mode);
  ui->transmit_carrier_freq_spinbox->setVisible(is_ssb_mode);
  
  // Show/hide FSK carrier label and spinboxes
  ui->label_6->setVisible(is_fsk_mode);  // "4-FSK Carriers" label
  ui->f1_tx_fsk_carrier_spinBox->setVisible(is_fsk_mode);
  ui->f2_tx_fsk_carrier_spinBox->setVisible(is_fsk_mode);
  ui->f3_tx_fsk_carrier_spinBox->setVisible(is_fsk_mode);
  ui->f4_tx_fsk_carrier_spinBox->setVisible(is_fsk_mode);
}

void MainWindow::update_rx_ui_visibility(int modulation_index) {
  bool is_fsk_mode = (modulation_index == 4);  // FSK is index 4
  bool is_ssb_mode = (modulation_index < 4);   // SSB modes are 0-3
  
  // Show/hide carrier frequency for SSB (not used in FSK)
  ui->label_3->setVisible(is_ssb_mode);
  ui->receive_carrier_freq_spinbox->setVisible(is_ssb_mode);
  
  // Show/hide FSK carrier label and spinboxes
  ui->label_7->setVisible(is_fsk_mode);  // "4-FSK Carriers" label
  ui->f1_rx_fsk_carrier_spinBox->setVisible(is_fsk_mode);
  ui->f2_rx_fsk_carrier_spinBox->setVisible(is_fsk_mode);
  ui->f3_rx_fsk_carrier_spinBox->setVisible(is_fsk_mode);
  ui->f4_rx_fsk_carrier_spinBox->setVisible(is_fsk_mode);
}

// ============================================================================
// NEW: GAIN CONTROL SLOTS
// ============================================================================

void MainWindow::on_modulation_gain_dial_valueChanged(int value) {
  // Sync spinbox with dial (without triggering another signal)
  ui->modulation_gain_spinbox->blockSignals(true);
  ui->modulation_gain_spinbox->setValue(value);
  ui->modulation_gain_spinbox->blockSignals(false);
  
  // Update gain in DSP client
  _current_gain = static_cast<float>(value);
  _client.set_modulation_gain(_current_gain);
  
  std::cout << "Modulation gain set to: " << _current_gain << "x" << std::endl;
}

void MainWindow::on_modulation_gain_spinbox_valueChanged(int value) {
  // Sync dial with spinbox (without triggering another signal)
  ui->modulation_gain_dial->blockSignals(true);
  ui->modulation_gain_dial->setValue(value);
  ui->modulation_gain_dial->blockSignals(false);
  
  // Update gain in DSP client
  _current_gain = static_cast<float>(value);
  _client.set_modulation_gain(_current_gain);
  
  std::cout << "Modulation gain set to: " << _current_gain << "x" << std::endl;
}

// ============================================================================
// NEW: FSK TRANSMIT CARRIER FREQUENCY SLOTS
// ============================================================================

void MainWindow::on_f1_tx_fsk_carrier_spinBox_valueChanged(int value) {
  // Get current frequencies
  auto freqs = _client.get_tx_fsk4_frequencies();
  freqs[0] = static_cast<float>(value);
  
  // Update in DSP client
  _client.set_tx_fsk4_frequencies(freqs[0], freqs[1], freqs[2], freqs[3]);
  
  std::cout << "TX FSK f1 set to: " << value << " Hz" << std::endl;
}

void MainWindow::on_f2_tx_fsk_carrier_spinBox_valueChanged(int value) {
  auto freqs = _client.get_tx_fsk4_frequencies();
  freqs[1] = static_cast<float>(value);
  _client.set_tx_fsk4_frequencies(freqs[0], freqs[1], freqs[2], freqs[3]);
  std::cout << "TX FSK f2 set to: " << value << " Hz" << std::endl;
}

void MainWindow::on_f3_tx_fsk_carrier_spinBox_valueChanged(int value) {
  auto freqs = _client.get_tx_fsk4_frequencies();
  freqs[2] = static_cast<float>(value);
  _client.set_tx_fsk4_frequencies(freqs[0], freqs[1], freqs[2], freqs[3]);
  std::cout << "TX FSK f3 set to: " << value << " Hz" << std::endl;
}

void MainWindow::on_f4_tx_fsk_carrier_spinBox_valueChanged(int value) {
  auto freqs = _client.get_tx_fsk4_frequencies();
  freqs[3] = static_cast<float>(value);
  _client.set_tx_fsk4_frequencies(freqs[0], freqs[1], freqs[2], freqs[3]);
  std::cout << "TX FSK f4 set to: " << value << " Hz" << std::endl;
}

// ============================================================================
// NEW: FSK RECEIVE CARRIER FREQUENCY SLOTS
// ============================================================================

void MainWindow::on_f1_rx_fsk_carrier_spinBox_valueChanged(int value) {
  auto freqs = _client.get_rx_fsk4_frequencies();
  freqs[0] = static_cast<float>(value);
  _client.set_rx_fsk4_frequencies(freqs[0], freqs[1], freqs[2], freqs[3]);
  std::cout << "RX FSK f1 set to: " << value << " Hz" << std::endl;
}

void MainWindow::on_f2_rx_fsk_carrier_spinBox_valueChanged(int value) {
  auto freqs = _client.get_rx_fsk4_frequencies();
  freqs[1] = static_cast<float>(value);
  _client.set_rx_fsk4_frequencies(freqs[0], freqs[1], freqs[2], freqs[3]);
  std::cout << "RX FSK f2 set to: " << value << " Hz" << std::endl;
}

void MainWindow::on_f3_rx_fsk_carrier_spinBox_valueChanged(int value) {
  auto freqs = _client.get_rx_fsk4_frequencies();
  freqs[2] = static_cast<float>(value);
  _client.set_rx_fsk4_frequencies(freqs[0], freqs[1], freqs[2], freqs[3]);
  std::cout << "RX FSK f3 set to: " << value << " Hz" << std::endl;
}

void MainWindow::on_f4_rx_fsk_carrier_spinBox_valueChanged(int value) {
  auto freqs = _client.get_rx_fsk4_frequencies();
  freqs[3] = static_cast<float>(value);
  _client.set_rx_fsk4_frequencies(freqs[0], freqs[1], freqs[2], freqs[3]);
  std::cout << "RX FSK f4 set to: " << value << " Hz" << std::endl;
}

// ============================================================================
// MODIFIED: MODULATION SCHEME CHANGE HANDLERS
// ============================================================================

void MainWindow::on_transmit_modulation_scheme_combobox_currentIndexChanged(int index) {
  // FIRST: Update UI visibility
  update_tx_ui_visibility(index);
  
  // THEN: Set modulation scheme in DSP client
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

void MainWindow::on_receive_modulation_scheme_combobox_currentIndexChanged(int index) {
  // FIRST: Update UI visibility
  update_rx_ui_visibility(index);
  
  // THEN: Set modulation scheme in DSP client
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