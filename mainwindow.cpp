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
#include "digital_link.h"
#include "jack_port_config.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <QMessageBox>
#include <cmath>
#include <sndfile.h>

// The one and only client instance belongs to the main window
dsp_client MainWindow::_client;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), _selectedFiles(),
      _timer(std::make_unique<QTimer>(this)), _times(),
      _tx_fsk_timer(std::make_unique<QTimer>(this)), _fsk_decoder_thread(),
      _fsk_decoder_running(false),
      _digital_tx_timer(std::make_unique<QTimer>(this)), _wav_buffer(),
      _is_transmitting_digital(false), _tx_symbol_counter(0),
      _decoded_symbols(), _tx_symbol_queue() // ok así
      ,
      _fsk_update_timer(std::make_unique<QTimer>(this)), _rx_bit_buffer(),
      _tx_bit_buffer(), _last_block_processed(0), _symbols_received(0),
      _symbols_transmitted(0), _errors_detected(0), _current_gain(1.0f),
      _loaded_wav_path() {
  if (_client.init() != jack::client_state::Running) {
    throw std::runtime_error("Could not initialize the JACK client");
  }

  ui->setupUi(this);

  // Setup the CustomPlot stuff
  ui->crt_plot->addGraph();

  const double ts = 1.0 / _client.sample_rate();
  double time_window = double(_client.buffer_size() - 1) * ts;
  ui->crt_plot->xAxis->setRange(0, time_window);
  ui->crt_plot->yAxis->setRange(-1.0, 1.0);

  _times.resize(_client.buffer_size());
  for (int i = 0; i < _times.size(); ++i) {
    _times[i] = double(i) * ts;
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
  ui->modulation_gain_dial->setValue(1); // Default gain = 1
  ui->modulation_gain_dial->setNotchesVisible(true);

  ui->modulation_gain_spinbox->setRange(0, 30);
  ui->modulation_gain_spinbox->setValue(1);
  ui->modulation_gain_spinbox->setSuffix("x"); // Shows "1x", "2x", etc.

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
  connect(ui->fileButton, SIGNAL(clicked()), this,
          SLOT(on_fileButton_clicked()));
  connect(ui->fileEdit, SIGNAL(returnPressed()), this,
          SLOT(on_fileEdit_returnPressed()));

  // Volume controls
  connect(ui->volumeDial, SIGNAL(sliderMoved(int)), this,
          SLOT(on_volumeDial_sliderMoved(int)));
  connect(ui->volumeSpin, SIGNAL(valueChanged(int)), this,
          SLOT(on_volumeSpin_valueChanged(int)));

  // Playback control buttons
  connect(ui->start_modulation_pbutton, SIGNAL(clicked()), this,
          SLOT(on_start_modulation_pbutton_clicked()));
  connect(ui->stop_modulation_pbutton, SIGNAL(clicked()), this,
          SLOT(on_stop_modulation_pbutton_clicked()));
  connect(ui->passthrough_mode_pbutton, SIGNAL(clicked()), this,
          SLOT(on_passthrough_mode_pbutton_clicked()));

  // Radio buttons for transmit/receive
  connect(ui->transmitButton, SIGNAL(toggled(bool)), this,
          SLOT(on_transmitButton_toggled(bool)));
  connect(ui->receivedButton, SIGNAL(toggled(bool)), this,
          SLOT(on_receivedButton_toggled(bool)));

  // Transmit controls
  connect(ui->transmit_carrier_freq_spinbox, SIGNAL(valueChanged(int)), this,
          SLOT(on_transmit_carrier_freq_spinbox_valueChanged(int)));
  connect(
      ui->transmit_modulation_scheme_combobox, SIGNAL(currentIndexChanged(int)),
      this,
      SLOT(on_transmit_modulation_scheme_combobox_currentIndexChanged(int)));

  // Receive controls
  connect(ui->receive_carrier_freq_spinbox, SIGNAL(valueChanged(int)), this,
          SLOT(on_receive_carrier_freq_spinbox_valueChanged(int)));
  connect(ui->receive_modulation_scheme_combobox,
          SIGNAL(currentIndexChanged(int)), this,
          SLOT(on_receive_modulation_scheme_combobox_currentIndexChanged(int)));

  // Connect gain controls BEFORE setting initial values
  connect(ui->modulation_gain_dial, SIGNAL(valueChanged(int)), this,
          SLOT(on_modulation_gain_dial_valueChanged(int)));
  connect(ui->modulation_gain_spinbox, SIGNAL(valueChanged(int)), this,
          SLOT(on_modulation_gain_spinbox_valueChanged(int)));

  // NEW: FSK TRANSMIT carrier frequency controls
  connect(ui->f1_tx_fsk_carrier_spinBox, SIGNAL(valueChanged(int)), this,
          SLOT(on_f1_tx_fsk_carrier_spinBox_valueChanged(int)));
  connect(ui->f2_tx_fsk_carrier_spinBox, SIGNAL(valueChanged(int)), this,
          SLOT(on_f2_tx_fsk_carrier_spinBox_valueChanged(int)));
  connect(ui->f3_tx_fsk_carrier_spinBox, SIGNAL(valueChanged(int)), this,
          SLOT(on_f3_tx_fsk_carrier_spinBox_valueChanged(int)));
  connect(ui->f4_tx_fsk_carrier_spinBox, SIGNAL(valueChanged(int)), this,
          SLOT(on_f4_tx_fsk_carrier_spinBox_valueChanged(int)));

  // NEW: FSK RECEIVE carrier frequency controls
  connect(ui->f1_rx_fsk_carrier_spinBox, SIGNAL(valueChanged(int)), this,
          SLOT(on_f1_rx_fsk_carrier_spinBox_valueChanged(int)));
  connect(ui->f2_rx_fsk_carrier_spinBox, SIGNAL(valueChanged(int)), this,
          SLOT(on_f2_rx_fsk_carrier_spinBox_valueChanged(int)));
  connect(ui->f3_rx_fsk_carrier_spinBox, SIGNAL(valueChanged(int)), this,
          SLOT(on_f3_rx_fsk_carrier_spinBox_valueChanged(int)));
  connect(ui->f4_rx_fsk_carrier_spinBox, SIGNAL(valueChanged(int)), this,
          SLOT(on_f4_rx_fsk_carrier_spinBox_valueChanged(int)));

  // Inicializar controladores digitales
  _is_transmitting_digital = false;

  // Timer para transmisión digital FSK
  connect(_digital_tx_timer.get(), &QTimer::timeout, this,
          &MainWindow::on_digital_symbol_timer);

  // Update timer - 30 fps for display
  connect(_timer.get(), SIGNAL(timeout()), this, SLOT(on_update_timer()));
  _timer->start(33); // approximately 30 fps (1000ms/30 = 33.33ms)
}

MainWindow::~MainWindow() {
  _client.stop();
  delete ui;
}

// Play button - start processing
void MainWindow::on_start_modulation_pbutton_clicked() {
  _client.start_processing();

  // Si estamos transmitiendo FSK-4 (o recibiendo en loopback), iniciar el timer
  if ((_client.get_mode() == dsp_client::Mode::Transmit &&
       _client.get_transmit_modulation() ==
           dsp_client::ModulationScheme::FSK_4) ||
      (_client.get_mode() == dsp_client::Mode::Receive &&
       _client.get_receive_modulation() ==
           dsp_client::ModulationScheme::FSK_4)) {

    // Digital transmission removed
    _is_transmitting_digital = true;
    // Calcular intervalo: samples_per_symbol / sample_rate * 1000 ms
    // Por ejemplo: 1024 samples / 48000 Hz = 21.33 ms
    int interval_ms = (1024 * 1000) / 48000;
    _digital_tx_timer->start(interval_ms);

    std::cout << "[MainWindow] Iniciando transmisión FSK-4, intervalo: "
              << interval_ms << " ms" << std::endl;
    std::cout << "  TX Ready: " << (_digital_link.tx_ready() ? "YES" : "NO")
              << "\n";
    std::cout << "  Total Symbols: " << _digital_link.get_tx_symbol_count()
              << "\n";
  }
}

// Si estamos en TX y la modulación seleccionada es 4-FSK, arrancar TX digital
void MainWindow::on_stop_modulation_pbutton_clicked() {
  /*
  if (_tx_fsk_timer) {
    _tx_fsk_timer->stop();
  }
  */
  // _tx_controller.reset();

  _client.stop_processing();
  _client.stop_files();

  // AÑADIR: Detener transmisión digital si está activa
  if (_is_transmitting_digital) {
    _digital_tx_timer->stop();
    _is_transmitting_digital = false;
    // _digital// _tx_controller->reset();
    std::cout << "[MainWindow] Transmisión FSK-4 detenida" << std::endl;
  }
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
    _client.set_mode(dsp_client::Mode::Transmit);

    // JACK connections: Automatic loopback for testing
    if (_client.is_connected()) {
      // Connect output to input for loopback (will work after JACK is ready)
      _client.connect_ports(jack_ports::APP_OUTPUT_L, jack_ports::APP_INPUT_L);
      _client.connect_ports(jack_ports::APP_OUTPUT_R, jack_ports::APP_INPUT_R);

      // Also connect to speakers
      _client.connect_ports(jack_ports::APP_OUTPUT_L,
                            jack_ports::HEADSET_PLAYBACK_L);
      _client.connect_ports(jack_ports::APP_OUTPUT_R,
                            jack_ports::HEADSET_PLAYBACK_R);

      std::cout << "[JACK] TX mode: Loopback + speakers connected" << std::endl;
    }

    // Si es FSK-4, preparar transmisin digital
    if (_client.get_transmit_modulation() ==
        dsp_client::ModulationScheme::FSK_4) {
      if (!_wav_buffer.empty()) {
        // Usar el WAV que ya fue cargado
        // _digital// _tx_controller->prepare_wav_payload(_wav_buffer.data(),
        // _wav_buffer.size());
        std::cout << "[MainWindow] Modo TX activado con WAV de "
                  << _wav_buffer.size() << " muestras" << std::endl;
      } else {
        // Solo si no hay WAV cargado, generar datos de prueba
        std::cout << "[ADVERTENCIA] No hay WAV cargado, usando datos de prueba"
                  << std::endl;
        // _digital// _tx_controller->prepare_test_payload();
      }
    }

    std::cout << "[MainWindow] Modo Transmit activado" << std::endl;
  }
}

// Receive radio button
void MainWindow::on_receivedButton_toggled(bool checked) {
  if (checked) {
    _client.set_mode(dsp_client::Mode::Receive);

    // JACK connections: Use external jack_connect (internal doesn't work for
    // loopback)
    if (_client.is_connected()) {
      // Use system() to run jack_connect from external process
      // This is necessary because jack_connect() from inside the client doesn't
      // work for loopback
      std::system("jack_connect dsp1:output dsp1:input 2>/dev/null");
      std::system("jack_connect dsp1:output 'RC30-026902, Gaming Headset [Nari "
                  "Essential, Wireless, Receiver] Analog Stereo:playback_FL' "
                  "2>/dev/null");
      std::system("jack_connect dsp1:output 'RC30-026902, Gaming Headset [Nari "
                  "Essential, Wireless, Receiver] Analog Stereo:playback_FR' "
                  "2>/dev/null");

      std::cout << "[JACK] RX mode: External loopback connections established"
                << std::endl;
    }

    // Si es FSK-4, resetear el controlador de recepción
    if (_client.get_receive_modulation() ==
        dsp_client::ModulationScheme::FSK_4) {
      _client.reset_rx_controller();

      // Asegurarse de que los detectores estén inicializados
      _client.init_fsk4();

      std::cout << "[MainWindow] Receptor FSK-4 inicializado y listo"
                << std::endl;
    }

    std::cout << "[MainWindow] Modo Receive activado" << std::endl;
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
  bool is_fsk_mode = (modulation_index == 4); // FSK is index 4
  bool is_ssb_mode = (modulation_index < 4);  // SSB modes are 0-3

  // Show/hide carrier frequency for SSB (not used in FSK)
  ui->label_2->setVisible(is_ssb_mode);
  ui->transmit_carrier_freq_spinbox->setVisible(is_ssb_mode);

  // Show/hide FSK carrier label and spinboxes
  ui->label_6->setVisible(is_fsk_mode); // "4-FSK Carriers" label
  ui->f1_tx_fsk_carrier_spinBox->setVisible(is_fsk_mode);
  ui->f2_tx_fsk_carrier_spinBox->setVisible(is_fsk_mode);
  ui->f3_tx_fsk_carrier_spinBox->setVisible(is_fsk_mode);
  ui->f4_tx_fsk_carrier_spinBox->setVisible(is_fsk_mode);
}

void MainWindow::update_rx_ui_visibility(int modulation_index) {
  bool is_fsk_mode = (modulation_index == 4); // FSK is index 4
  bool is_ssb_mode = (modulation_index < 4);  // SSB modes are 0-3

  // Show/hide carrier frequency for SSB (not used in FSK)
  ui->label_3->setVisible(is_ssb_mode);
  ui->receive_carrier_freq_spinbox->setVisible(is_ssb_mode);

  // Show/hide FSK carrier label and spinboxes
  ui->label_7->setVisible(is_fsk_mode); // "4-FSK Carriers" label
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

void MainWindow::on_transmit_modulation_scheme_combobox_currentIndexChanged(
    int index) {
  // FIRST: Update UI visibility
  update_tx_ui_visibility(index);

  // THEN: Set modulation scheme in DSP client
  dsp_client::ModulationScheme scheme;
  switch (index) {
  case 0:
    scheme = dsp_client::ModulationScheme::SSB_USB;
    break;
  case 1:
    scheme = dsp_client::ModulationScheme::SSB_LSB;
    break;
  case 2:
    scheme = dsp_client::ModulationScheme::SSB_USB_SC;
    break;
  case 3:
    scheme = dsp_client::ModulationScheme::SSB_LSB_SC;
    break;
  case 4:
    scheme = dsp_client::ModulationScheme::FSK_4;
    break;
  default:
    scheme = dsp_client::ModulationScheme::SSB_USB;
  }
  _client.set_transmit_modulation(scheme);
}

void MainWindow::on_receive_modulation_scheme_combobox_currentIndexChanged(
    int index) {
  // FIRST: Update UI visibility
  update_rx_ui_visibility(index);

  // THEN: Set modulation scheme in DSP client
  dsp_client::ModulationScheme scheme;
  switch (index) {
  case 0:
    scheme = dsp_client::ModulationScheme::SSB_USB;
    break;
  case 1:
    scheme = dsp_client::ModulationScheme::SSB_LSB;
    break;
  case 2:
    scheme = dsp_client::ModulationScheme::SSB_USB_SC;
    break;
  case 3:
    scheme = dsp_client::ModulationScheme::SSB_LSB_SC;
    break;
  case 4:
    scheme = dsp_client::ModulationScheme::FSK_4;
    break;
  default:
    scheme = dsp_client::ModulationScheme::SSB_USB;
  }
  _client.set_receive_modulation(scheme);
}

// File selection
bool MainWindow::add_file(const std::filesystem::path &file) {
  ui->fileEdit->setText(file.c_str());
  return _client.add_file(file);
}

void MainWindow::on_fileButton_clicked() {
  QString fileName = QFileDialog::getOpenFileName(
      this, tr("Open Audio File"), "",
      tr("Audio Files (*.wav *.flac *.ogg);;All Files (*)"));

  if (!fileName.isEmpty()) {
    ui->fileEdit->setText(fileName);
    _selectedFiles = QStringList() << fileName;

    // Debug prints for modulation state
    std::cout << "[MainWindow] Checking modulation for file load:\n";
    std::cout << "  TX Mod: " << (int)_client.get_transmit_modulation()
              << " (FSK4=" << (int)dsp_client::ModulationScheme::FSK_4 << ")\n";
    std::cout << "  RX Mod: " << (int)_client.get_receive_modulation()
              << " (FSK4=" << (int)dsp_client::ModulationScheme::FSK_4 << ")\n";

    // Si estamos en modo FSK-4, cargar el WAV para transmisión digital
    if (_client.get_transmit_modulation() ==
            dsp_client::ModulationScheme::FSK_4 ||
        _client.get_receive_modulation() ==
            dsp_client::ModulationScheme::FSK_4) {

      // CORREGIDO: Usar digital_link en lugar de wav_buffer
      if (load_wav_to_digital_link(fileName)) {
        QMessageBox::information(this, "FSK-4 Digital Mode",
                                 QString("WAV cargado y preparado:\n"
                                         "%1 símbolos totales\n"
                                         "Listo para transmisión digital FSK-4")
                                     .arg(_digital_link.get_tx_symbol_count()));

        std::cout << "[MainWindow] WAV procesado para transmisión digital"
                  << std::endl;
      } else {
        QMessageBox::warning(this, "Error", "No se pudo cargar el archivo WAV");
      }
    } else {
      // Modo normal: añadir archivo a la cola de reproducción
      std::filesystem::path file(fileName.toStdString());
      if (_client.add_file(file)) {
        std::cout << "File added to playlist: " << fileName.toStdString()
                  << std::endl;
      }
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
  _client.set_volume(float(value) / 100.0f);
}

void MainWindow::on_volumeSpin_valueChanged(int value) {
  ui->volumeDial->setValue(value);
  _client.set_volume(float(value) / 100.0f);
}

// Update timer - runs at ~30 fps
void MainWindow::on_update_timer() {
  float p = std::sqrt(_client.power());
  ui->powerBar->setValue(static_cast<int>(p * 100));

  QVector<double> vals(_times.size());
  // Update status bar
  if (_client.get_mode() == dsp_client::Mode::Receive) {
    QString status_msg = QString("Rx: %1 Hz | SNR: %2 dB | Bits: %3")
                             .arg(_client.get_fsk4_data().strongest_index)
                             .arg(10.0, 0, 'f', 1)
                             .arg(_symbols_received);
    ui->statusbar->showMessage(status_msg);
    process_rx_symbols();
  } else {
    ui->statusbar->showMessage("Ready");
  }
}

bool MainWindow::load_wav_for_digital_tx(const QString &filename) {
  SF_INFO info;
  info.format = 0;

  SNDFILE *file = sf_open(filename.toStdString().c_str(), SFM_READ, &info);
  if (!file) {
    std::cout << "[MainWindow] Error abriendo archivo WAV: "
              << filename.toStdString() << std::endl;
    return false;
  }

  // Leer todas las muestras del archivo
  std::vector<float> samples(info.frames * info.channels);
  sf_count_t frames_read = sf_readf_float(file, samples.data(), info.frames);
  sf_close(file);

  if (frames_read != info.frames) {
    std::cout << "[MainWindow] Error leyendo archivo WAV" << std::endl;
    return false;
  }

  // Convertir a mono si es necesario
  _wav_buffer.clear();
  if (info.channels == 1) {
    _wav_buffer = samples;
  } else {
    // Promediar canales para obtener mono
    _wav_buffer.resize(info.frames);
    for (sf_count_t i = 0; i < info.frames; ++i) {
      float sum = 0.0f;
      for (int ch = 0; ch < info.channels; ++ch) {
        sum += samples[i * info.channels + ch];
      }
      _wav_buffer[i] = sum / info.channels;
    }
  }

  std::cout << "[MainWindow] WAV cargado: " << info.frames << " frames, "
            << info.channels << " canales, " << info.samplerate << " Hz"
            << std::endl;

  return true;
}

// Added for debugg purpouses 2025.11.20

void MainWindow::on_transmit_digital_button_clicked() {
  // Agregar lógica de transmisión digital
  qDebug() << "Transmit digital button clicked";
}

void MainWindow::on_receive_digital_button_clicked() {
  // Agregar lógica de recepción digital
  qDebug() << "Receive digital button clicked";
}

// ============================================================================
// Digital Link Integration
// ============================================================================

bool MainWindow::load_wav_to_digital_link(const QString &filename) {
  if (!_digital_link.load_wav(filename.toStdString())) {
    std::cerr << "[MainWindow] Failed to load WAV into digital link\n";
    return false;
  }

  if (!_digital_link.prepare_tx_payload()) {
    std::cerr << "[MainWindow] Failed to prepare TX payload\n";
    return false;
  }

  _loaded_wav_path = filename;
  std::cout << "[MainWindow] WAV loaded and framed: " << filename.toStdString()
            << "\n";
  std::cout << "  Total symbols (with preamble/header/CRC): "
            << _digital_link.get_tx_symbol_count() << "\n";

  return true;
}

void MainWindow::start_digital_transmission() {
  /*
  if (!_digital_link.tx_ready()) {
    std::cerr << "[MainWindow] No payload ready for transmission\n";
    return;

  }
*/
  _digital_link.reset_tx();

  if (!_tx_fsk_timer) {
    _tx_fsk_timer = std::make_unique<QTimer>(this);
    connect(_tx_fsk_timer.get(), &QTimer::timeout, this,
            &MainWindow::on_digital_symbol_timer);
  }

  _tx_fsk_timer->start(20);
  std::cout << "[MainWindow] Started digital transmission\n";
}

void MainWindow::stop_digital_transmission() {
  /*
  if (_tx_fsk_timer) {
    _tx_fsk_timer->stop();
  }
  std::cout << "[MainWindow] Stopped digital transmission\n";
  */
}

/* Old on_digital_symbol_timer version

void MainWindow::on_digital_symbol_timer() {
  // Transmit symbol if in transmit mode
  if (_client.get_mode() == dsp_client::Mode::Transmit) {
    uint8_t symbol = _digital_link.next_tx_symbol();
    if (symbol != 0xFF) {
      _client.set_fsk_symbol(symbol);

      // Update progress - alternativa si no hay get_tx_progress()
      static int tx_counter = 0;
      tx_counter++;
      if (tx_counter % 100 == 0) {
        std::cout << "[TX] Transmitting symbol " << tx_counter << std::endl;
      }
    } else {
      // Transmission complete
      std::cout << "[TX] Transmission completed" << std::endl;
      _digital_tx_timer->stop();
      _tx_symbol_counter = 0; // Reset counter
    }
  }

  // RX Integration - Process received symbols and play audio
  if (_client.get_mode() == dsp_client::Mode::Receive) {
    _symbols_received++;

    // Check if frame is complete
    if (_digital_link.frame_complete()) {
      std::cout << "[MainWindow] Frame received! Saving to output.wav\n";

      // Save reconstructed WAV
      QString output_path = "/tmp/received_audio.wav";
      if (_digital_link.save_received_wav(output_path.toStdString())) {
        std::cout << "[MainWindow] Audio saved to: "
                  << output_path.toStdString() << "\n";
        std::cout << "  Sample rate: " << _digital_link.get_sample_rate()
                  << "\n";
        std::cout << "  Symbols received: " << _symbols_received << "\n";
      }

      // Reset for next frame
      _digital_link.reset_rx();
      _symbols_received = 0;
    }
  }
}

*/

// Removed RX section
void MainWindow::on_digital_symbol_timer() {
  static int tx_counter = 0;

  if (_digital_link.tx_ready()) {
    int symbol = _digital_link.next_tx_symbol();

    if (symbol >= 0) {
      _client.set_fsk_symbol(symbol);

      if (tx_counter % 10 == 0) {
        std::cout << "[TX] Symbol " << tx_counter << ": " << symbol
                  << std::endl;
      }
      tx_counter++;
    } else {
      // Transmission complete
      std::cout << "[TX] Transmission completed" << std::endl;
      if (_digital_tx_timer) {
        _digital_tx_timer->stop();
      }
      tx_counter = 0;
      _tx_symbol_counter = 0;
      _is_transmitting_digital = false;
    }
  }
}

// Improved RX Integration with Auto-Playback

void MainWindow::process_rx_symbols() {
  // Get all pending symbols from the queue
  std::vector<uint8_t> symbols = _client.get_rx_symbols();

  if (symbols.empty()) {
    return;
  }

  // Process all symbols
  for (uint8_t symbol : symbols) {
    // Feed symbol to digital link
    _digital_link.process_rx_symbol(symbol);
    _symbols_received++;
  }

  // Check if frame is complete
  if (_digital_link.frame_complete()) {
    std::cout << "\n========================================\n";
    std::cout << "[MainWindow] ✓ FRAME RECEIVED!\n";
    std::cout << "========================================\n";

    // Save reconstructed WAV
    QString output_path = "/tmp/received_audio.wav";
    if (_digital_link.save_received_wav(output_path.toStdString())) {
      std::cout << "  Sample rate: " << _digital_link.get_sample_rate()
                << " Hz\n";
      std::cout << "  Channels: " << _digital_link.get_channels() << "\n";
      std::cout << "  Symbols received: " << _symbols_received << "\n";
      std::cout << "  Saved to: " << output_path.toStdString() << "\n";

      // Automatically play through JACK
      if (add_file(output_path.toStdString())) {
        std::cout << "  ♪ Playing audio through speakers...\n";

        // Switch to Passthrough mode to hear the playback
        // (In Passthrough, 'in' comes from the file because of add_file, and is
        // copied to 'out')
        _client.set_mode(dsp_client::Mode::Passthrough);

        // Update UI to reflect mode change (optional but good for consistency)
        // We might want to uncheck the Receive button or update status
        std::cout << "[MainWindow] Switched to Passthrough mode for playback\n";
      } else {
        std::cerr << "  Failed to queue audio for playback\n";
      }
      std::cout << "========================================\n\n";

      // Reset for next frame
      _digital_link.reset_rx();
      _symbols_received = 0;
    }
  }
}
