/**
 * main.cpp
 *
 * Copyright (C) 2023  Pablo Alvarado
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

/** @file main.cpp
 *
 * @brief This is a C++ version of a simple jack client, to serve as
 * a simple framework to test basic digital signal processing algorithms,
 * applied to audio.
 */

#include <cstdlib>

#include <iostream>
#include <stdexcept>
#include <filesystem>
#include <vector>

#include <csignal>

#include <boost/program_options.hpp>

#include "mainwindow.h"

#include <QtWidgets/QApplication>

namespace po=boost::program_options;

void signal_handler(int signal) {
  if (signal == SIGINT) {
    std::cout << "Ctrl-C caught, cleaning up and exiting" << std::endl;

    // Let RAII do the clean-up
    exit(EXIT_SUCCESS);
  }
}

int main (int argc, char *argv[])
{
  std::signal(SIGINT,signal_handler);

  try {

  
    QApplication a(argc, argv);

    po::options_description desc("Allowed options");

    desc.add_options()
      ("help,h","show usage information")
      ("files,f",
       po::value<std::vector<std::filesystem::path> >()->multitoken(),
       "List of audio files to be played");

    po::variables_map vm;
    po::store(po::parse_command_line(argc,argv,desc),vm);

    if (vm.count("help")) {
      std::cout << desc << std::endl;
      return EXIT_SUCCESS;
    }

    MainWindow w;
    
    if (vm.count("files")) {
      const std::vector< std::filesystem::path >&
        audio_files = vm["files"].as< std::vector<std::filesystem::path> >();
    
      for (const auto& f : audio_files) {
        bool ok =w.add_file(f);
        std::cout << "Adding file '" << f.c_str() << "' "
                  << (ok ? "succedded" : "failed") << std::endl;
      }
    }

    w.show();
    return a.exec();
  }
  catch (std::exception& exc) {
    std::cout << argv[0] << ": Error: " << exc.what() << std::endl;
    exit(EXIT_FAILURE);
  }

  exit(EXIT_SUCCESS);
}
