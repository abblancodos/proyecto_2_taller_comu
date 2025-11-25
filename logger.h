#pragma once
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
namespace logging {
class Logger {
public:
  enum class Level {
    DEBUG,   // Todo el spam va aquí
    INFO,    // Mensajes importantes
    WARNING, // Advertencias
    ERROR    // Errores críticos
  };
  static Logger &instance() {
    static Logger logger;
    return logger;
  }
  void init(const std::string &log_dir = "../logs") {
    // Crear directorio si no existe
    std::filesystem::create_directories(log_dir);
    // Limpiar archivos antiguos
    for (const auto &entry : std::filesystem::directory_iterator(log_dir)) {
      std::filesystem::remove(entry.path());
    }
    // Abrir archivo de log
    std::string log_file = log_dir + "/debug.log";
    _file.open(log_file, std::ios::out | std::ios::trunc);
    if (_file.is_open()) {
      log(Level::INFO, "Logger initialized: " + log_file);
    }
  }
  void log(Level level, const std::string &message) {
    std::string timestamp = get_timestamp();
    std::string level_str = level_to_string(level);
    std::string formatted =
        "[" + timestamp + "] [" + level_str + "] " + message;
    // Siempre escribir al archivo
    if (_file.is_open()) {
      _file << formatted << std::endl;
      _file.flush();
    }
    // Solo mostrar en consola si es INFO, WARNING o ERROR
    if (level >= Level::INFO) {
      std::cout << formatted << std::endl;
    }
  }
  ~Logger() {
    if (_file.is_open()) {
      _file.close();
    }
  }
private:
  Logger() = default;
  std::ofstream _file;
  std::string get_timestamp() {
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%H:%M:%S");
    return oss.str();
  }
  std::string level_to_string(Level level) {
    switch (level) {
    case Level::DEBUG:
      return "DEBUG";
    case Level::INFO:
      return "INFO ";
    case Level::WARNING:
      return "WARN ";
    case Level::ERROR:
      return "ERROR";
    default:
      return "?????";
    }
  }
};
// Macros convenientes
#define LOG_DEBUG(msg)                                                         \
  logging::Logger::instance().log(logging::Logger::Level::DEBUG, msg)
#define LOG_INFO(msg)                                                          \
  logging::Logger::instance().log(logging::Logger::Level::INFO, msg)
#define LOG_WARN(msg)                                                          \
  logging::Logger::instance().log(logging::Logger::Level::WARNING, msg)
#define LOG_ERROR(msg)                                                         \
  logging::Logger::instance().log(logging::Logger::Level::ERROR, msg)
} // namespace logging
