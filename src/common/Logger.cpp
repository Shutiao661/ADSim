#include "adsim/common/Logger.h"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <thread>

namespace adsim {

const char* toString(LogLevel level) {
  switch (level) {
    case LogLevel::kTrace: return "TRACE";
    case LogLevel::kDebug: return "DEBUG";
    case LogLevel::kInfo:  return "INFO ";
    case LogLevel::kWarn:  return "WARN ";
    case LogLevel::kError: return "ERROR";
    case LogLevel::kFatal: return "FATAL";
  }
  return "?????";
}

namespace {

const char* levelColor(LogLevel level) {
  switch (level) {
    case LogLevel::kTrace: return "\033[90m";  // 灰
    case LogLevel::kDebug: return "\033[36m";  // 青
    case LogLevel::kInfo:  return "\033[32m";  // 绿
    case LogLevel::kWarn:  return "\033[33m";  // 黄
    case LogLevel::kError: return "\033[31m";  // 红
    case LogLevel::kFatal: return "\033[41m";  // 红底
  }
  return "\033[0m";
}

constexpr const char* kColorReset = "\033[0m";

std::string currentTimestamp() {
  using namespace std::chrono;
  const auto now = system_clock::now();
  const auto secs = time_point_cast<seconds>(now);
  const auto ms = duration_cast<milliseconds>(now - secs).count();
  const std::time_t t = system_clock::to_time_t(now);

  std::tm tm_buf{};
#if defined(_WIN32)
  localtime_s(&tm_buf, &t);
#else
  localtime_r(&t, &tm_buf);
#endif

  char buf[64];
  std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d.%03d",
                tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
                tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec,
                static_cast<int>(ms));
  return buf;
}

/// 将线程 id 压缩为可读的短标识
std::string shortThreadId() {
  const std::size_t id = std::hash<std::thread::id>{}(std::this_thread::get_id());
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%04zx", id & 0xFFFF);
  return buf;
}

}  // namespace

Logger& Logger::instance() {
  static Logger logger;
  return logger;
}

void Logger::setLevel(LogLevel level) {
  std::lock_guard<std::mutex> lock(mutex_);
  level_ = level;
}

void Logger::setLogFile(const std::string& path) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (file_.is_open()) {
    file_.close();
  }
  if (!path.empty()) {
    file_.open(path, std::ios::out | std::ios::app);
    if (!file_.is_open()) {
      std::cerr << "[ADSIM] 无法打开日志文件: " << path << std::endl;
    }
  }
}

void Logger::log(LogLevel level, const char* file, int line, const std::string& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (level < level_) return;

  const std::string ts = currentTimestamp();
  const std::string tid = shortThreadId();
  const char* fname = detail::basename(file);

  if (console_enabled_) {
    if (color_enabled_) {
      std::cout << levelColor(level) << '[' << ts << "][" << toString(level) << "][t"
                << tid << "] " << message << "  (" << fname << ':' << line << ')'
                << kColorReset << std::endl;
    } else {
      std::cout << '[' << ts << "][" << toString(level) << "][t" << tid << "] "
                << message << "  (" << fname << ':' << line << ')' << std::endl;
    }
  }

  if (file_.is_open()) {
    file_ << '[' << ts << "][" << toString(level) << "][t" << tid << "] "
          << message << "  (" << fname << ':' << line << ')' << '\n';
    file_.flush();
  }

  if (level == LogLevel::kFatal) {
    std::abort();
  }
}

namespace detail {

const char* basename(const char* path) {
  if (path == nullptr) return "";
  const char* last = path;
  for (const char* p = path; *p != '\0'; ++p) {
    if (*p == '/' || *p == '\\') last = p + 1;
  }
  return last;
}

}  // namespace detail

}  // namespace adsim
