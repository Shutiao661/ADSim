// =============================================================================
//  Logger.h — 轻量线程安全日志
//
//  支持级别过滤、彩色终端输出、文件落盘。供数据管道与仿真内核记录运行状态。
// =============================================================================
#pragma once

#include <fstream>
#include <mutex>
#include <sstream>
#include <string>

namespace adsim {

enum class LogLevel { kTrace = 0, kDebug = 1, kInfo = 2, kWarn = 3, kError = 4, kFatal = 5 };

const char* toString(LogLevel level);

class Logger {
 public:
  static Logger& instance();

  void setLevel(LogLevel level);
  LogLevel level() const { return level_; }

  /// 同时输出到文件；传空字符串则关闭文件输出
  void setLogFile(const std::string& path);

  /// 是否输出到标准输出
  void setConsoleEnabled(bool enabled) { console_enabled_ = enabled; }

  /// 是否输出颜色（终端环境下默认开启）
  void setColorEnabled(bool enabled) { color_enabled_ = enabled; }

  void log(LogLevel level, const char* file, int line, const std::string& message);

  bool shouldLog(LogLevel level) const { return level >= level_; }

 private:
  Logger() = default;

  LogLevel level_{LogLevel::kInfo};
  bool console_enabled_{true};
  bool color_enabled_{true};
  std::mutex mutex_;
  std::ofstream file_;
};

namespace detail {
/// 取文件路径中的文件名部分，避免日志中出现冗长路径
const char* basename(const char* path);

/// 将任意参数流式拼接为字符串
inline void appendToStream(std::ostringstream&) {}

template <typename First, typename... Rest>
void appendToStream(std::ostringstream& oss, First&& first, Rest&&... rest) {
  oss << std::forward<First>(first);
  appendToStream(oss, std::forward<Rest>(rest)...);
}
}  // namespace detail

}  // namespace adsim

// ---------------------------------------------------------------------------
// 日志宏
// ---------------------------------------------------------------------------
#define ADSIM_LOG_IMPL(level, ...)                                            \
  do {                                                                        \
    ::adsim::Logger& _logger = ::adsim::Logger::instance();                    \
    if (_logger.shouldLog(level)) {                                            \
      std::ostringstream _oss;                                                 \
      ::adsim::detail::appendToStream(_oss, __VA_ARGS__);                      \
      _logger.log(level, __FILE__, __LINE__, _oss.str());                      \
    }                                                                          \
  } while (false)

#define ADSIM_LOG_TRACE(...) ADSIM_LOG_IMPL(::adsim::LogLevel::kTrace, __VA_ARGS__)
#define ADSIM_LOG_DEBUG(...) ADSIM_LOG_IMPL(::adsim::LogLevel::kDebug, __VA_ARGS__)
#define ADSIM_LOG_INFO(...)  ADSIM_LOG_IMPL(::adsim::LogLevel::kInfo, __VA_ARGS__)
#define ADSIM_LOG_WARN(...)  ADSIM_LOG_IMPL(::adsim::LogLevel::kWarn, __VA_ARGS__)
#define ADSIM_LOG_ERROR(...) ADSIM_LOG_IMPL(::adsim::LogLevel::kError, __VA_ARGS__)
#define ADSIM_LOG_FATAL(...) ADSIM_LOG_IMPL(::adsim::LogLevel::kFatal, __VA_ARGS__)
