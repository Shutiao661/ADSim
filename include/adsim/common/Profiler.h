// =============================================================================
//  Profiler.h — 运行期性能剖析
//
//  用于量化"数据管道各阶段耗时占比"，支撑简历中"100G 数据处理 30min → 8min"
//  这类优化结论的可复现验证。
//
//  用法：
//      {
//        ADSIM_PROFILE_SCOPE("lidar_denoise");
//        denoise(frame);
//      }
//      std::cout << Profiler::instance().report();
// =============================================================================
#pragma once

#include <chrono>
#include <cstddef>
#include <mutex>
#include <string>
#include <vector>

namespace adsim {

class Profiler {
 public:
  struct Entry {
    std::string name;
    std::size_t count{0};
    double total_ms{0.0};
    double mean_ms{0.0};
    double min_ms{0.0};
    double max_ms{0.0};
    double percentage{0.0};  ///< 占总耗时的比例
  };

  static Profiler& instance();

  void record(const std::string& name, double milliseconds);

  std::vector<Entry> entries() const;

  /// 按总耗时降序生成可读报告
  std::string report() const;

  void reset();

  /// 全局开关，关闭后 record 立即返回（用于性能对比测试）
  void setEnabled(bool enabled) { enabled_ = enabled; }
  bool enabled() const { return enabled_; }

 private:
  Profiler() = default;

  struct Accumulator {
    std::size_t count{0};
    double total_ms{0.0};
    double min_ms{1e300};
    double max_ms{0.0};
  };

  mutable std::mutex mutex_;
  std::vector<std::pair<std::string, Accumulator>> data_;
  bool enabled_{true};
};

// ---------------------------------------------------------------------------
// ScopedTimer — RAII 计时器
// ---------------------------------------------------------------------------
class ScopedTimer {
 public:
  explicit ScopedTimer(std::string name) : name_(std::move(name)) {
    if (Profiler::instance().enabled()) {
      start_ = std::chrono::steady_clock::now();
      active_ = true;
    }
  }

  ~ScopedTimer() { stop(); }

  ScopedTimer(const ScopedTimer&) = delete;
  ScopedTimer& operator=(const ScopedTimer&) = delete;

  /// 提前结束计时并记录；重复调用只生效一次
  void stop() {
    if (!active_) return;
    const auto end = std::chrono::steady_clock::now();
    const double ms =
        std::chrono::duration<double, std::milli>(end - start_).count();
    Profiler::instance().record(name_, ms);
    active_ = false;
  }

 private:
  std::string name_;
  std::chrono::steady_clock::time_point start_;
  bool active_{false};
};

/// 累加型计时器：适合"启动/停止"跨越多段代码的场景
class LapTimer {
 public:
  void start() { start_ = std::chrono::steady_clock::now(); }

  /// 返回自 start 以来的毫秒数并累加
  double lap() {
    const auto now = std::chrono::steady_clock::now();
    const double ms = std::chrono::duration<double, std::milli>(now - start_).count();
    start_ = now;
    total_ms_ += ms;
    ++laps_;
    return ms;
  }

  double totalMs() const { return total_ms_; }
  std::size_t laps() const { return laps_; }
  void reset() { total_ms_ = 0.0; laps_ = 0; }

 private:
  std::chrono::steady_clock::time_point start_{};
  double total_ms_{0.0};
  std::size_t laps_{0};
};

}  // namespace adsim

#define ADSIM_PROFILE_CONCAT_IMPL(a, b) a##b
#define ADSIM_PROFILE_CONCAT(a, b) ADSIM_PROFILE_CONCAT_IMPL(a, b)

/// 作用域计时，离开作用域自动记录
#define ADSIM_PROFILE_SCOPE(name)                                        \
  ::adsim::ScopedTimer ADSIM_PROFILE_CONCAT(_adsim_timer_, __LINE__)(name)

/// 带唯一后缀的计时器变量名，便于同一作用域内多次使用
#define ADSIM_PROFILE_SCOPE_UNIQUE(name) ADSIM_PROFILE_SCOPE(name)
