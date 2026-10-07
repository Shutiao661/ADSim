#include "adsim/common/Profiler.h"

#include "adsim/common/TextTable.h"

#include <algorithm>
#include <iomanip>
#include <sstream>

namespace adsim {

Profiler& Profiler::instance() {
  static Profiler profiler;
  return profiler;
}

void Profiler::record(const std::string& name, double milliseconds) {
  if (!enabled_) return;

  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& item : data_) {
    if (item.first == name) {
      Accumulator& acc = item.second;
      ++acc.count;
      acc.total_ms += milliseconds;
      acc.min_ms = std::min(acc.min_ms, milliseconds);
      acc.max_ms = std::max(acc.max_ms, milliseconds);
      return;
    }
  }

  Accumulator acc;
  acc.count = 1;
  acc.total_ms = milliseconds;
  acc.min_ms = milliseconds;
  acc.max_ms = milliseconds;
  data_.emplace_back(name, acc);
}

std::vector<Profiler::Entry> Profiler::entries() const {
  std::lock_guard<std::mutex> lock(mutex_);

  double grand_total = 0.0;
  for (const auto& item : data_) {
    grand_total += item.second.total_ms;
  }

  std::vector<Entry> result;
  result.reserve(data_.size());
  for (const auto& item : data_) {
    const Accumulator& acc = item.second;
    Entry e;
    e.name = item.first;
    e.count = acc.count;
    e.total_ms = acc.total_ms;
    e.mean_ms = acc.count == 0 ? 0.0 : acc.total_ms / static_cast<double>(acc.count);
    e.min_ms = acc.count == 0 ? 0.0 : acc.min_ms;
    e.max_ms = acc.max_ms;
    e.percentage = grand_total > 0.0 ? acc.total_ms / grand_total * 100.0 : 0.0;
    result.push_back(std::move(e));
  }

  std::sort(result.begin(), result.end(),
            [](const Entry& a, const Entry& b) { return a.total_ms > b.total_ms; });
  return result;
}

std::string Profiler::report() const {
  const std::vector<Entry> items = entries();

  std::ostringstream oss;
  oss.setf(std::ios::fixed);

  // 列宽用显示宽度手工补齐：setw 按字节计宽，中英混排会错位
  constexpr std::size_t kNameWidth = 26;
  constexpr std::size_t kCountWidth = 10;
  constexpr std::size_t kValueWidth = 13;
  constexpr std::size_t kPercentWidth = 9;

  oss << "\n";
  oss << "======================== 性能剖析报告 ========================\n";
  oss << padTo("阶段", kNameWidth, true) << padTo("次数", kCountWidth, false)
      << padTo("总耗时(ms)", kValueWidth, false) << padTo("均值(ms)", kValueWidth, false)
      << padTo("最大(ms)", kValueWidth, false) << padTo("占比", kPercentWidth, false) << "\n";
  oss << "--------------------------------------------------------------\n";

  double total = 0.0;
  for (const Entry& e : items) {
    total += e.total_ms;

    // 百分比单独格式化：直接截字符串在 100.0% 这类值上会截错
    std::ostringstream percent;
    percent.setf(std::ios::fixed);
    percent.precision(1);
    percent << e.percentage << "%";

    oss << padTo(e.name, kNameWidth, true)
        << padTo(std::to_string(e.count), kCountWidth, false)
        << padTo(e.total_ms, 2, kValueWidth, false)
        << padTo(e.mean_ms, 2, kValueWidth, false)
        << padTo(e.max_ms, 2, kValueWidth, false)
        << padTo(percent.str(), kPercentWidth, false) << "\n";
  }

  oss << "--------------------------------------------------------------\n";
  oss << padTo("合计", kNameWidth, true) << padTo("", kCountWidth, false)
      << padTo(total, 2, kValueWidth, false) << "\n";
  oss << "==============================================================\n";
  return oss.str();
}

void Profiler::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  data_.clear();
}

}  // namespace adsim
