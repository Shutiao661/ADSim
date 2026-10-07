#include "adsim/common/Types.h"

#include <algorithm>
#include <numeric>
#include <sstream>

namespace adsim {

std::string Statistics::toString() const {
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(4);
  oss << "n=" << count << " min=" << min << " max=" << max << " mean=" << mean
      << " std=" << stddev;
  return oss.str();
}

Statistics computeStatistics(const std::vector<double>& values) {
  Statistics s;
  s.count = values.size();
  if (values.empty()) return s;

  const auto [lo, hi] = std::minmax_element(values.begin(), values.end());
  s.min = *lo;
  s.max = *hi;

  const double sum = std::accumulate(values.begin(), values.end(), 0.0);
  s.mean = sum / static_cast<double>(values.size());

  // 使用两遍法计算标准差，数值稳定性优于 sum(x^2) - mean^2
  double accum = 0.0;
  for (double v : values) {
    const double d = v - s.mean;
    accum += d * d;
  }
  s.stddev = std::sqrt(accum / static_cast<double>(values.size()));
  return s;
}

}  // namespace adsim
