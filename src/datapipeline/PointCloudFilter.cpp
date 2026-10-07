#include "adsim/datapipeline/PointCloudFilter.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <random>
#include <sstream>
#include <unordered_map>

namespace adsim {

namespace {

/// 二维整数栅格坐标
struct CellKey {
  std::int32_t x{0};
  std::int32_t y{0};

  bool operator==(const CellKey& o) const { return x == o.x && y == o.y; }
};

struct CellKeyHash {
  std::size_t operator()(const CellKey& key) const noexcept {
    // 两个 32 位整数混合为 64 位再散列，避免相邻坐标聚集
    const std::uint64_t mixed =
        (static_cast<std::uint64_t>(static_cast<std::uint32_t>(key.x)) << 32) ^
        static_cast<std::uint32_t>(key.y);
    std::uint64_t h = mixed * 0x9E3779B97F4A7C15ull;
    h ^= h >> 33;
    h *= 0xFF51AFD7ED558CCDull;
    h ^= h >> 33;
    return static_cast<std::size_t>(h);
  }
};

inline CellKey cellOf(float x, float y, double cell_size) {
  return CellKey{static_cast<std::int32_t>(std::floor(x / cell_size)),
                 static_cast<std::int32_t>(std::floor(y / cell_size))};
}

inline double squaredDistance(const LidarPoint& a, const LidarPoint& b) {
  const double dx = static_cast<double>(a.x) - static_cast<double>(b.x);
  const double dy = static_cast<double>(a.y) - static_cast<double>(b.y);
  const double dz = static_cast<double>(a.z) - static_cast<double>(b.z);
  return dx * dx + dy * dy + dz * dz;
}

/// 均匀网格空间索引，用于 k 近邻查询
class SpatialGrid {
 public:
  SpatialGrid(const std::vector<LidarPoint>& points, double cell_size)
      : points_(points), cell_size_(cell_size > 0.0 ? cell_size : 1.0) {
    cells_.reserve(points.size() * 2);
    for (std::size_t i = 0; i < points.size(); ++i) {
      cells_[cellOf(points[i].x, points[i].y, cell_size_)].push_back(i);
    }
  }

  std::size_t cellCount() const { return cells_.size(); }

  /// 查询距 query 最近的 k 个点的索引（结果按距离升序）。
  ///
  /// 采用环形扩展搜索：访问完第 r 环后，所有未访问的单元到 query 的距离
  /// 至少为 r × cell_size，因此当第 k 近的距离不超过该下界时即可安全终止，
  /// 结果与暴力搜索完全一致。
  void knn(const LidarPoint& query, int k, std::vector<std::size_t>& out) const {
    out.clear();
    if (k <= 0 || points_.empty()) return;

    const CellKey center = cellOf(query.x, query.y, cell_size_);
    // 防御性上限：稀疏点周围邻域不足时避免长时间扩环，
    // 调用方需自行处理"邻居数不足 k"的情况
    const int max_ring = 16;

    for (int ring = 0; ring <= max_ring; ++ring) {
      // 收集本环内的候选点
      for (int dx = -ring; dx <= ring; ++dx) {
        for (int dy = -ring; dy <= ring; ++dy) {
          // 仅处理环的边界，内部单元在之前的环中已处理
          if (ring > 0 && std::abs(dx) != ring && std::abs(dy) != ring) continue;

          const CellKey key{center.x + dx, center.y + dy};
          const auto it = cells_.find(key);
          if (it == cells_.end()) continue;

          for (std::size_t index : it->second) {
            out.push_back(index);
          }
        }
      }

      if (static_cast<int>(out.size()) >= k) {
        // 按距离部分排序，取前 k 个
        std::partial_sort(out.begin(), out.begin() + k, out.end(),
                          [this, &query](std::size_t a, std::size_t b) {
                            return squaredDistance(points_[a], query) <
                                   squaredDistance(points_[b], query);
                          });
        out.resize(static_cast<std::size_t>(k));

        // 第 k 近的距离若小于下一环的下界，则不可能再有更近的点
        const double kth_distance = squaredDistance(points_[out.back()], query);
        const double next_ring_lower_bound =
            static_cast<double>(ring) * cell_size_;  // 保守下界
        if (kth_distance <= next_ring_lower_bound * next_ring_lower_bound) {
          return;
        }
        // 否则继续扩环，把剩余候选也纳入考虑
        continue;
      }
    }

    // 达到环数上限仍未凑够 k 个：返回已有的全部候选
    std::sort(out.begin(), out.end(), [this, &query](std::size_t a, std::size_t b) {
      return squaredDistance(points_[a], query) < squaredDistance(points_[b], query);
    });
  }

 private:
  const std::vector<LidarPoint>& points_;
  double cell_size_;
  std::unordered_map<CellKey, std::vector<std::size_t>, CellKeyHash> cells_;
};

}  // namespace

// ---------------------------------------------------------------------------
// 构造
// ---------------------------------------------------------------------------

PointCloudFilter::PointCloudFilter() = default;
PointCloudFilter::PointCloudFilter(const Config& config) : config_(config) {}

// ---------------------------------------------------------------------------
// 各处理环节
// ---------------------------------------------------------------------------

std::vector<LidarPoint> PointCloudFilter::cropByRange(const std::vector<LidarPoint>& points,
                                                      double min_range, double max_range,
                                                      double min_z, double max_z) {
  std::vector<LidarPoint> out;
  out.reserve(points.size());

  const double min_sq = min_range * min_range;
  const double max_sq = max_range * max_range;

  for (const LidarPoint& p : points) {
    const double range_sq = static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y;
    if (range_sq < min_sq || range_sq > max_sq) continue;
    if (p.z < min_z || p.z > max_z) continue;
    out.push_back(p);
  }
  return out;
}

std::vector<LidarPoint> PointCloudFilter::removeEgoBox(const std::vector<LidarPoint>& points,
                                                       double length, double width) {
  const double half_length = length * 0.5;
  const double half_width = width * 0.5;

  std::vector<LidarPoint> out;
  out.reserve(points.size());

  for (const LidarPoint& p : points) {
    // 自车位于原点，车头朝 +x
    if (std::abs(p.x) <= half_length && std::abs(p.y) <= half_width) continue;
    out.push_back(p);
  }
  return out;
}

std::vector<LidarPoint> PointCloudFilter::voxelDownsample(const std::vector<LidarPoint>& points,
                                                          double voxel_size,
                                                          std::size_t* cell_count) {
  if (voxel_size <= 0.0 || points.empty()) {
    if (cell_count != nullptr) *cell_count = 0;
    return points;
  }

  // 累加每个体素内所有点的坐标，最后取质心——相比取第一个点，
  // 质心对噪声更鲁棒，且不会因点序不同产生不同结果
  struct Accumulator {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double intensity{0.0};
    std::size_t count{0};
  };

  std::unordered_map<CellKey, Accumulator, CellKeyHash> voxels;
  voxels.reserve(points.size() / 2 + 1);

  for (const LidarPoint& p : points) {
    Accumulator& acc = voxels[cellOf(p.x, p.y, voxel_size)];
    acc.x += p.x;
    acc.y += p.y;
    acc.z += p.z;
    acc.intensity += p.intensity;
    ++acc.count;
  }

  std::vector<LidarPoint> out;
  out.reserve(voxels.size());
  for (const auto& entry : voxels) {
    const Accumulator& acc = entry.second;
    if (acc.count == 0) continue;
    const double inv = 1.0 / static_cast<double>(acc.count);
    LidarPoint p;
    p.x = static_cast<float>(acc.x * inv);
    p.y = static_cast<float>(acc.y * inv);
    p.z = static_cast<float>(acc.z * inv);
    p.intensity = static_cast<float>(acc.intensity * inv);
    out.push_back(p);
  }

  if (cell_count != nullptr) *cell_count = voxels.size();
  return out;
}

std::vector<LidarPoint> PointCloudFilter::statisticalOutlierRemoval(
    const std::vector<LidarPoint>& points, int k, double std_ratio) {
  if (points.size() <= static_cast<std::size_t>(k) || k <= 0) {
    return points;
  }

  // 网格边长取平均点间距量级，使每个单元平均含少量点，邻域查询效率最高
  BoundingBox2 box;
  for (const LidarPoint& p : points) box.expand({p.x, p.y});
  const double area = std::max((box.max.x - box.min.x) * (box.max.y - box.min.y), 1e-6);
  const double cell_size = std::sqrt(area / static_cast<double>(points.size())) * 2.0;

  const SpatialGrid grid(points, cell_size > 1e-3 ? cell_size : 1e-3);

  std::vector<double> mean_distances(points.size(), 0.0);
  std::vector<bool> sparse(points.size(), false);
  std::vector<std::size_t> neighbors;

  for (std::size_t i = 0; i < points.size(); ++i) {
    grid.knn(points[i], k + 1, neighbors);  // +1 是因为结果中含自身

    double sum = 0.0;
    int counted = 0;
    for (std::size_t index : neighbors) {
      if (index == i) continue;
      sum += std::sqrt(squaredDistance(points[index], points[i]));
      ++counted;
    }

    if (counted >= k) {
      mean_distances[i] = sum / static_cast<double>(counted);
    } else {
      // 邻域内凑不满 k 个邻居，说明该点本就孤立。
      // 不能拿实际邻居数取平均——孤立点的平均距离反而会偏小而被误留；
      // 也不能直接置为极大值——那会污染下面的全局均值与标准差。
      // 正确做法是把这类点排除在统计之外，并直接判为离群点。
      sparse[i] = true;
    }
  }

  // 仅用邻域充分的点估计分布，孤立点不参与
  double sum = 0.0;
  std::size_t valid_count = 0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (!sparse[i]) {
      sum += mean_distances[i];
      ++valid_count;
    }
  }
  if (valid_count == 0) {
    return points;  // 全部孤立，无从判定，保持原样
  }
  const double mean = sum / static_cast<double>(valid_count);

  double variance = 0.0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (!sparse[i]) {
      variance += (mean_distances[i] - mean) * (mean_distances[i] - mean);
    }
  }
  variance /= static_cast<double>(valid_count);
  const double stddev = std::sqrt(variance);

  const double threshold = mean + std_ratio * stddev;

  std::vector<LidarPoint> out;
  out.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (!sparse[i] && mean_distances[i] <= threshold) {
      out.push_back(points[i]);
    }
  }
  return out;
}

void PointCloudFilter::separateGround(const std::vector<LidarPoint>& points,
                                      int iterations,
                                      double distance_threshold,
                                      double slope_limit,
                                      std::vector<LidarPoint>& ground,
                                      std::vector<LidarPoint>& non_ground) {
  ground.clear();
  non_ground.clear();

  if (points.size() < 3) {
    non_ground = points;
    return;
  }

  // ---- RANSAC 拟合平面 z = ax + by + c ----
  std::mt19937 rng(20240924u);  // 固定种子保证结果可复现
  std::uniform_int_distribution<std::size_t> dist(0, points.size() - 1);

  double best_a = 0.0, best_b = 0.0, best_c = 0.0;
  std::size_t best_inliers = 0;
  const double slope_tan = std::tan(slope_limit);

  for (int iter = 0; iter < iterations; ++iter) {
    const LidarPoint& p1 = points[dist(rng)];
    const LidarPoint& p2 = points[dist(rng)];
    const LidarPoint& p3 = points[dist(rng)];

    // 解 3×3 线性方程组求平面系数
    const double x1 = p1.x, y1 = p1.y, z1 = p1.z;
    const double x2 = p2.x, y2 = p2.y, z2 = p2.z;
    const double x3 = p3.x, y3 = p3.y, z3 = p3.z;

    const double det = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1);
    if (std::abs(det) < 1e-9) continue;  // 三点共线

    const double a = ((z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)) / det;
    const double b = ((z3 - z1) * (x2 - x1) - (z2 - z1) * (x3 - x1)) / det;
    const double c = z1 - a * x1 - b * y1;

    // 坡度约束：平面法向 (a,b,-1) 与竖直方向夹角过大则不是地面
    if (std::sqrt(a * a + b * b) > slope_tan) continue;

    std::size_t inliers = 0;
    const double norm = std::sqrt(a * a + b * b + 1.0);
    for (const LidarPoint& p : points) {
      const double distance = std::abs(a * p.x + b * p.y - p.z + c) / norm;
      if (distance <= distance_threshold) ++inliers;
    }

    if (inliers > best_inliers) {
      best_inliers = inliers;
      best_a = a;
      best_b = b;
      best_c = c;
    }
  }

  if (best_inliers == 0) {
    // 未找到可靠平面，全部视为非地面
    non_ground = points;
    return;
  }

  const double norm = std::sqrt(best_a * best_a + best_b * best_b + 1.0);
  for (const LidarPoint& p : points) {
    const double distance =
        std::abs(best_a * p.x + best_b * p.y - p.z + best_c) / norm;
    if (distance <= distance_threshold) {
      ground.push_back(p);
    } else {
      non_ground.push_back(p);
    }
  }
}

// ---------------------------------------------------------------------------
// 完整处理链
// ---------------------------------------------------------------------------

std::string PointCloudFilter::Statistics::toString() const {
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(2);

  oss << "点云处理统计:";
  oss.precision(0);
  oss << " 输入 " << input_points;
  oss << " → 距离裁剪 " << after_range_crop;
  oss << " → 车体剔除 " << after_ego_removal;
  oss << " → 体素降采样 " << after_voxel;
  oss << " → 离群点去除 " << after_outlier;
  if (ground_points > 0) oss << " (地面点 " << ground_points << ")";
  oss << " → 输出 " << output_points;
  oss.precision(1);
  oss << "  压缩率 " << reductionRatio() * 100.0 << "%";
  oss.precision(0);
  oss << "  网格单元 " << grid_cells;
  return oss.str();
}

LidarFrame PointCloudFilter::filter(const LidarFrame& input, Statistics* statistics) const {
  Statistics stats;
  stats.input_points = input.points.size();

  std::vector<LidarPoint> points = input.points;

  // 1. 距离与高度裁剪
  points = cropByRange(points, config_.min_range, config_.max_range, config_.min_z,
                       config_.max_z);
  stats.after_range_crop = points.size();

  // 2. 自车车体剔除
  if (config_.remove_ego_box) {
    points = removeEgoBox(points, config_.ego_length, config_.ego_width);
  }
  stats.after_ego_removal = points.size();

  // 3. 体素降采样
  if (config_.enable_voxel) {
    points = voxelDownsample(points, config_.voxel_size, &stats.grid_cells);
  }
  stats.after_voxel = points.size();

  // 4. 统计离群点去除
  if (config_.enable_statistical_outlier) {
    points = statisticalOutlierRemoval(points, config_.outlier_k, config_.outlier_std_ratio);
  }
  stats.after_outlier = points.size();

  // 5. 地面分离
  if (config_.enable_ground_removal) {
    std::vector<LidarPoint> ground;
    std::vector<LidarPoint> non_ground;
    separateGround(points, config_.ransac_iterations, config_.ground_distance_threshold,
                   config_.ground_slope_limit, ground, non_ground);
    stats.ground_points = ground.size();
    points = std::move(non_ground);
  }

  stats.output_points = points.size();
  if (statistics != nullptr) *statistics = stats;

  LidarFrame output;
  output.stamp = input.stamp;
  output.frame_id = input.frame_id;
  output.points = std::move(points);
  return output;
}

}  // namespace adsim
