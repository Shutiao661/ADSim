// =============================================================================
//  PointCloudFilter.h — 激光雷达点云降噪与抽稀
//
//  实车点云含大量噪声：雨雾/扬尘产生的孤立点、车身自身反射、地面点、以及
//  远超感知范围的无用点。这些点会显著拖慢后续感知与规划，必须先做清洗。
//
//  处理链（按顺序）：
//      距离裁剪 → 自车车体剔除 → 体素降采样 → 统计离群点去除 → 地面分离
//
//  统计离群点去除依赖 k 近邻搜索，采用均匀网格做空间索引，
//  把朴素 O(n²) 的邻域查询降到接近 O(n)。
// =============================================================================
#pragma once

#include "adsim/common/Types.h"

#include <cstddef>
#include <string>
#include <vector>

namespace adsim {

class PointCloudFilter {
 public:
  struct Config {
    // ---- 距离裁剪 ----
    double min_range{1.5};   ///< 小于该距离的点多半是车身反射
    double max_range{80.0};
    double min_z{-3.0};
    double max_z{3.0};

    // ---- 自车车体剔除 ----
    bool remove_ego_box{true};
    double ego_length{5.0};
    double ego_width{2.6};

    // ---- 体素降采样 ----
    bool enable_voxel{true};
    double voxel_size{0.2};

    // ---- 统计离群点去除 ----
    bool enable_statistical_outlier{true};
    int outlier_k{10};            ///< 近邻个数
    double outlier_std_ratio{1.0};  ///< 阈值 = 均值 + ratio × 标准差

    // ---- 地面分离 ----
    bool enable_ground_removal{false};
    int ransac_iterations{100};
    double ground_distance_threshold{0.15};  ///< 点到拟合平面的距离阈值
    double ground_slope_limit{0.3};          ///< 平面法向与竖直方向夹角上限（弧度）
  };

  struct Statistics {
    std::size_t input_points{0};
    std::size_t after_range_crop{0};
    std::size_t after_ego_removal{0};
    std::size_t after_voxel{0};
    std::size_t after_outlier{0};
    std::size_t ground_points{0};
    std::size_t output_points{0};
    std::size_t grid_cells{0};

    /// 相对输入的压缩比例 [0,1]
    double reductionRatio() const {
      return input_points == 0
                 ? 0.0
                 : 1.0 - static_cast<double>(output_points) / static_cast<double>(input_points);
    }

    std::string toString() const;
  };

  PointCloudFilter();
  explicit PointCloudFilter(const Config& config);

  /// 执行完整处理链；statistics 非空时填充各级统计
  LidarFrame filter(const LidarFrame& input, Statistics* statistics = nullptr) const;

  const Config& config() const { return config_; }
  Config& config() { return config_; }

  // ---- 各环节单独暴露，便于测试与组合 ----

  /// 按距离与高度裁剪
  static std::vector<LidarPoint> cropByRange(const std::vector<LidarPoint>& points,
                                             double min_range, double max_range,
                                             double min_z, double max_z);

  /// 剔除位于自车矩形轮廓内的点（车身反射）
  static std::vector<LidarPoint> removeEgoBox(const std::vector<LidarPoint>& points,
                                              double length, double width);

  /// 体素栅格降采样：每个体素内取质心，保证降采样后点分布均匀
  static std::vector<LidarPoint> voxelDownsample(const std::vector<LidarPoint>& points,
                                                 double voxel_size,
                                                 std::size_t* cell_count = nullptr);

  /// 统计离群点去除：剔除平均近邻距离超过 μ + ratio×σ 的点
  static std::vector<LidarPoint> statisticalOutlierRemoval(
      const std::vector<LidarPoint>& points, int k, double std_ratio);

  /// RANSAC 平面拟合分离地面，输出地面点与非地面点
  static void separateGround(const std::vector<LidarPoint>& points,
                             int iterations,
                             double distance_threshold,
                             double slope_limit,
                             std::vector<LidarPoint>& ground,
                             std::vector<LidarPoint>& non_ground);

 private:
  Config config_;
};

}  // namespace adsim
