// =============================================================================
//  CarlaBridge.h — 与 CARLA 0.9.x 仿真引擎的可选桥接层
//
//  定位：本项目默认用轻量的二维车道图（World）做行为决策回归，吞吐量比真实
//  引擎高数个量级；但"感知-决策-控制"闭环最终必须在真实物理引擎里验证一次。
//  本桥接层负责两个方向的数据搬运：
//
//      仿真内核(World/VehicleState/LidarFrame)  <-->  CARLA(OpenDRIVE/Transform/传感器)
//
//  设计要点：
//   1) 可选依赖：默认不参与构建（CMake 里列在 ADSIM_OPTIONAL_SOURCES 中），
//      只有 -DADSIM_WITH_CARLA=ON 且 ADSIM_CARLA_ROOT 存在时才编译，
//      并定义 ADSIM_HAS_CARLA。这样没有 CARLA 的机器上也能构建、跑测试。
//   2) 未接入时的降级：所有接口仍然存在且可链接，只是返回失败并给出原因，
//      调用方（如 CLI 的 --carla 选项）据此打印提示而不是崩溃。
//   3) 真实 API 调用全部包在 `#if defined(ADSIM_HAS_CARLA)` 内，非 CARLA 分支
//      提供等价的桩实现；对外接口在两种配置下**签名一致**（CarlaLocation /
//      CarlaTransform 在接入时是 carla::geom 类型的别名，未接入时是同构 POD），
//      因此调用方代码无需条件编译。
//   4) 纯数据转换（坐标/位姿/点云/车道图重建）不依赖 CARLA 运行时，
//      以 inline 形式放在头文件中，可被单元测试直接覆盖。
//
//  坐标系约定（易错点，单独说明）：
//    * CARLA 使用左手系：x 向前、y 向右、z 向上，yaw 绕 z 轴、右手正方向为
//      顺时针（从上方看）。
//    * 本项目使用右手系：x 向前、y 向左、theta 逆时针为正。
//    * 因此转换只有一条规则：y_adsim = -y_carla，theta_adsim = -yaw_carla；
//      z / roll / pitch 原样保留（本项目是平面模型，不使用这两个自由度）。
// =============================================================================
#pragma once

#include "adsim/common/Types.h"
#include "adsim/sim/Scenario.h"
#include "adsim/sim/SimEngine.h"
#include "adsim/sim/VehicleModel.h"
#include "adsim/sim/World.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#if defined(ADSIM_HAS_CARLA)
#include <carla/geom/Location.h>
#include <carla/geom/Transform.h>
namespace adsim {
using CarlaLocation = carla::geom::Location;
using CarlaTransform = carla::geom::Transform;
}  // namespace adsim
#else
namespace adsim {
/// 未接入 CARLA 时的同构 POD 占位（与 carla::geom::Location 字段一致）
struct CarlaLocation {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};
/// 未接入 CARLA 时的同构 POD 占位（仅保留本项目用得到的字段）
struct CarlaTransform {
  CarlaLocation location;
  double roll{0.0};   ///< 度（CARLA 的 geom::Rotation 亦为度）
  double pitch{0.0};  ///< 度
  double yaw{0.0};    ///< 度
};
}  // namespace adsim
#endif

namespace adsim {

/// 一帧回放数据：路测/数据管道清洗后的结果，用于"数据回灌仿真"
struct ReplayFrame {
  Timestamp stamp{kInvalidTimestamp};
  VehicleState ego;                    ///< 记录的自车位姿与状态
  bool has_ego{false};
  std::vector<RoadObject> objects;     ///< 同一时刻的其他交通参与者
  LidarFrame lidar;                    ///< 同一时刻的点云（可为空）
};

class CarlaBridge {
 public:
  /// 连接与仿真步进参数
  struct Config {
    std::string host{"127.0.0.1"};
    std::uint16_t port{2000};
    double timeout_seconds{10.0};       ///< RPC 超时，弱网下适当放大
    std::string ego_blueprint{"vehicle.tesla.model3"};
    std::string object_blueprint{"vehicle.tesla.model3"};  ///< 其他车辆物体用
    double fixed_delta_seconds{0.05};   ///< 与 SimEngine::Config::step_size 对齐
    bool synchronous_mode{true};        ///< 同步步进：与内核定步长一一对应
    /// CARLA 车辆的最大前轮转角 (rad)：用于把本模型的前轮转角归一化到
    /// CARLA 的 steer ∈ [-1,1]。不同 blueprint 的取值不同，接入实车时需标定。
    double max_steer_angle{1.22};
  };

  /// 激光雷达传感器参数（字段名与 CARLA 的传感器属性一一对应）
  struct LidarConfig {
    double range{80.0};                 ///< m
    int channels{32};
    double points_per_second{600000.0};
    double rotation_frequency{20.0};    ///< Hz
    double upper_fov{10.0};             ///< 度
    double lower_fov{-25.0};            ///< 度
    double sensor_height{1.8};          ///< 安装高度（相对车体）(m)
    std::string channel{"lidar"};       ///< 写入 LidarFrame::frame_id
  };

  using LidarCallback = std::function<void(const LidarFrame&)>;

  /// 车道图重建的输入：一条车道的采样点序列（按行驶方向排列）
  struct WaypointSample {
    Vec2 position;
    double heading{0.0};      ///< rad
    double lane_width{3.5};   ///< m
    int lane_id{-1};          ///< CARLA 的车道号（符号表示左右半幅）
    int road_id{-1};
    bool is_junction{false};
    double speed_limit{13.9}; ///< m/s
  };

  /// 一条车道的采样序列
  struct LaneSamples {
    int road_id{-1};
    int lane_id{-1};
    bool is_junction{false};
    double lane_width{3.5};
    double speed_limit{13.9};
    std::vector<WaypointSample> samples;  ///< 按行驶方向排列，至少两个点
  };

  CarlaBridge();
  explicit CarlaBridge(const Config& config);
  ~CarlaBridge();

  // 桥接对象持有 CARLA 客户端与传感器句柄，禁止拷贝
  CarlaBridge(const CarlaBridge&) = delete;
  CarlaBridge& operator=(const CarlaBridge&) = delete;

  // -------------------------------------------------------------------------
  // 可用性（编译期决定，任何时候都可安全调用）
  // -------------------------------------------------------------------------

  /// 是否在构建时接入了 CARLA
  static bool isAvailable();

  /// 未接入时的原因说明，供 CLI/日志打印
  static std::string unavailableReason();

  // -------------------------------------------------------------------------
  // 连接管理
  // -------------------------------------------------------------------------

  /// 连接 CARLA 服务端；失败时通过 error 返回原因（未接入 CARLA 时必然失败）
  bool connect(std::string* error = nullptr);
  void disconnect();
  bool isConnected() const;

  /// 服务端版本号（未连接时返回空串）
  std::string serverVersion() const;

  /// 切换到同步模式并按 fixed_delta_seconds 步进
  bool applySynchronousSettings(std::string* error = nullptr) const;

  /// 推进一个仿真步（同步模式下由客户端驱动服务端）
  bool tick(std::string* error = nullptr) const;

  // -------------------------------------------------------------------------
  // 坐标 / 位姿转换（纯函数，两种构建配置下行为一致，可单测）
  // -------------------------------------------------------------------------

  /// CARLA Location → 本项目 Vec2（y 取反、丢弃 z）
  static Vec2 locationToVec2(const CarlaLocation& location);

  /// 本项目 Vec2 → CARLA Location（y 取反，z 由调用方指定）
  static CarlaLocation vec2ToLocation(const Vec2& point, double z = 0.0);

  /// CARLA Transform → 本项目 Pose2（y 取反、yaw 取反并转弧度）
  static Pose2 transformToPose2(const CarlaTransform& transform);

  /// 本项目 Pose2 → CARLA Transform（y 取反、theta 取反并转角度）
  static CarlaTransform pose2ToTransform(const Pose2& pose, double z = 0.0);

  /// CARLA 的激光雷达点云 → LidarFrame（同样做 y 取反，intensity 原样保留）
  static LidarFrame lidarToFrame(std::vector<LidarPoint> points, Timestamp stamp,
                                 const std::string& frame_id = "lidar");

  // -------------------------------------------------------------------------
  // OpenDRIVE 地图 → 车道图
  // -------------------------------------------------------------------------

  /// 由采样到的车道序列重建车道图（不依赖 CARLA，可离线单测）。
  /// 拓扑的恢复规则：
  ///   * 同一条路上横向相邻的车道：中心线中点沿左右法向平移一个车道宽后
  ///     能落在对方中心线附近（容差 0.5m）即互指 left/right；
  ///   * 跨路段：本车道起点/终点与另一条车道端点距离小于 1m 且航向大致
  ///     一致（夹角 < 60°）时互指 predecessor/successor。
  static World buildLaneGraph(const std::vector<LaneSamples>& runs, double sample_step = 2.0);

  /// 加载地图并把 OpenDRIVE 车道网络转换为 World（需要先 connect）
  bool buildWorld(const std::string& map_name, World* world, std::string* error = nullptr,
                  double sample_step = 2.0);

  // -------------------------------------------------------------------------
  // 传感器
  // -------------------------------------------------------------------------

  /// 在自车（或指定 RC 车模）上挂载激光雷达，回调在每帧数据到达时被调用
  bool attachLidar(const LidarConfig& config, LidarCallback callback,
                   std::string* error = nullptr);
  void detachSensors();

  // -------------------------------------------------------------------------
  // 车辆控制与状态同步
  // -------------------------------------------------------------------------

  /// 生成自车（已连接时）；pose 为本项目坐标系下的初始位姿
  bool spawnEgo(const Pose2& pose, std::string* error = nullptr);

  /// 向自车下发归一化控制量（与 IPolicy::Command 对应）
  bool applyControl(double throttle, double brake, double steering,
                    std::string* error = nullptr);

  /// 把仿真内核的状态写回 CARLA（数字回放/孪生同步用）
  bool setEgoState(const VehicleState& state, std::string* error = nullptr);

  /// 把世界里的其他交通参与者同步到 CARLA：按 id 复用已生成的演员，
  /// 位置/朝向逐帧刷新（数量较少时开销可接受）
  bool syncObjects(const World& world, std::string* error = nullptr);

  // -------------------------------------------------------------------------
  // 数字回放
  // -------------------------------------------------------------------------

  /// 用记录的数据驱动仿真内核：把记录的自车位姿作为跟踪目标，
  /// 由内核重算轨迹与安全指标（TTC/间距/碰撞），实现"数据回灌仿真"。
  /// 该接口不需要 CARLA 运行时，因此在两种构建配置下都可用。
  static SimulationResult replay(const std::vector<ReplayFrame>& frames, const World& world,
                                 const SimEngine::Config& config,
                                 std::string* error = nullptr);

  /// 在 CARLA 里复现记录的数据：按 playback_speed 倍速回放自车与物体位姿
  /// （需要先 connect 并 spawnEgo）。返回实际回放的帧数。
  std::size_t replayInCarla(const std::vector<ReplayFrame>& frames, double playback_speed,
                            std::string* error = nullptr);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};


/// 数字回放策略：按记录的自车位姿跟踪，供 CarlaBridge::replay 使用。
/// 独立导出是为了让"回灌数据 + 自研决策算法"的对比实验也能复用同一套跟踪逻辑。
class ReplayTrackingPolicy : public IPolicy {
 public:
  /// @param frames 记录的数据（按时间递增，至少两帧）
  /// @param lookahead 前视距离 (m)
  /// @param fallback_dt 记录时间戳无效时使用的帧间隔 (s)
  explicit ReplayTrackingPolicy(std::vector<ReplayFrame> frames, double lookahead = 8.0,
                                double fallback_dt = 0.05);

  Command computeCommand(const VehicleState& ego, const World& world, double dt) override;
  std::string name() const override { return "ReplayTracking"; }
  void reset() override;

  /// 当前跟踪的时刻（相对首帧，单位 s）
  double replayTime() const { return time_; }

  /// 目标位姿与目标速度查询（供外部复用，例如对比实验里换用自研跟踪器）
  bool targetAt(double t, Pose2* pose, double* speed) const;

 private:
  std::vector<ReplayFrame> frames_;
  std::vector<double> times_;  ///< 与 frames_ 一一对应的相对时间轴 (s)
  double lookahead_;
  double time_{0.0};
};

}  // namespace adsim
