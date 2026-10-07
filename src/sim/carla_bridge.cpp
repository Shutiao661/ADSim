// =============================================================================
//  carla_bridge.cpp — CARLA 0.9.x 桥接层实现
//
//  ---------------------------------------------------------------------------
//  为什么需要这一层
//  ---------------------------------------------------------------------------
//  行为决策算法的回归必须能在两种环境下跑同一套代码：
//    * 轻量内核（默认）：World 是二维车道图，单场景毫秒级，用于大批量回归；
//    * CARLA（可选）：真实物理与传感器，用于关键场景的"最后一公里"验证。
//  桥接层只做数据搬运与单位/坐标系转换，不含任何决策逻辑 —— 决策代码通过
//  IPolicy 接口注入，因此切换到 CARLA 时算法本体一行都不用改。
//
//  ---------------------------------------------------------------------------
//  本文件的两条编译路径
//  ---------------------------------------------------------------------------
//  * 定义 ADSIM_HAS_CARLA 时（CMake: -DADSIM_WITH_CARLA=ON）：
//      真实调用 CARLA 客户端 API —— 连接服务端、加载地图、遍历 waypoint 重建
//      车道图、生成激光雷达并接收回调、下发控制、回放位姿。
//  * 未定义时（默认构建）：
//      所有对外接口仍然存在且可链接（便于测试与 CLI 编译），需要 CARLA 的
//      接口返回失败并给出原因；不需要 CARLA 的纯逻辑（坐标转换、点云转换、
//      车道图重建、数据回灌）在两种配置下行为完全一致，因此这部分可以被
//      单元测试直接覆盖 —— 这正是"现场只跑一次"的 CARLA 之外仍然可回归的部分。
//
//  ---------------------------------------------------------------------------
//  坐标系与单位（本项目最容易出错的地方）
//  ---------------------------------------------------------------------------
//    CARLA：左手系，x 前、y 右、z 上；yaw/roll/pitch 单位为**度**。
//    本项目：右手系，x 前、y 左；theta 单位为**弧度**，逆时针为正。
//    转换规则只有一条：y 取反、yaw/theta 取反（角度/弧度互转）。
//    激光雷达点云同样遵循该规则（CARLA 点云 y 向右，转换时取反）。
//
//  ---------------------------------------------------------------------------
//  实现约定与已知限制
//  ---------------------------------------------------------------------------
//   1) 客户端句柄：CARLA 0.9.14 起 LibCarla 的 SharedPtr 即 std::shared_ptr
//      （不再依赖 boost），本文件据此编写；若使用更早版本，只需把
//      std::dynamic_pointer_cast 换成 boost::dynamic_pointer_cast。
//   2) 服务端对象生命周期用类型擦除的 shared_ptr<void> 持有（见 Impl 注释），
//      避免在成员声明里出现随版本变化的智能指针类型。
//   3) 限速：CARLA 把限速放在 landmark 里（类型形如 "Speed_City_50"），
//      解析逻辑集中在 speedLimitFromLandmarks 一处；取不到时回落到默认限速。
//   4) 行人（walker）需要额外的 WalkerController/AI 控制，本层暂不同步，
//      syncObjects 会在 error 里给出被跳过的数量。
//   5) 车道图重建是几何重建（按车道宽做横向邻接、按端点距离做纵向邻接），
//      对环岛/分叉等复杂拓扑会退化为多条独立车道，需要时再扩展。
//   6) 本文件的 CARLA 分支依赖真实头文件，无法在没有 CARLA 的机器上编译校验，
//      因此涉及"随版本可能变化"的 API 都做了局部化（限速解析用模板接收地标
//      容器、传感器句柄用类型擦除、走车道用 GetNext 而非内部结构），
//      把需要适配的改动面收敛到单点。
// =============================================================================
#include "adsim/sim/CarlaBridge.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <thread>
#include <utility>

#if defined(ADSIM_HAS_CARLA)
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/Vehicle.h>
#include <carla/client/Waypoint.h>
#include <carla/client/World.h>
#include <carla/geom/Location.h>
#include <carla/geom/Rotation.h>
#include <carla/geom/Transform.h>
#include <carla/rpc/VehicleControl.h>
#include <carla/rpc/WorldSettings.h>
#include <carla/sensor/data/LidarMeasurement.h>
#endif

namespace adsim {

namespace {

constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

/// 车道图重建参数
constexpr double kLateralTolerance = 0.5;  ///< 横向邻接的匹配容差 (m)
constexpr double kLateralGapFactor = 1.5;  ///< 作为邻道候选的最大中点间距（车道宽倍数）
constexpr double kLinkDistance = 1.0;      ///< 纵向邻接的端点距离容差 (m)
constexpr double kLinkHeading = 60.0 * kDegToRad;  ///< 纵向邻接的航向容差

/// 车道缺省值（取不到 CARLA 属性时使用，与 World 的缺省一致）
constexpr double kDefaultLaneWidth = 3.5;
constexpr double kDefaultSpeedLimit = 13.9;  ///< ≈50 km/h

/// 主循环退出容差：与 SimEngine 内部保持同一量级
constexpr double kReplayTimeEps = 1e-9;

void setError(std::string* error, const std::string& message) {
  if (error != nullptr) *error = message;
}

/// 采样序列 → 中心线：丢弃非有限值与重合点，保证折线严格推进
std::vector<Vec2> toCenterline(const std::vector<CarlaBridge::WaypointSample>& samples) {
  std::vector<Vec2> line;
  line.reserve(samples.size());
  for (const CarlaBridge::WaypointSample& sample : samples) {
    if (!std::isfinite(sample.position.x) || !std::isfinite(sample.position.y)) continue;
    if (!line.empty() && (sample.position - line.back()).norm() < 1e-6) continue;
    line.push_back(sample.position);
  }
  return line;
}

/// 记录序列的时间轴（相对首帧，单位 s）。
/// 时间戳有效时直接相减；全部无效时退化为"帧序 × fallback_dt"，
/// 这样即使是没有时间信息的原始回放数据也能被驱动起来。
std::vector<double> buildTimeline(const std::vector<ReplayFrame>& frames, double fallback_dt) {
  std::vector<double> times;
  times.reserve(frames.size());
  const bool has_valid_stamps = !frames.empty() && frames.front().stamp != kInvalidTimestamp;
  for (std::size_t i = 0; i < frames.size(); ++i) {
    if (has_valid_stamps && frames[i].stamp != kInvalidTimestamp) {
      times.push_back(toSeconds(frames[i].stamp - frames.front().stamp));
    } else {
      times.push_back(static_cast<double>(i) * fallback_dt);
    }
  }
  // 时间轴必须单调不减，否则插值会取到错乱的目标点
  for (std::size_t i = 1; i < times.size(); ++i) {
    if (times[i] < times[i - 1]) times[i] = times[i - 1];
  }
  return times;
}

/// 在时间轴中定位 t 所在区间（返回下标 i 使 times[i] <= t <= times[i+1]）
std::size_t locateInterval(const std::vector<double>& times, double t) {
  if (times.size() < 2) return 0;
  std::size_t index = 0;
  while (index + 2 < times.size() && times[index + 1] <= t) ++index;
  return index;
}

#if defined(ADSIM_HAS_CARLA)
/// 从地标中解析限速：CARLA 把限速作为 Landmark 给出，类型形如
/// "Speed_City_50"/"Speed_50"，数值为 km/h；取不到时沿用 fallback。
/// 用模板接收容器，避免绑定某一版本的 Landmark 具体类型 ——
/// 各版本间唯一的差异点就集中在这个函数里。
template <typename LandmarkList>
double speedLimitFromLandmarks(const LandmarkList& landmarks, double fallback) {
  double limit = fallback;
  for (const auto& landmark : landmarks) {
    const std::string type = landmark.GetType();
    if (type.rfind("Speed_", 0) != 0) continue;
    std::string digits;  // 从尾部取连续数字（"Speed_City_50" → "50"）
    for (std::size_t i = type.size(); i > 0; --i) {
      const char c = type[i - 1];
      if (c < '0' || c > '9') break;
      digits.insert(digits.begin(), c);
    }
    if (digits.empty()) continue;
    const double kmh = std::stod(digits);
    if (kmh > 0.0) limit = kmh / 3.6;  // km/h → m/s
  }
  return limit;
}
#endif  // ADSIM_HAS_CARLA

}  // namespace

// ---------------------------------------------------------------------------
// 可用性
// ---------------------------------------------------------------------------

bool CarlaBridge::isAvailable() {
#if defined(ADSIM_HAS_CARLA)
  return true;
#else
  return false;
#endif
}

std::string CarlaBridge::unavailableReason() {
#if defined(ADSIM_HAS_CARLA)
  return {};
#else
  return "构建时未启用 CARLA：请用 -DADSIM_WITH_CARLA=ON 且 ADSIM_CARLA_ROOT "
         "指向 CARLA 安装根目录后重新配置构建";
#endif
}

// ---------------------------------------------------------------------------
// 坐标 / 位姿 / 点云转换（纯函数，两种配置下行为一致）
// ---------------------------------------------------------------------------

Vec2 CarlaBridge::locationToVec2(const CarlaLocation& location) {
  // CARLA 左手系（y 向右）→ 本项目右手系（y 向左）
  return Vec2{static_cast<double>(location.x), -static_cast<double>(location.y)};
}

CarlaLocation CarlaBridge::vec2ToLocation(const Vec2& point, double z) {
  CarlaLocation location;
  location.x = point.x;
  location.y = -point.y;  // 反向转换同样是 y 取反
  location.z = z;
  return location;
}

Pose2 CarlaBridge::transformToPose2(const CarlaTransform& transform) {
  // yaw 在 CARLA 中为度、从上方看顺时针为正；本项目为弧度、逆时针为正
  const Vec2 position = locationToVec2(transform.location);
  return Pose2(position.x, position.y, -transform.yaw * kDegToRad);
}

CarlaTransform CarlaBridge::pose2ToTransform(const Pose2& pose, double z) {
  CarlaTransform transform;
  transform.location = vec2ToLocation(pose.position(), z);
  transform.roll = 0.0;
  transform.pitch = 0.0;
  transform.yaw = -pose.theta * kRadToDeg;
  return transform;
}

LidarFrame CarlaBridge::lidarToFrame(std::vector<LidarPoint> points, Timestamp stamp,
                                     const std::string& frame_id) {
  LidarFrame frame;
  frame.stamp = stamp;
  frame.frame_id = frame_id;
  frame.points.reserve(points.size());
  for (const LidarPoint& point : points) {
    LidarPoint converted = point;
    converted.y = -point.y;  // 同样按"y 取反"的约定，保证横向符号与 World 一致
    frame.points.push_back(converted);
  }
  return frame;
}

// ---------------------------------------------------------------------------
// OpenDRIVE 车道网络 → World（不依赖 CARLA 运行时）
// ---------------------------------------------------------------------------

World CarlaBridge::buildLaneGraph(const std::vector<LaneSamples>& runs, double sample_step) {
  (void)sample_step;  // 采样密度在采集阶段确定，这里只做几何重建
  World world;

  // ---- 1) 逐条车道建立中心线 ----
  // World::lanes() 只提供常量访问（车道图构建完成后视为不变），
  // 因此先在本地把拓扑关系全部算好，再一次性交给 world。
  std::vector<Lane> lanes;
  lanes.reserve(runs.size());
  for (const LaneSamples& run : runs) {
    std::vector<Vec2> centerline = toCenterline(run.samples);
    if (centerline.size() < 2) continue;  // 少于两点无法构成车道
    Lane lane;
    lane.id = static_cast<int>(lanes.size());
    lane.centerline = std::move(centerline);
    lane.width = run.lane_width > 0.0 ? run.lane_width : kDefaultLaneWidth;
    lane.speed_limit = run.speed_limit > 0.0 ? run.speed_limit : kDefaultSpeedLimit;
    lane.is_junction = run.is_junction;
    lanes.push_back(std::move(lane));
  }

  // 弧长参数化与关键位姿只算一次，邻接搜索复用
  struct LaneGeometry {
    std::vector<double> arc;
    double length{0.0};
    Vec2 start;
    Vec2 end;
    Vec2 mid;
    double start_heading{0.0};
    double end_heading{0.0};
    double mid_heading{0.0};
  };
  std::vector<LaneGeometry> geometry(lanes.size());
  for (std::size_t i = 0; i < lanes.size(); ++i) {
    LaneGeometry& g = geometry[i];
    const std::vector<Vec2>& line = lanes[i].centerline;
    g.arc = laneArcLength(line);
    g.length = g.arc.back();
    g.start = line.front();
    g.end = line.back();
    g.start_heading = headingAlong(line, g.arc, 0.0);
    g.end_heading = headingAlong(line, g.arc, g.length);
    g.mid = interpolateAlong(line, g.arc, g.length * 0.5);
    g.mid_heading = headingAlong(line, g.arc, g.length * 0.5);
  }

  const std::size_t count = lanes.size();

  // ---- 2) 横向邻接 ----
  // 规则：把本车道中心线的中点沿左/右法向平移一个车道宽，若能落到另一条
  // 车道中心线附近（容差 0.5m），则两者互为左右邻道；多候选取最近的一条。
  // 对向车道同样按下式互指（与 World::intersection 的约定一致）。
  for (std::size_t i = 0; i < count; ++i) {
    if (lanes[i].is_junction) continue;  // 路口车道不在此恢复横向关系
    const Vec2 left_normal{-std::sin(geometry[i].mid_heading),
                           std::cos(geometry[i].mid_heading)};
    const double max_gap = lanes[i].width * kLateralGapFactor;
    double best_left = kLateralTolerance;
    double best_right = kLateralTolerance;
    for (std::size_t j = 0; j < count; ++j) {
      if (i == j || lanes[j].is_junction) continue;
      const Vec2 delta = geometry[j].mid - geometry[i].mid;
      if (delta.norm() > max_gap) continue;  // 隔了隔离带/多条车道，不构成相邻
      // 把邻道中点投影到本车道的左右法向上：接近一个车道宽即为左/右邻道
      const double lateral = delta.dot(left_normal);
      if (std::fabs(lateral - lanes[i].width) < best_left) {
        best_left = std::fabs(lateral - lanes[i].width);
        lanes[i].left_lane_id = lanes[j].id;
      }
      if (std::fabs(lateral + lanes[i].width) < best_right) {
        best_right = std::fabs(lateral + lanes[i].width);
        lanes[i].right_lane_id = lanes[j].id;
      }
    }
  }

  // ---- 3) 纵向邻接（前驱/后继）----
  // 规则：本车道终点与另一条车道起点距离小于 1m，且航向夹角小于 60°。
  // 路口车道同样参与：它正是两段路段之间的连接段。
  for (std::size_t i = 0; i < count; ++i) {
    double best_successor = kLinkDistance;
    double best_predecessor = kLinkDistance;
    for (std::size_t j = 0; j < count; ++j) {
      if (i == j) continue;
      const double d_successor = (geometry[j].start - geometry[i].end).norm();
      if (d_successor < best_successor &&
          std::fabs(normalizeAngle(geometry[j].start_heading - geometry[i].end_heading)) <
              kLinkHeading) {
        best_successor = d_successor;
        lanes[i].successor_id = lanes[j].id;
      }
      const double d_predecessor = (geometry[j].end - geometry[i].start).norm();
      if (d_predecessor < best_predecessor &&
          std::fabs(normalizeAngle(geometry[i].start_heading - geometry[j].end_heading)) <
              kLinkHeading) {
        best_predecessor = d_predecessor;
        lanes[i].predecessor_id = lanes[j].id;
      }
    }
  }

  for (const Lane& lane : lanes) world.addLane(lane);
  return world;
}

// ---------------------------------------------------------------------------
// 连接管理
// ---------------------------------------------------------------------------

struct CarlaBridge::Impl {
  Config config;
  bool connected{false};
  std::string server_version;

#if defined(ADSIM_HAS_CARLA)
  std::unique_ptr<carla::client::Client> client;
  /// 传感器句柄用类型擦除的 shared_ptr<void> 持有：删除器里捕获真实的
  /// 智能指针。这样成员声明不依赖具体版本的智能指针类型，也保证回调
  /// 注册期间传感器对象不会被提前销毁。
  std::shared_ptr<void> sensor_handle;
  std::uint32_t ego_actor_id{0};
  std::map<int, std::uint32_t> object_actor_ids;  ///< RoadObject::id → 演员 id
#endif
};

CarlaBridge::CarlaBridge() : CarlaBridge(Config{}) {}

CarlaBridge::CarlaBridge(const Config& config) : impl_(new Impl()) {
  impl_->config = config;
}

CarlaBridge::~CarlaBridge() {
  // 析构不应抛异常：disconnect 只在失败时返回，不做错误上报
  disconnect();
}

bool CarlaBridge::connect(std::string* error) {
#if defined(ADSIM_HAS_CARLA)
  if (impl_->connected) return true;
  try {
    impl_->client.reset(new carla::client::Client(impl_->config.host, impl_->config.port));
    impl_->client->SetTimeout(std::chrono::milliseconds(
        static_cast<long long>(impl_->config.timeout_seconds * 1e3)));
    // GetServerVersion 会真正发起一次 RPC：用它确认连通性，同时留下版本号便于排查
    impl_->server_version = impl_->client->GetServerVersion();
    impl_->connected = true;
  } catch (const std::exception& e) {
    impl_->client.reset();
    impl_->connected = false;
    impl_->server_version.clear();
    setError(error, std::string("连接 CARLA 失败: ") + e.what());
    return false;
  }
  return applySynchronousSettings(error);
#else
  setError(error, unavailableReason());
  return false;
#endif
}

void CarlaBridge::disconnect() {
#if defined(ADSIM_HAS_CARLA)
  impl_->sensor_handle.reset();
  impl_->object_actor_ids.clear();
  impl_->ego_actor_id = 0;
  impl_->client.reset();
  impl_->server_version.clear();
#endif
  impl_->connected = false;
}

bool CarlaBridge::isConnected() const { return impl_->connected; }

std::string CarlaBridge::serverVersion() const { return impl_->server_version; }

bool CarlaBridge::applySynchronousSettings(std::string* error) const {
#if defined(ADSIM_HAS_CARLA)
  if (!impl_->connected || !impl_->client) {
    setError(error, "未连接 CARLA 服务端");
    return false;
  }
  try {
    auto world = impl_->client->GetWorld();
    carla::rpc::WorldSettings settings = world->GetSettings();
    // 同步 + 固定步长：让服务端的时间推进与本内核的定步长一一对应。
    // 异步模式下"仿真时间"与"墙钟时间"解耦，指标无法与轻量内核对齐。
    settings.synchronous_mode = impl_->config.synchronous_mode;
    settings.fixed_delta_seconds = impl_->config.fixed_delta_seconds;
    world->ApplySettings(settings);
  } catch (const std::exception& e) {
    setError(error, std::string("设置同步模式失败: ") + e.what());
    return false;
  }
  return true;
#else
  setError(error, unavailableReason());
  return false;
#endif
}

bool CarlaBridge::tick(std::string* error) const {
#if defined(ADSIM_HAS_CARLA)
  if (!impl_->connected || !impl_->client) {
    setError(error, "未连接 CARLA 服务端");
    return false;
  }
  try {
    auto world = impl_->client->GetWorld();
    if (impl_->config.synchronous_mode) {
      // 同步模式：由客户端驱动，服务端推进一拍后返回
      world->Tick(std::chrono::milliseconds(
          static_cast<long long>(impl_->config.fixed_delta_seconds * 1e3)));
    } else {
      // 异步模式：等服务端自己推进一拍，避免客户端空转
      world->WaitForTick(std::chrono::seconds(1));
    }
  } catch (const std::exception& e) {
    setError(error, std::string("仿真步进失败: ") + e.what());
    return false;
  }
  return true;
#else
  setError(error, unavailableReason());
  return false;
#endif
}

// ---------------------------------------------------------------------------
// 地图 → World
// ---------------------------------------------------------------------------

bool CarlaBridge::buildWorld(const std::string& map_name, World* world, std::string* error,
                             double sample_step) {
  if (world == nullptr) {
    setError(error, "world 输出指针为空");
    return false;
  }
#if defined(ADSIM_HAS_CARLA)
  if (!impl_->connected || !impl_->client) {
    setError(error, "未连接 CARLA 服务端，无法加载地图");
    return false;
  }
  const double step = sample_step > 0.1 ? sample_step : 2.0;
  try {
    auto world_ptr = impl_->client->GetWorld();
    if (!map_name.empty()) {
      world_ptr = impl_->client->LoadWorld(map_name);  // 地图资源需存在于服务端
    }
    auto map = world_ptr->GetMap();
    if (!map) {
      setError(error, "加载地图失败: " + map_name);
      return false;
    }

    // 按 (road_id, lane_id) 把 waypoint 聚合成车道序列：
    // GenerateWaypoints 给出的是全图离散采样点，而车道图需要"有序的中心线"，
    // 因此对每个尚未采集过的 (road, lane) 沿 GetNext 顺序走一遍。
    std::vector<LaneSamples> runs;
    std::map<std::pair<int, int>, bool> visited;
    for (const auto& waypoint : map->GenerateWaypoints(static_cast<float>(step))) {
      if (!waypoint) continue;
      const int road_id = static_cast<int>(waypoint->GetRoadId());
      const int lane_id = waypoint->GetLaneId();
      const std::pair<int, int> key(road_id, lane_id);
      if (visited[key]) continue;
      visited[key] = true;

      LaneSamples run;
      run.road_id = road_id;
      run.lane_id = lane_id;
      run.is_junction = waypoint->IsJunction();
      run.lane_width =
          waypoint->GetLaneWidth() > 0.0 ? waypoint->GetLaneWidth() : kDefaultLaneWidth;
      run.speed_limit =
          speedLimitFromLandmarks(waypoint->GetLandmarks(50.0), kDefaultSpeedLimit);
      auto current = waypoint;
      for (int guard = 0; guard < 8192 && current; ++guard) {
        const carla::geom::Transform transform = current->GetTransform();
        WaypointSample sample;
        sample.position = locationToVec2(transform.location);
        sample.heading = -transform.rotation.yaw * kDegToRad;  // 度、顺时针 → 弧度、逆时针
        sample.lane_width = run.lane_width;
        sample.lane_id = lane_id;
        sample.road_id = road_id;
        sample.is_junction = run.is_junction;
        sample.speed_limit = run.speed_limit;
        run.samples.push_back(sample);

        const auto next = current->GetNext(step);
        if (next.empty() || !next.front()) break;
        const auto candidate = next.front();
        // 换到别的 road/lane（路口边界、变道口）即停止：跨段连接交给
        // buildLaneGraph 的纵向邻接规则恢复，避免把两条不同车道串成一条。
        if (static_cast<int>(candidate->GetRoadId()) != road_id ||
            candidate->GetLaneId() != lane_id) {
          break;
        }
        current = candidate;
      }
      if (run.samples.size() >= 2) runs.push_back(std::move(run));
    }

    *world = buildLaneGraph(runs, step);
    if (world->laneCount() == 0) {
      setError(error, "地图 " + map_name + " 中未解析出任何车道");
      return false;
    }
  } catch (const std::exception& e) {
    setError(error, std::string("地图转换失败: ") + e.what());
    return false;
  }
  return true;
#else
  (void)map_name;
  (void)sample_step;
  setError(error, unavailableReason());
  return false;
#endif
}

// ---------------------------------------------------------------------------
// 传感器
// ---------------------------------------------------------------------------

bool CarlaBridge::attachLidar(const LidarConfig& config, LidarCallback callback,
                              std::string* error) {
#if defined(ADSIM_HAS_CARLA)
  if (!impl_->connected || !impl_->client) {
    setError(error, "未连接 CARLA 服务端，无法挂载传感器");
    return false;
  }
  if (!callback) {
    setError(error, "激光雷达回调为空");
    return false;
  }
  try {
    auto world = impl_->client->GetWorld();
    auto blueprint = world->GetBlueprintLibrary()->Find("sensor.lidar.ray_cast");
    if (!blueprint) {
      setError(error, "未找到 sensor.lidar.ray_cast 蓝图");
      return false;
    }
    // 属性名与 CARLA 传感器蓝图一一对应；取值以字符串形式传入
    blueprint->SetAttribute("range", std::to_string(config.range));
    blueprint->SetAttribute("channels", std::to_string(config.channels));
    blueprint->SetAttribute("points_per_second", std::to_string(config.points_per_second));
    blueprint->SetAttribute("rotation_frequency", std::to_string(config.rotation_frequency));
    blueprint->SetAttribute("upper_fov", std::to_string(config.upper_fov));
    blueprint->SetAttribute("lower_fov", std::to_string(config.lower_fov));

    auto ego = world->GetActor(impl_->ego_actor_id);
    if (!ego) {
      setError(error, "尚未生成自车，请先调用 spawnEgo");
      return false;
    }
    // 先按自车当前位姿放到车顶，再 AttachTo 让传感器随车体一起运动
    carla::geom::Transform mount = ego->GetTransform();
    mount.location.z = static_cast<float>(config.sensor_height);
    auto sensor = world->SpawnActor<carla::client::Sensor>(*blueprint, mount);
    if (!sensor) {
      setError(error, "生成激光雷达失败");
      return false;
    }
    sensor->AttachTo(ego);

    const std::string frame_id = config.channel;
    const double delta = impl_->config.fixed_delta_seconds;
    sensor->Listen([callback, frame_id, delta](auto data) {
      // CARLA 0.9.14 起 SharedPtr 即 std::shared_ptr；更早版本换成 boost:: 同名函数
      auto lidar = std::dynamic_pointer_cast<carla::sensor::data::LidarMeasurement>(data);
      if (!lidar) return;
      std::vector<LidarPoint> points;
      points.reserve(lidar->size());
      for (const auto& detection : *lidar) {
        LidarPoint point;
        point.x = static_cast<float>(detection.point.x);
        point.y = static_cast<float>(detection.point.y);
        point.z = static_cast<float>(detection.point.z);
        point.intensity = static_cast<float>(detection.intensity);
        points.push_back(point);
      }
      // CARLA 的传感器时间戳是"帧号 + 仿真内已过秒数"，不是 Unix 纳秒。
      // 这里换算成以首帧为原点的单调时间戳，保证与内核的定步长时序对齐；
      // 需要与 rosbag 对齐时由调用方在回调里用真实时钟覆盖 stamp。
      const Timestamp stamp =
          fromSeconds(static_cast<double>(lidar->GetFrameNumber()) * delta);
      callback(lidarToFrame(std::move(points), stamp, frame_id));
    });

    // 类型擦除地持有传感器：删除器捕获真实智能指针，保证回调期间对象存活
    impl_->sensor_handle = std::shared_ptr<void>(
        static_cast<void*>(sensor.get()), [sensor](void*) mutable { (void)sensor; });
  } catch (const std::exception& e) {
    setError(error, std::string("挂载激光雷达失败: ") + e.what());
    return false;
  }
  return true;
#else
  (void)config;
  (void)callback;
  setError(error, unavailableReason());
  return false;
#endif
}

void CarlaBridge::detachSensors() {
#if defined(ADSIM_HAS_CARLA)
  // 释放唯一持有者即可：CARLA 在最后一个引用消失时销毁演员并注销回调
  impl_->sensor_handle.reset();
#endif
}

// ---------------------------------------------------------------------------
// 车辆控制与状态同步
// ---------------------------------------------------------------------------

bool CarlaBridge::spawnEgo(const Pose2& pose, std::string* error) {
#if defined(ADSIM_HAS_CARLA)
  if (!impl_->connected || !impl_->client) {
    setError(error, "未连接 CARLA 服务端");
    return false;
  }
  try {
    auto world = impl_->client->GetWorld();
    auto blueprint = world->GetBlueprintLibrary()->Find(impl_->config.ego_blueprint);
    if (!blueprint) {
      setError(error, "未找到自车蓝图: " + impl_->config.ego_blueprint);
      return false;
    }
    blueprint->SetAttribute("role_name", "ego");
    auto vehicle =
        world->SpawnActor<carla::client::Vehicle>(*blueprint, pose2ToTransform(pose, 0.3));
    if (!vehicle) {
      setError(error, "生成自车失败（可能与其他演员重叠）");
      return false;
    }
    impl_->ego_actor_id = vehicle->GetId();
    impl_->object_actor_ids.clear();
  } catch (const std::exception& e) {
    setError(error, std::string("生成自车异常: ") + e.what());
    return false;
  }
  return true;
#else
  (void)pose;
  setError(error, unavailableReason());
  return false;
#endif
}

bool CarlaBridge::applyControl(double throttle, double brake, double steering,
                               std::string* error) {
#if defined(ADSIM_HAS_CARLA)
  if (!impl_->connected || !impl_->client) {
    setError(error, "未连接 CARLA 服务端");
    return false;
  }
  try {
    auto world = impl_->client->GetWorld();
    auto vehicle = std::dynamic_pointer_cast<carla::client::Vehicle>(
        world->GetActor(impl_->ego_actor_id));
    if (!vehicle) {
      setError(error, "自车演员不存在或不是车辆");
      return false;
    }
    // 本模型给出的是前轮转角 (rad)，CARLA 需要归一化到 [-1,1]：
    // steer = δ / δ_max，δ_max 由 Config::max_steer_angle 给出（按蓝图标定）
    const double max_steer =
        impl_->config.max_steer_angle > 1e-3 ? impl_->config.max_steer_angle : 1.22;
    carla::rpc::VehicleControl control;
    control.throttle = static_cast<float>(clamp(throttle, 0.0, 1.0));
    control.brake = static_cast<float>(clamp(brake, 0.0, 1.0));
    control.steer = static_cast<float>(clamp(steering / max_steer, -1.0, 1.0));
    control.hand_brake = false;
    control.reverse = false;
    control.manual_gear_shift = false;
    control.gear = 0;
    vehicle->ApplyControl(control);
  } catch (const std::exception& e) {
    setError(error, std::string("下发控制失败: ") + e.what());
    return false;
  }
  return true;
#else
  (void)throttle;
  (void)brake;
  (void)steering;
  setError(error, unavailableReason());
  return false;
#endif
}

bool CarlaBridge::setEgoState(const VehicleState& state, std::string* error) {
#if defined(ADSIM_HAS_CARLA)
  if (!impl_->connected || !impl_->client) {
    setError(error, "未连接 CARLA 服务端");
    return false;
  }
  try {
    auto world = impl_->client->GetWorld();
    auto actor = world->GetActor(impl_->ego_actor_id);
    if (!actor) {
      setError(error, "自车演员不存在");
      return false;
    }
    // 位姿直接写回：数字回放/孪生同步时以本内核为准（服务端物理不做干预）
    actor->SetTransform(pose2ToTransform(state.pose(), 0.3));
    const double max_steer =
        impl_->config.max_steer_angle > 1e-3 ? impl_->config.max_steer_angle : 1.22;
    carla::rpc::VehicleControl control;
    control.throttle = static_cast<float>(clamp(state.throttle, 0.0, 1.0));
    control.brake = static_cast<float>(clamp(state.brake, 0.0, 1.0));
    control.steer = static_cast<float>(clamp(state.steering / max_steer, -1.0, 1.0));
    control.hand_brake = false;
    control.reverse = state.speed < 0.0;
    control.manual_gear_shift = false;
    control.gear = state.gear;
    auto vehicle = std::dynamic_pointer_cast<carla::client::Vehicle>(actor);
    if (vehicle) vehicle->ApplyControl(control);
  } catch (const std::exception& e) {
    setError(error, std::string("同步自车状态失败: ") + e.what());
    return false;
  }
  return true;
#else
  (void)state;
  setError(error, unavailableReason());
  return false;
#endif
}

bool CarlaBridge::syncObjects(const World& world, std::string* error) {
#if defined(ADSIM_HAS_CARLA)
  if (!impl_->connected || !impl_->client) {
    setError(error, "未连接 CARLA 服务端");
    return false;
  }
  std::size_t skipped_pedestrians = 0;
  std::size_t synced = 0;
  try {
    auto carla_world = impl_->client->GetWorld();
    for (const RoadObject& object : world.objects()) {
      if (object.type == RoadObject::Type::kPedestrian) {
        // 行人需要 WalkerController + AI 才能行走，本层暂不同步（见文件头限制）
        ++skipped_pedestrians;
        continue;
      }
      const auto it = impl_->object_actor_ids.find(object.id);
      if (it == impl_->object_actor_ids.end()) {
        auto blueprint =
            carla_world->GetBlueprintLibrary()->Find(impl_->config.object_blueprint);
        if (!blueprint) {
          setError(error, "未找到物体蓝图: " + impl_->config.object_blueprint);
          return false;
        }
        blueprint->SetAttribute("role_name", "adsim_object_" + std::to_string(object.id));
        auto actor = carla_world->SpawnActor<carla::client::Vehicle>(
            *blueprint, pose2ToTransform(object.pose, 0.3));
        if (!actor) continue;
        impl_->object_actor_ids[object.id] = actor->GetId();
      } else {
        auto actor = carla_world->GetActor(it->second);
        if (actor) actor->SetTransform(pose2ToTransform(object.pose, 0.3));
      }
      ++synced;
    }
  } catch (const std::exception& e) {
    setError(error, std::string("同步物体失败: ") + e.what());
    return false;
  }
  if (skipped_pedestrians > 0) {
    setError(error, "已同步 " + std::to_string(synced) + " 个物体，跳过 " +
                        std::to_string(skipped_pedestrians) +
                        " 个行人（需要 walker 控制器）");
  }
  return true;
#else
  (void)world;
  setError(error, unavailableReason());
  return false;
#endif
}

// ---------------------------------------------------------------------------
// 数字回放
// ---------------------------------------------------------------------------

SimulationResult CarlaBridge::replay(const std::vector<ReplayFrame>& frames, const World& world,
                                     const SimEngine::Config& config, std::string* error) {
  SimulationResult result;
  if (frames.size() < 2) {
    setError(error, "回放数据不足：至少需要 2 帧");
    return result;
  }
  if (!frames.front().has_ego) {
    setError(error, "回放数据首帧缺少自车状态");
    return result;
  }

  // 回放时长由**记录本身**决定：数据放完即结束。若沿用调用方的 max_duration，
  // 记录结束后的时间窗里跟踪策略仍会锁定最后一帧目标继续行驶，回放轨迹比
  // 原始记录多出一截（实测 13m/s 下多跑 1s 就多出 13m），指标对比随之失真。
  // 因此把它写进内核自己的 config 而不是只加在循环条件上 —— 否则收尾用的
  // run() 会按旧的 max_duration 继续空跑（无物体更新器）到超时。
  const std::vector<double> times = buildTimeline(frames, config.step_size);
  SimEngine::Config engine_config = config;
  engine_config.max_duration = std::min(config.max_duration, times.back());

  // 与场景层同样的用法：内核负责步进与指标采集，回放数据通过"每步改写
  // 物体列表"与"跟踪策略"注入。于是 TTC / 间距 / 碰撞 / 驶出路面等指标
  // 与在线仿真完全同源 —— 这正是数据回灌仿真的价值所在。
  SimEngine engine(world, VehicleModel(), engine_config);
  engine.setEgo(frames.front().ego);
  engine.setPolicy(std::make_shared<ReplayTrackingPolicy>(frames, 8.0, config.step_size));

  // 自建循环而不是 engine.run()：只有 step() 接受"物体更新器"，
  // 回放需要每步把记录中的其他交通参与者写回世界。
  while (engine.time() < engine_config.max_duration - kReplayTimeEps) {
    engine.step([&frames, &times](World& target_world, double t) {
      const std::size_t index = locateInterval(times, t);
      // 取"不晚于当前时刻"的最近一帧，保证回放的因果性
      const std::size_t selected =
          (index + 1 < times.size() && times[index + 1] <= t) ? index + 1 : index;
      target_world.objects() = frames[selected].objects;
    });
  }
  result = engine.run();  // 触发 finalize：汇总平均速度、里程与完成标志
  result.scenario_name = "Replay";
  return result;
}

std::size_t CarlaBridge::replayInCarla(const std::vector<ReplayFrame>& frames,
                                       double playback_speed, std::string* error) {
#if defined(ADSIM_HAS_CARLA)
  if (frames.empty()) {
    setError(error, "回放数据为空");
    return 0;
  }
  if (!impl_->connected || !impl_->client) {
    setError(error, "未连接 CARLA 服务端");
    return 0;
  }
  const double speed = playback_speed > 0.0 ? playback_speed : 1.0;
  const std::vector<double> times = buildTimeline(frames, impl_->config.fixed_delta_seconds);

  std::size_t played = 0;
  double previous = times.front();
  for (std::size_t i = 0; i < frames.size(); ++i) {
    std::string step_error;
    if (!setEgoState(frames[i].ego, &step_error)) {
      setError(error, step_error);
      break;
    }
    World object_world;
    object_world.objects() = frames[i].objects;
    std::string sync_error;
    if (!syncObjects(object_world, &sync_error) && error != nullptr && error->empty()) {
      *error = sync_error;  // 物体同步失败不中断回放，但记录原因
    }
    if (!tick(&step_error)) {
      setError(error, step_error);
      break;
    }
    // 按记录的时间间隔 sleep，使回放速度接近实车（playback_speed=1 即实时）
    const double dt = (times[i] - previous) / speed;
    if (dt > 1e-3) {
      std::this_thread::sleep_for(
          std::chrono::microseconds(static_cast<long long>(dt * 1e6)));
    }
    previous = times[i];
    ++played;
  }
  return played;
#else
  (void)frames;
  (void)playback_speed;
  setError(error, unavailableReason());
  return 0;
#endif
}

// ---------------------------------------------------------------------------
// 回放跟踪策略
// ---------------------------------------------------------------------------

ReplayTrackingPolicy::ReplayTrackingPolicy(std::vector<ReplayFrame> frames, double lookahead,
                                           double fallback_dt)
    : frames_(std::move(frames)),
      times_(buildTimeline(frames_, fallback_dt > 0.0 ? fallback_dt : 0.05)),
      lookahead_(lookahead > 1.0 ? lookahead : 8.0) {}

void ReplayTrackingPolicy::reset() { time_ = 0.0; }

bool ReplayTrackingPolicy::targetAt(double t, Pose2* pose, double* speed) const {
  if (frames_.size() < 2) return false;
  const std::size_t index = locateInterval(times_, t);
  const std::size_t next = std::min(index + 1, frames_.size() - 1);
  const double span = times_[next] - times_[index];
  const double ratio = span > 1e-9 ? clamp((t - times_[index]) / span, 0.0, 1.0) : 0.0;

  const Pose2 a = frames_[index].ego.pose();
  const Pose2 b = frames_[next].ego.pose();
  if (pose != nullptr) {
    // 位置线性插值；航向走最短弧插值，避免 ±π 附近出现"整车掉头"
    const Vec2 position = a.position() + (b.position() - a.position()) * ratio;
    const double theta = a.theta + normalizeAngle(b.theta - a.theta) * ratio;
    *pose = Pose2(position.x, position.y, normalizeAngle(theta));
  }
  if (speed != nullptr) {
    const double va = frames_[index].ego.speed;
    const double vb = frames_[next].ego.speed;
    *speed = va + (vb - va) * ratio;
  }
  return true;
}

IPolicy::Command ReplayTrackingPolicy::computeCommand(const VehicleState& ego, const World&,
                                                      double dt) {
  Command command;
  if (!(dt > 0.0) || !std::isfinite(dt)) return command;
  time_ += dt;

  Pose2 target;
  double target_speed = 0.0;
  if (!targetAt(time_, &target, &target_speed)) return command;

  // 横向：纯跟踪（pure pursuit）—— 用前视点算曲率，再反解前轮转角。
  // 与 VehicleModel::stepAlongPath 用的是同一套几何关系，行为可直接对照。
  const VehicleModel model;
  const Vec2 to_target = target.position() - ego.pose().position();
  if (to_target.squaredNorm() > 1e-12) {
    const double alpha = normalizeAngle(std::atan2(to_target.y, to_target.x) - ego.theta);
    const double kappa = 2.0 * std::sin(alpha) / lookahead_;
    command.steering = model.steeringFromCurvature(kappa);
  } else {
    command.steering = ego.steering;  // 与目标重合时保持当前转角，避免无意义抖动
  }
  // 纵向：一阶（P）速度跟踪，时间常数 0.5s —— 与内核同一控制律，
  // 这样回放轨迹与原始记录的偏差只来自车辆约束，而不是换了控制算法。
  command.acceleration = clamp(2.0 * (target_speed - ego.speed), -6.0, 3.0);
  command.valid = true;
  return command;
}

}  // namespace adsim
