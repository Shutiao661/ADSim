// =============================================================================
//  commands.cpp — adsim_cli 各命令的实现
// =============================================================================
#include "adsim/agent/ScenarioTagger.h"
#include "adsim/common/Logger.h"
#include "adsim/common/MemoryPool.h"
#include "adsim/common/Profiler.h"
#include "adsim/common/TextTable.h"
#include "adsim/common/ThreadPool.h"
#include "adsim/datapipeline/DataPipeline.h"
#include "adsim/datapipeline/GpsFilter.h"
#include "adsim/datapipeline/MessageCodec.h"
#include "adsim/datapipeline/PointCloudFilter.h"
#include "adsim/datapipeline/RosBagReader.h"
#include "adsim/datapipeline/RosBagWriter.h"
#include "adsim/planning/PathPlanner.h"
#include "adsim/planning/PlanningPolicy.h"
#include "adsim/planning/geometry/CurveFit.h"
#include "adsim/planning/geometry/Spline.h"
#include "adsim/planning/optimizer/PathOptimizer.h"
#include "adsim/sim/Scenario.h"
#include "adsim/viz/SimVisualizer.h"
#include "adsim/viz/SvgCanvas.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace adsim {
namespace cli {

namespace {

// ---------------------------------------------------------------------------
// 参数解析辅助
// ---------------------------------------------------------------------------

class ArgParser {
 public:
  explicit ArgParser(const std::vector<std::string>& args) : args_(args) {}

  /// 提取位置参数（非 -- 开头）
  std::vector<std::string> positional() const {
    std::vector<std::string> result;
    for (const std::string& arg : args_) {
      if (arg.rfind("--", 0) != 0) result.push_back(arg);
    }
    return result;
  }

  bool has(const std::string& flag) const {
    return std::find(args_.begin(), args_.end(), "--" + flag) != args_.end();
  }

  std::string get(const std::string& name, const std::string& fallback = "") const {
    const std::string prefix = "--" + name + "=";
    for (const std::string& arg : args_) {
      if (arg.rfind(prefix, 0) == 0) return arg.substr(prefix.size());
    }
    return fallback;
  }

  double getDouble(const std::string& name, double fallback) const {
    const std::string value = get(name);
    if (value.empty()) return fallback;
    try {
      return std::stod(value);
    } catch (...) {
      return fallback;
    }
  }

  int getInt(const std::string& name, int fallback) const {
    const std::string value = get(name);
    if (value.empty()) return fallback;
    try {
      return std::stoi(value);
    } catch (...) {
      return fallback;
    }
  }

  /// 打印未知参数警告
  void warnUnknown(const std::vector<std::string>& known) const {
    for (const std::string& arg : args_) {
      if (arg.rfind("--", 0) != 0) continue;
      const std::size_t equals = arg.find('=');
      const std::string name = arg.substr(2, equals == std::string::npos
                                                 ? std::string::npos
                                                 : equals - 2);
      if (std::find(known.begin(), known.end(), name) == known.end()) {
        std::cerr << "警告: 未知参数 " << arg << std::endl;
      }
    }
  }

 private:
  std::vector<std::string> args_;
};

/// 生成只含典型可选依赖状态的文本
std::string optionalDependencyLine(const char* name, bool enabled) {
  return std::string("  ") + name + (enabled ? " 已启用" : " 未启用");
}

}  // namespace

// ---------------------------------------------------------------------------
// info
// ---------------------------------------------------------------------------

int commandInfo(const std::vector<std::string>& args) {
  (void)args;

  std::cout << "\nADSim — 自动驾驶行为决策仿真及数据分析平台\n" << std::endl;
  std::cout << "版本      : 1.0.0" << std::endl;
  std::cout << "语言标准  : C++17" << std::endl;

  const unsigned cores = std::thread::hardware_concurrency();
  std::cout << "硬件并发  : " << cores << " 线程" << std::endl;

  std::cout << "\n可选依赖:" << std::endl;
#if defined(ADSIM_HAS_CARLA)
  std::cout << optionalDependencyLine("CARLA", true) << std::endl;
#else
  std::cout << optionalDependencyLine("CARLA", false)
            << "   (构建时加 -DADSIM_WITH_CARLA=ON)" << std::endl;
#endif
#if defined(ADSIM_HAS_CERES)
  std::cout << optionalDependencyLine("Ceres Solver", true) << std::endl;
#else
  std::cout << optionalDependencyLine("Ceres Solver", false)
            << "   (构建时加 -DADSIM_WITH_CERES=ON)" << std::endl;
#endif
#if defined(ADSIM_HAS_QT)
  std::cout << optionalDependencyLine("Qt", true) << std::endl;
#else
  std::cout << optionalDependencyLine("Qt", false)
            << "   (构建时加 -DADSIM_WITH_QT=ON)" << std::endl;
#endif
#if defined(ADSIM_HAS_ROS)
  std::cout << optionalDependencyLine("ROS", true) << std::endl;
#else
  std::cout << optionalDependencyLine("ROS", false)
            << "   (构建时加 -DADSIM_WITH_ROS=ON)" << std::endl;
#endif
#if defined(ADSIM_HAS_CURL)
  std::cout << optionalDependencyLine("libcurl", true) << std::endl;
#else
  std::cout << optionalDependencyLine("libcurl", false)
            << "   (构建时加 -DADSIM_WITH_CURL=ON)" << std::endl;
#endif

  std::cout << "\n核心能力（零外部依赖，任何环境下均可用）:" << std::endl;
  std::cout << "  • ROS Bag v2.0 流式解析与写入" << std::endl;
  std::cout << "  • 多线程数据管道（内存池 + 无锁环形缓冲 + 线程池）" << std::endl;
  std::cout << "  • 多传感器时间对齐与点云/GPS 降噪" << std::endl;
  std::cout << "  • 有限状态机 + 行为树的行为决策" << std::endl;
  std::cout << "  • Delaunay/Voronoi 路径生成 + B 样条拟合 + 数值优化" << std::endl;
  std::cout << "  • 二维仿真内核与危险工况库" << std::endl;
  std::cout << "  • SVG 可视化与 HTML 报告" << std::endl;
  std::cout << "  • 交互场景规则/LLM 打标" << std::endl;
  std::cout << std::endl;

  return 0;
}

// ---------------------------------------------------------------------------
// probe
// ---------------------------------------------------------------------------

int commandProbe(const std::vector<std::string>& args) {
  ArgParser parser(args);
  const std::vector<std::string> positional = parser.positional();

  if (positional.empty()) {
    std::cerr << "用法: adsim_cli probe <bag 文件>" << std::endl;
    return 1;
  }

  const std::string path = positional.front();

  ADSIM_PROFILE_SCOPE("probe");

  RosBagReader reader(path);
  reader.open(true);

  std::cout << reader.info().summary() << std::endl;
  return 0;
}

// ---------------------------------------------------------------------------
// pipeline
// ---------------------------------------------------------------------------

int commandPipeline(const std::vector<std::string>& args) {
  ArgParser parser(args);
  const std::vector<std::string> positional = parser.positional();

  if (positional.empty()) {
    std::cerr << "用法: adsim_cli pipeline <bag 文件> [--out=<输出>]" << std::endl;
    return 1;
  }

  DataPipeline::Config config;
  config.thread_count = static_cast<std::size_t>(parser.getInt("threads", 0));
  config.queue_capacity = static_cast<std::size_t>(parser.getInt("queue", 256));
  config.enable_lidar_filter = !parser.has("no-lidar-filter");
  config.enable_gps_filter = !parser.has("no-gps-filter");
  config.keep_lidar = true;

  const std::string input = positional.front();
  const std::string output = parser.get("out");

  DataPipeline pipeline(config);
  const DataPipeline::Report report = pipeline.run(input, output);

  std::cout << report.toString() << std::endl;

  if (config.enable_lidar_filter) {
    std::cout << "点云降噪: 输入 " << report.lidar_points_in << " 点 → 输出 "
              << report.lidar_points_out << " 点";
    if (report.lidar_points_in > 0) {
      const double ratio = 100.0 * static_cast<double>(report.lidar_points_out) /
                           static_cast<double>(report.lidar_points_in);
      std::cout << std::fixed << std::setprecision(1) << "（保留 " << ratio << "%）";
    }
    std::cout << std::endl;
  }

  if (config.enable_profiling) {
    std::cout << Profiler::instance().report();
  }

  return 0;
}

// ---------------------------------------------------------------------------
// scenarios
// ---------------------------------------------------------------------------

int commandScenarios(const std::vector<std::string>& args) {
  (void)args;

  const std::vector<ScenarioInfo> infos = ScenarioRegistry::instance().allInfo();
  if (infos.empty()) {
    std::cout << "未注册任何场景" << std::endl;
    return 0;
  }

  std::cout << "\n内置场景库（共 " << infos.size() << " 个）\n" << std::endl;
  std::cout << std::left << std::setw(26) << "名称" << std::setw(10) << "类别"
            << std::setw(8) << "危险" << "说明" << std::endl;
  std::cout << std::string(90, '-') << std::endl;

  for (const ScenarioInfo& info : infos) {
    std::cout << std::left << std::setw(26) << info.name << std::setw(10) << info.category
              << std::setw(8) << (info.is_critical ? "是" : "否") << info.description
              << std::endl;
  }
  std::cout << "\n运行某个场景: adsim_cli scenario <名称>  例如 adsim_cli scenario LeadBrake" << std::endl;
  return 0;
}

// ---------------------------------------------------------------------------
// scenario
// ---------------------------------------------------------------------------

int commandScenario(const std::vector<std::string>& args) {
  ArgParser parser(args);
  const std::vector<std::string> positional = parser.positional();

  if (positional.empty()) {
    std::cerr << "用法: adsim_cli scenario <场景名> [--policy=planning|cruise]" << std::endl;
    std::cerr << "可用场景见: adsim_cli scenarios" << std::endl;
    return 1;
  }

  const std::string name = positional.front();
  std::unique_ptr<Scenario> scenario = ScenarioRegistry::instance().create(name);
  if (scenario == nullptr) {
    std::cerr << "未知场景: " << name << std::endl;
    std::cerr << "可用场景见: adsim_cli scenarios" << std::endl;
    return 1;
  }

  // ---- 组装策略 ----
  SimEngine::Config engine_config;
  const double duration = parser.getDouble("duration", 0.0);
  if (duration > 0.0) {
    engine_config.max_duration = duration;
  }

  const std::string policy_name = parser.get("policy", "planning");
  std::shared_ptr<IPolicy> policy;

  if (policy_name == "cruise") {
    // 对照策略：定速巡航 + 纯跟踪，不含决策与优化。
    // 用于回答"引入决策与优化到底带来了多少收益"。
    PlanningPolicy::Config config;
    config.enable_decision = false;
    config.enable_path_optimization = false;
    policy = std::make_shared<PlanningPolicy>(config);
  } else if (policy_name == "planning") {
    PlanningPolicy::Config config;
    config.enable_path_optimization = !parser.has("no-optimize");
    policy = std::make_shared<PlanningPolicy>(config);
  } else {
    std::cerr << "未知策略: " << policy_name << "（可选 planning / cruise）" << std::endl;
    return 1;
  }

  const ScenarioInfo info = scenario->info();

  std::cout << "\n运行场景: " << info.name << std::endl;
  std::cout << "  类别    : " << info.category << std::endl;
  std::cout << "  说明    : " << info.description << std::endl;
  std::cout << "  策略    : " << policy->name() << std::endl;
  std::cout << "  仿真时长: " << engine_config.max_duration << " s" << std::endl;

  // ---- 运行 ----
  const SimulationResult result = scenario->run(engine_config, policy);

  std::cout << result.toString() << std::endl;

  // ---- 可视化 ----
  const std::string svg_path = parser.get("svg");
  const std::string report_path = parser.get("report");
  const World world = scenario->buildWorld();

  if (!svg_path.empty()) {
    SimVisualizer visualizer{VisualizerConfig()};
    if (visualizer.writeReport(result, world, svg_path)) {
      std::cout << "可视化已输出: " << svg_path << std::endl;
    } else {
      std::cerr << "可视化输出失败: " << svg_path << std::endl;
    }
  }

  if (!report_path.empty()) {
    SimVisualizer visualizer{VisualizerConfig()};
    const std::string overview = visualizer.renderOverview(result, world);
    const std::string timeline = visualizer.renderTimeline(result);

    const std::string html = buildHtmlReport(
        {{"仿真总览", overview}, {"时序指标", timeline}}, "ADSim 仿真报告 — " + info.name);

    std::ofstream out(report_path);
    if (out) {
      out << html;
      std::cout << "HTML 报告已输出: " << report_path << std::endl;
    } else {
      std::cerr << "报告输出失败: " << report_path << std::endl;
    }
  }

  // 危险工况下返回非零码，便于脚本化回归测试直接判失败
  return result.isCritical() ? 2 : 0;
}

// ---------------------------------------------------------------------------
// smooth
// ---------------------------------------------------------------------------

namespace {

/// 单个平滑后端的一次实验结果
struct SmoothingTrial {
  std::string name;
  std::vector<Vec2> path;
  double max_curvature{0.0};
  double curvature_rate{0.0};
  double max_deviation{0.0};
  double elapsed_ms{0.0};
  bool constraint_met{false};
  std::string note;
};

/// 构造一条带噪声的圆弧路径（模拟上游模块给出的锯齿路点）
std::vector<Vec2> makeJaggedArc(int point_count, double radius, double noise,
                                unsigned seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> jitter(0.0, noise);

  std::vector<Vec2> path;
  path.reserve(static_cast<std::size_t>(point_count));
  const double angular_step = 2.0 / static_cast<double>(point_count);

  for (int i = 0; i < point_count; ++i) {
    const double angle = i * angular_step;
    path.push_back({radius * std::sin(angle) + jitter(rng),
                    radius * (1.0 - std::cos(angle)) + jitter(rng)});
  }
  return path;
}

/// 跑一个后端，采集全部指标
SmoothingTrial runTrial(const std::string& name, SmoothingBackend backend,
                        const std::vector<Vec2>& raw, double max_kappa,
                        double bspline_deviation_limit) {
  SmoothingTrial trial;
  trial.name = name;

  PathPlanner::Config config;
  config.max_curvature = max_kappa;
  config.smoothing = backend;
  config.bspline_max_deviation = bspline_deviation_limit;
  config.optimizer.max_curvature = max_kappa;
  config.optimizer.max_iterations = 500;

  PathPlanningRequest request;
  request.ego = VehicleState();
  // 直接喂参考路径：借助 PathPlanner 的平滑环节，但不走车道采样与绕障
  request.world = nullptr;

  // PathPlanner::plan 需要世界来生成参考路径，这里改用它内部的平滑逻辑：
  // 通过一个只含参考路径的请求不可行，因此直接调用两个后端本身。
  const auto start = std::chrono::steady_clock::now();

  if (backend == SmoothingBackend::kBSpline) {
    const Trajectory trajectory =
        geometry::smoothPathWithCurvatureLimit(raw, max_kappa, 200);
    for (const TrajectoryPoint& point : trajectory) {
      trial.path.push_back({point.x, point.y});
    }
  } else {
    PathOptimizerOptions options = config.optimizer;
    PathOptimizer optimizer(options);
    PathOptimizationReport report;
    trial.path = optimizer.optimize(raw, &report);
    trial.note = report.converged ? "" : "未收敛";
  }

  trial.elapsed_ms = std::chrono::duration<double, std::milli>(
                         std::chrono::steady_clock::now() - start)
                         .count();

  // 指标统一在"输出路径"上计算，保证两个后端可比
  trial.max_curvature = PathOptimizer::maxAbsCurvature(trial.path);
  trial.curvature_rate = PathOptimizer::maxCurvatureRate(trial.path);

  for (const Vec2& point : trial.path) {
    trial.max_deviation =
        std::max(trial.max_deviation, geometry::distanceToPolyline(raw, point));
  }

  trial.constraint_met = trial.max_curvature <= max_kappa * 1.05;
  return trial;
}

/// 打印对照表。列宽用显示宽度对齐——setw 按字节计宽，中英混排会错位。
void printTrialTable(const std::vector<SmoothingTrial>& trials) {
  constexpr std::size_t kName = 16;
  constexpr std::size_t kCount = 8;
  constexpr std::size_t kValue = 13;
  constexpr std::size_t kFlag = 10;

  const std::string rule(kName + kCount + kValue * 4 + kFlag, '-');

  std::cout << "\n" << rule << "\n";
  std::cout << padTo("后端", kName, true) << padTo("点数", kCount, false)
            << padTo("最大曲率", kValue, false) << padTo("曲率变化率", kValue, false)
            << padTo("最大偏移", kValue, false) << padTo("耗时(ms)", kValue, false)
            << padTo("满足约束", kFlag, false) << "\n";
  std::cout << rule << "\n";

  for (const SmoothingTrial& trial : trials) {
    std::cout << padTo(trial.name, kName, true)
              << padTo(std::to_string(trial.path.size()), kCount, false)
              << padTo(trial.max_curvature, 4, kValue, false)
              << padTo(trial.curvature_rate, 4, kValue, false)
              << padTo(trial.max_deviation, 3, kValue, false)
              << padTo(trial.elapsed_ms, 2, kValue, false)
              << padTo(trial.constraint_met ? "是" : "否", kFlag, false);
    if (!trial.note.empty()) std::cout << "  " << trial.note;
    std::cout << "\n";
  }
  std::cout << rule << "\n";
}

}  // namespace

int commandSmooth(const std::vector<std::string>& args) {
  ArgParser parser(args);

  const int point_count = parser.getInt("points", 41);
  const double radius = parser.getDouble("radius", 30.0);
  const double noise = parser.getDouble("noise", 0.18);
  const double max_kappa = parser.getDouble("max-kappa", 0.05);
  const double deviation_limit = parser.getDouble("deviation-limit", 0.8);
  const std::string svg_path = parser.get("svg");
  const std::string which = parser.get("backend", "compare");

  if (point_count < 5) {
    std::cerr << "路径点数至少为 5" << std::endl;
    return 1;
  }
  if (radius <= 0.0) {
    std::cerr << "圆弧半径必须为正" << std::endl;
    return 1;
  }

  const std::vector<Vec2> raw = makeJaggedArc(point_count, radius, noise, 20240924u);

  std::cout << "\n路径平滑对照实验" << std::endl;
  std::cout << std::fixed << std::setprecision(4);
  std::cout << "  基准圆弧半径  : " << radius << " m（理想曲率 " << (1.0 / radius)
            << " 1/m）" << std::endl;
  std::cout << "  横向噪声 σ    : " << noise << " m" << std::endl;
  std::cout << "  曲率上限      : " << max_kappa << " 1/m" << std::endl;
  std::cout << "  输入: " << raw.size() << " 点，最大曲率 "
            << PathOptimizer::maxAbsCurvature(raw) << " 1/m，曲率变化率 "
            << PathOptimizer::maxCurvatureRate(raw) << " 1/m²" << std::endl;

  // ---- 选择要跑的后端 ----
  std::vector<std::pair<std::string, SmoothingBackend>> plan;
  if (which == "compare") {
    plan = {{"bspline", SmoothingBackend::kBSpline},
            {"optimization", SmoothingBackend::kOptimization}};
  } else if (which == "bspline") {
    plan = {{"bspline", SmoothingBackend::kBSpline}};
  } else if (which == "optimization") {
    plan = {{"optimization", SmoothingBackend::kOptimization}};
  } else {
    std::cerr << "未知后端: " << which
              << "（可选 compare / bspline / optimization）" << std::endl;
    return 1;
  }

  std::vector<SmoothingTrial> trials;
  for (const auto& entry : plan) {
    trials.push_back(
        runTrial(entry.first, entry.second, raw, max_kappa, deviation_limit));
  }

  printTrialTable(trials);

  // ---- 结论 ----
  if (trials.size() == 2) {
    const SmoothingTrial& bs = trials[0];
    const SmoothingTrial& opt = trials[1];

    std::cout << "\n结论:" << std::endl;
    std::cout.precision(4);
    std::cout << "  B 样条的曲率变化率是优化的 "
              << (opt.curvature_rate > 1e-9 ? bs.curvature_rate / opt.curvature_rate : 0.0)
              << " 倍（越低越平滑）" << std::endl;
    std::cout << "  B 样条的偏移是优化的 "
              << (opt.max_deviation > 1e-9 ? bs.max_deviation / opt.max_deviation : 0.0)
              << " 倍（越低越贴合原路径）" << std::endl;
    std::cout.precision(1);
    std::cout << "  B 样条耗时是优化的 "
              << (bs.elapsed_ms > 1e-9 ? opt.elapsed_ms / bs.elapsed_ms : 0.0)
              << " 分之一" << std::endl;

    std::cout << "\n  两个后端不是替代关系，而是取舍：" << std::endl;
    std::cout << "    需要精确贴合车道（偏移 < 车道宽的一半）→ 数值优化" << std::endl;
    std::cout << "    生成全新平滑路径、对贴合要求宽松   → B 样条" << std::endl;
    std::cout << "    不确定                             → PathPlanner 的"
              << " kAuto（按偏移阈值 " << deviation_limit << " m 自动裁决）" << std::endl;
  }

  // ---- 可视化：把两条结果画在同一张图上 ----
  if (!svg_path.empty()) {
    BoundingBox2 bounds;
    for (const Vec2& p : raw) bounds.expand(p);
    for (const SmoothingTrial& trial : trials) {
      for (const Vec2& p : trial.path) bounds.expand(p);
    }

    SvgCanvas canvas(1100.0, 800.0);
    canvas.setWorldBounds(bounds, 40.0);
    canvas.drawGrid(5.0, Style::solid("#EEEEEE", 0.6));

    canvas.drawPolyline(raw, Style::dashedLine("#9E9E9E", 1.5, 5.0));

    const char* colors[] = {"#1565C0", "#C62828"};
    double label_y = bounds.max.y;
    for (std::size_t i = 0; i < trials.size(); ++i) {
      Trajectory trajectory(trials[i].path.size());
      for (std::size_t k = 0; k < trials[i].path.size(); ++k) {
        trajectory[k].x = trials[i].path[k].x;
        trajectory[k].y = trials[i].path[k].y;
      }
      canvas.drawTrajectoryColored(trajectory, max_kappa, 2.5);
      canvas.drawText({bounds.min.x, label_y}, trials[i].name, 13.0,
                      colors[i % 2]);
      label_y -= (bounds.max.y - bounds.min.y) * 0.05;
    }
    canvas.drawText({bounds.min.x, label_y}, "原始路径（灰色虚线）", 13.0, "#616161");

    if (canvas.save(svg_path)) {
      std::cout << "\n对照图已输出: " << svg_path << std::endl;
    } else {
      std::cerr << "对照图输出失败: " << svg_path << std::endl;
    }
  }

  return 0;
}

// ---------------------------------------------------------------------------
// tag
// ---------------------------------------------------------------------------

int commandTag(const std::vector<std::string>& args) {
  ArgParser parser(args);
  const std::vector<std::string> positional = parser.positional();

  if (positional.empty()) {
    std::cerr << "用法: adsim_cli tag <bag 文件> [--limit=N] [--no-llm]" << std::endl;
    return 1;
  }

  const std::string path = positional.front();
  const int limit = parser.getInt("limit", 500);

  std::cout << "\n读取路测数据: " << path << std::endl;

  // ---- 读取并对齐数据 ----
  RosBagReader reader(path);
  reader.open(true);

  std::vector<GpsFrame> gps_frames;
  std::vector<VehicleState> odometry;
  std::vector<LidarFrame> lidar_frames;

  std::size_t processed = 0;
  reader.forEachMessage([&](const bag::MessageView& view) {
    if (processed >= static_cast<std::size_t>(limit)) return false;

    if (view.type() == "sensor_msgs/NavSatFix") {
      gps_frames.push_back(ros::decodeNavSatFix(view.data, view.size));
    } else if (view.type() == "nav_msgs/Odometry") {
      odometry.push_back(ros::decodeOdometry(view.data, view.size));
    } else if (view.type() == "sensor_msgs/PointCloud2") {
      // 打标不关心点云细节，只保留时间戳占位，避免占用大量内存
      LidarFrame frame;
      frame.stamp = view.stamp;
      lidar_frames.push_back(frame);
    }

    ++processed;
    return true;
  });

  std::cout << "  读取消息: " << processed << " 条" << std::endl;
  std::cout << "  GPS " << gps_frames.size() << " 帧, 状态 " << odometry.size()
            << " 帧, 点云 " << lidar_frames.size() << " 帧" << std::endl;

  if (odometry.empty() && gps_frames.empty()) {
    std::cerr << "数据中没有可用于打标的内容" << std::endl;
    return 1;
  }

  TimeAligner::Config align_config;
  align_config.match_tolerance = 100 * 1000000LL;
  TimeAligner aligner(align_config);
  const std::vector<AlignedFrame> aligned = aligner.align(gps_frames, odometry, lidar_frames);

  std::cout << "  对齐帧数: " << aligned.size() << std::endl;

  // ---- 打标 ----
  ScenarioTagger::Config tagger_config;
  tagger_config.enable_llm = !parser.has("no-llm");

  ScenarioTagger tagger(tagger_config);
  if (tagger_config.enable_llm) {
    // 默认使用离线桩实现：规则打标始终可用，LLM 只是增强
    auto client = std::make_shared<MockLlmClient>();
    tagger.setLlmClient(client);
  }

  const TaggingResult result = tagger.tag(aligned);

  std::cout << "\n" << result.toString() << std::endl;

  if (result.critical) {
    std::cout << "\n该片段被判定为危险工况，建议人工复核。" << std::endl;
  }

  return 0;
}

// ---------------------------------------------------------------------------
// bench
// ---------------------------------------------------------------------------

int commandBench(const std::vector<std::string>& args) {
  ArgParser parser(args);

  const int frames = parser.getInt("frames", 200);
  const int points = parser.getInt("points", 20000);

  if (frames <= 0 || points <= 0) {
    std::cerr << "帧数与点数必须为正" << std::endl;
    return 1;
  }

  std::cout << "\n性能基准测试" << std::endl;
  std::cout << "  点云帧数: " << frames << "，每帧 " << points << " 点" << std::endl;
  std::cout << "  工作线程: " << std::thread::hardware_concurrency() << std::endl;

  // ---- 构造测试数据 ----
  std::mt19937 rng(20240924u);
  std::uniform_real_distribution<double> angle(0.0, 2.0 * kPi);
  std::uniform_real_distribution<double> range(3.0, 60.0);
  std::normal_distribution<double> noise(0.0, 0.02);

  std::vector<LidarFrame> cloud(static_cast<std::size_t>(frames));
  for (LidarFrame& frame : cloud) {
    frame.points.reserve(static_cast<std::size_t>(points));
    for (int i = 0; i < points; ++i) {
      const double a = angle(rng);
      const double r = range(rng);
      LidarPoint p;
      p.x = static_cast<float>(r * std::cos(a) + noise(rng));
      p.y = static_cast<float>(r * std::sin(a) + noise(rng));
      p.z = static_cast<float>(noise(rng) * 5.0 - 1.8);
      p.intensity = static_cast<float>(i % 255);
      frame.points.push_back(p);
    }
  }

  // ---- 单线程基准 ----
  PointCloudFilter filter;
  const auto serial_start = std::chrono::steady_clock::now();
  std::size_t total_out = 0;
  {
    ADSIM_PROFILE_SCOPE("点云降噪_单线程");
    for (const LidarFrame& frame : cloud) {
      total_out += filter.filter(frame).points.size();
    }
  }
  const double serial_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - serial_start).count();

  std::cout << "\n单线程:" << std::endl;
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "  耗时  : " << serial_seconds << " s" << std::endl;
  std::cout << "  吞吐  : " << frames / serial_seconds << " 帧/s" << std::endl;

  // ---- 多线程基准 ----
  ThreadPool pool(0);
  const auto parallel_start = std::chrono::steady_clock::now();

  std::vector<std::size_t> outputs(cloud.size(), 0);
  {
    ADSIM_PROFILE_SCOPE("点云降噪_多线程");
    pool.parallelFor(0, cloud.size(), 1, [&cloud, &outputs](std::size_t i) {
      PointCloudFilter local_filter;
      outputs[i] = local_filter.filter(cloud[i]).points.size();
    });
  }

  const double parallel_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - parallel_start).count();

  std::size_t parallel_out = 0;
  for (std::size_t n : outputs) parallel_out += n;

  std::cout << "\n多线程（" << pool.size() << " 线程）:" << std::endl;
  std::cout << "  耗时  : " << parallel_seconds << " s" << std::endl;
  std::cout << "  吞吐  : " << frames / parallel_seconds << " 帧/s" << std::endl;
  if (parallel_seconds > 0.0) {
    std::cout << "  加速比: " << serial_seconds / parallel_seconds << "×" << std::endl;
  }

  // 并行与串行必须产出相同的点数，否则说明存在数据竞争
  if (total_out != parallel_out) {
    std::cerr << "\n警告: 并行与串行结果不一致（" << parallel_out << " vs " << total_out
              << "），可能存在数据竞争" << std::endl;
    return 1;
  }
  std::cout << "\n并行与串行结果一致（输出点数 " << parallel_out << "）" << std::endl;

  // ---- 内存池统计 ----
  const auto pool_stats = MemoryPoolRegistry::allStats();
  if (!pool_stats.empty()) {
    std::cout << "\n内存池:" << std::endl;
    for (const auto& entry : pool_stats) {
      std::cout << "  块大小 " << entry.first << " B: 分配 "
                << entry.second.allocate_calls << " 次, 峰值占用 "
                << entry.second.peak_blocks_in_use << " 块, 缓存命中率 " << std::setprecision(1)
                << entry.second.thread_cache_hit_rate * 100.0 << "%" << std::endl;
    }
  }

  std::cout << Profiler::instance().report();
  return 0;
}

}  // namespace cli
}  // namespace adsim
