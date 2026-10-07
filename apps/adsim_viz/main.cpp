// =============================================================================
//  adsim_viz — Qt 交互式仿真复盘前端入口
//
//  **可选依赖**：需要 Qt5/Qt6 Widgets，以 `-DADSIM_WITH_QT=ON` 构建时才存在。
//
//  用途：把一次仿真或一段路测回放装载进可交互窗口，支持鼠标缩放平移与
//  时间轴拖动，让工程师逐帧复盘某个决策失误点。
//
//  与 adsim_cli 的分工：
//      adsim_cli  —— 批量化、脚本化，产出 SVG/HTML 归档，适合 CI 与回归
//      adsim_viz  —— 交互式，适合人工定位问题
//  两者共用同一套几何与可视化语义（SvgCanvas / SimVisualizer）。
// =============================================================================
#include "adsim/common/Logger.h"
#include "adsim/sim/Scenario.h"
#include "adsim/viz/QtVisualizer.h"

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

void printUsage() {
  std::cout << R"(用法: adsim_viz <场景名> [选项]

运行一个内置场景并把结果装载进交互窗口。

选项:
  --duration=<秒>     覆盖场景默认时长
  --no-timeline       不显示底部时间轴
  --follow-ego        视角跟随自车
  --list              列出全部可用场景后退出
  --help, -h          显示本帮助

可用场景见 `adsim_cli scenarios`，或加 --list 查看。

交互方式:
  鼠标滚轮     缩放
  按住左键拖动  平移
  底部滑块     切换仿真帧)" << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
  std::vector<std::string> args(argv + 1, argv + argc);

  if (args.empty()) {
    printUsage();
    return 0;
  }

  adsim::QtVisualizerConfig config;

  std::string scenario_name;
  double duration = 0.0;
  bool list_only = false;

  for (const std::string& arg : args) {
    if (arg == "--help" || arg == "-h") {
      printUsage();
      return 0;
    } else if (arg == "--list") {
      list_only = true;
    } else if (arg == "--no-timeline") {
      config.show_timeline = false;
    } else if (arg == "--follow-ego") {
      config.follow_ego = true;
    } else if (arg.rfind("--duration=", 0) == 0) {
      try {
        duration = std::stod(arg.substr(11));
      } catch (...) {
        std::cerr << "时长参数无法解析: " << arg << std::endl;
        return 1;
      }
    } else if (arg.rfind("--", 0) != 0) {
      scenario_name = arg;
    } else {
      std::cerr << "未知参数: " << arg << std::endl;
      printUsage();
      return 1;
    }
  }

  // --list 不依赖 Qt，因此在未启用 Qt 的构建里也能用
  if (list_only) {
    std::cout << "可用场景:" << std::endl;
    for (const adsim::ScenarioInfo& info :
         adsim::ScenarioRegistry::instance().allInfo()) {
      std::cout << "  " << info.name << "  [" << info.category << "]  "
                << info.description << std::endl;
    }
    return 0;
  }

  // 未编译进 Qt 时给出明确提示，而不是崩溃或静默失败
  if (!adsim::QtVisualizer::isCompiled()) {
    std::cerr << "本构建未包含 Qt 支持。\n"
              << "请以 -DADSIM_WITH_QT=ON 重新配置并构建（需已安装 Qt5/Qt6 Widgets）。\n"
              << "若只需要产出图与报告，可改用 adsim_cli scenario <名称> --report=... --svg=..."
              << std::endl;
    return 2;
  }

  if (scenario_name.empty()) {
    std::cerr << "请指定场景名（用 --list 查看可用场景）" << std::endl;
    return 1;
  }

  std::unique_ptr<adsim::Scenario> scenario =
      adsim::ScenarioRegistry::instance().create(scenario_name);
  if (scenario == nullptr) {
    std::cerr << "未知场景: " << scenario_name << "（用 --list 查看可用场景）" << std::endl;
    return 1;
  }

  adsim::SimEngine::Config engine_config;
  if (duration > 0.0) {
    engine_config.max_duration = duration;
  }

  const adsim::ScenarioInfo info = scenario->info();
  ADSIM_LOG_INFO("运行场景: ", info.name, " (", info.category, ")");

  // 不带策略运行：只回放场景本身，不做决策。
  // 交互复盘关心的是"场景长什么样"，算法对比交给 adsim_cli。
  const adsim::SimulationResult result = scenario->run(engine_config);
  const adsim::World world = scenario->buildWorld();

  adsim::QtVisualizer visualizer(config);
  visualizer.loadSimulation(result, world);

  return visualizer.exec();
}
