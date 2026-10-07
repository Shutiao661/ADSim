// =============================================================================
//  adsim_cli — ADSim 平台命令行入口
//
//  把平台的四大能力暴露为可直接使用的命令：
//     数据管道   probe / pipeline      —— 解包、对齐、降噪、落盘
//     仿真验证   scenarios / scenario  —— 危险工况复现与算法回归
//     路径规划   smooth                —— 路径平滑与曲率约束
//     数据打标   tag                   —— 交互场景的规则/LLM 标注
//
//  所有命令都支持 --help 查看详细用法。
// =============================================================================
#include "adsim/common/Logger.h"
#include "adsim/common/Profiler.h"

#include <cstring>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace adsim {
namespace cli {

// 各命令实现在 commands.cpp
int commandInfo(const std::vector<std::string>& args);
int commandProbe(const std::vector<std::string>& args);
int commandPipeline(const std::vector<std::string>& args);
int commandScenarios(const std::vector<std::string>& args);
int commandScenario(const std::vector<std::string>& args);
int commandSmooth(const std::vector<std::string>& args);
int commandTag(const std::vector<std::string>& args);
int commandBench(const std::vector<std::string>& args);

}  // namespace cli
}  // namespace adsim

namespace {

using CommandFn = std::function<int(const std::vector<std::string>&)>;

const std::map<std::string, CommandFn>& commands() {
  static const std::map<std::string, CommandFn> table = {
      {"info", adsim::cli::commandInfo},
      {"probe", adsim::cli::commandProbe},
      {"pipeline", adsim::cli::commandPipeline},
      {"scenarios", adsim::cli::commandScenarios},
      {"scenario", adsim::cli::commandScenario},
      {"smooth", adsim::cli::commandSmooth},
      {"tag", adsim::cli::commandTag},
      {"bench", adsim::cli::commandBench},
  };
  return table;
}

void printBanner() {
  std::cout << R"(
    ___    ____  _
   /   |  / __ \(_)___ ___ _
  / /| | / / / / / __ `__ \
 / ___ |/ /_/ / / / / / / /
/_/  |_/_____/_/_/ /_/ /_/

  自动驾驶行为决策仿真及数据分析平台
)" << std::endl;
}

void printUsage() {
  std::cout << R"(用法: adsim_cli <命令> [选项]

数据管道:
  probe <bag>              查看 bag 文件的元信息（话题、消息数、时间跨度）
  pipeline <bag> [选项]    执行完整处理管道：解析 → 时间对齐 → 降噪 → 落盘
      --out=<路径>             输出 bag 路径
      --threads=<N>            工作线程数（默认使用全部核心）
      --no-lidar-filter        关闭点云降噪
      --no-gps-filter          关闭 GPS 滤波
      --queue=<N>              流水线缓冲槽位数（默认 256）

仿真验证:
  scenarios                列出全部内置场景
  scenario <名称> [选项]   运行指定场景并输出安全指标
      --policy=<类型>          决策策略: planning | cruise（默认 planning）
      --svg=<路径>             输出可视化 SVG
      --report=<路径>          输出 HTML 报告
      --no-optimize            关闭路径优化（用于对比优化增益）
      --duration=<秒>          覆盖场景默认时长

路径规划:
  smooth [选项]            路径平滑对照实验：B 样条 vs 数值优化
      --backend=<类型>         平滑后端: compare | bspline | optimization
                               （默认 compare，跑两个后端并对照）
      --points=<N>             路径点数（默认 41）
      --radius=<R>             基准圆弧半径（默认 30）
      --noise=<σ>              横向噪声标准差（默认 0.18）
      --max-kappa=<κ>          曲率上限（默认 0.05）
      --deviation-limit=<m>    自动选后端时允许的最大偏移（默认 0.8）
      --svg=<路径>             输出对照图

数据打标:
  tag <bag> [选项]         对路测数据做交互场景标注
      --limit=<N>              最多处理多少帧（默认 500）
      --no-llm                 仅使用规则打标

其他:
  info                     显示平台信息与已启用的可选依赖
  bench [选项]             性能基准测试
      --frames=<N>             点云帧数（默认 200）
      --points=<N>             每帧点数（默认 20000）
  -h, --help               显示本帮助

全局选项:
  --log-level=<级别>       trace|debug|info|warn|error（默认 info）
  --no-color               关闭彩色输出
)" << std::endl;
}

/// 提取全局选项并返回剩余参数
std::vector<std::string> parseGlobalOptions(const std::vector<std::string>& args) {
  std::vector<std::string> remaining;
  for (const std::string& arg : args) {
    if (arg.rfind("--log-level=", 0) == 0) {
      const std::string level = arg.substr(12);
      if (level == "trace") adsim::Logger::instance().setLevel(adsim::LogLevel::kTrace);
      else if (level == "debug") adsim::Logger::instance().setLevel(adsim::LogLevel::kDebug);
      else if (level == "info") adsim::Logger::instance().setLevel(adsim::LogLevel::kInfo);
      else if (level == "warn") adsim::Logger::instance().setLevel(adsim::LogLevel::kWarn);
      else if (level == "error") adsim::Logger::instance().setLevel(adsim::LogLevel::kError);
      else {
        std::cerr << "未知日志级别: " << level << std::endl;
        std::exit(1);
      }
    } else if (arg == "--no-color") {
      adsim::Logger::instance().setColorEnabled(false);
    } else {
      remaining.push_back(arg);
    }
  }
  return remaining;
}

}  // namespace

int main(int argc, char** argv) {
  std::vector<std::string> args(argv + 1, argv + argc);
  args = parseGlobalOptions(args);

  if (args.empty()) {
    printBanner();
    printUsage();
    return 0;
  }

  const std::string command = args.front();
  if (command == "-h" || command == "--help" || command == "help") {
    printBanner();
    printUsage();
    return 0;
  }

  const auto it = commands().find(command);
  if (it == commands().end()) {
    std::cerr << "未知命令: " << command << "\n" << std::endl;
    printUsage();
    return 1;
  }

  const std::vector<std::string> command_args(args.begin() + 1, args.end());

  try {
    return it->second(command_args);
  } catch (const std::exception& e) {
    std::cerr << "执行失败: " << e.what() << std::endl;
    return 1;
  }
}
