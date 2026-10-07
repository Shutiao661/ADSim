#include "TestFramework.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iomanip>

namespace adsim {
namespace test {

std::vector<TestCase>& registry() {
  static std::vector<TestCase> cases;
  return cases;
}

void fail(const char* file, int line, const std::string& message) {
  std::ostringstream oss;
  oss << file << ':' << line << "  " << message;
  throw AssertionFailure(oss.str());
}

namespace {

const char* kColorRed = "\033[31m";
const char* kColorGreen = "\033[32m";
const char* kColorYellow = "\033[33m";
const char* kColorCyan = "\033[36m";
const char* kColorDim = "\033[90m";
const char* kColorReset = "\033[0m";

}  // namespace

int runAll(int argc, char** argv) {
  const char* filter = nullptr;
  bool verbose = false;

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "-v") == 0 || std::strcmp(argv[i], "--verbose") == 0) {
      verbose = true;
    } else if (std::strncmp(argv[i], "--filter=", 9) == 0) {
      filter = argv[i] + 9;
    } else if (argv[i][0] != '-') {
      filter = argv[i];
    }
  }

  std::vector<TestCase> cases = registry();
  std::sort(cases.begin(), cases.end(), [](const TestCase& a, const TestCase& b) {
    return a.suite == b.suite ? a.name < b.name : a.suite < b.suite;
  });

  std::size_t passed = 0;
  std::size_t failed = 0;
  std::size_t skipped = 0;
  std::string current_suite;
  std::vector<std::string> failures;

  const auto wall_start = std::chrono::steady_clock::now();

  for (const TestCase& tc : cases) {
    // 过滤器匹配：suite 名或 suite.name 的子串
    if (filter != nullptr) {
      const std::string full = tc.suite + "." + tc.name;
      if (full.find(filter) == std::string::npos) {
        ++skipped;
        continue;
      }
    }

    if (tc.suite != current_suite) {
      current_suite = tc.suite;
      std::cout << "\n" << kColorCyan << "[" << current_suite << "]" << kColorReset
                << std::endl;
    }

    const auto start = std::chrono::steady_clock::now();
    bool ok = true;
    std::string error;

    try {
      tc.fn();
    } catch (const AssertionFailure& e) {
      ok = false;
      error = e.what();
    } catch (const std::exception& e) {
      ok = false;
      error = std::string("未捕获异常: ") + e.what();
    } catch (...) {
      ok = false;
      error = "未捕获的未知异常";
    }

    const double ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - start)
                          .count();

    if (ok) {
      ++passed;
      if (verbose) {
        std::cout << "  " << kColorGreen << "✓" << kColorReset << " " << tc.name
                  << kColorDim << "  (" << std::fixed << std::setprecision(1) << ms
                  << " ms)" << kColorReset << std::endl;
      } else {
        std::cout << "  " << kColorGreen << "✓" << kColorReset << " " << tc.name
                  << std::endl;
      }
    } else {
      ++failed;
      std::cout << "  " << kColorRed << "✗ " << tc.name << kColorReset << std::endl;
      std::cout << "      " << kColorRed << error << kColorReset << std::endl;
      failures.push_back(tc.suite + "." + tc.name + "\n      " + error);
    }
  }

  const double total_ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - wall_start)
                              .count();

  std::cout << "\n======================================================" << std::endl;
  std::cout << "  用例总数 : " << (passed + failed) << std::endl;
  std::cout << "  通过     : " << kColorGreen << passed << kColorReset << std::endl;
  if (failed > 0) {
    std::cout << "  失败     : " << kColorRed << failed << kColorReset << std::endl;
  } else {
    std::cout << "  失败     : 0" << std::endl;
  }
  if (skipped > 0) {
    std::cout << "  已跳过   : " << skipped << kColorYellow << " (被过滤器排除)"
              << kColorReset << std::endl;
  }
  std::cout << "  总耗时   : " << std::fixed << std::setprecision(1) << total_ms
            << " ms" << std::endl;
  std::cout << "======================================================" << std::endl;

  if (!failures.empty()) {
    std::cout << "\n失败详情:" << std::endl;
    for (const std::string& f : failures) {
      std::cout << "  " << kColorRed << "• " << f << kColorReset << std::endl;
    }
  }

  return static_cast<int>(failed);
}

}  // namespace test
}  // namespace adsim

int main(int argc, char** argv) {
  std::cout << "ADSim 单元测试" << std::endl;
  return adsim::test::runAll(argc, argv);
}
