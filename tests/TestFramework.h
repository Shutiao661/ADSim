// =============================================================================
//  TestFramework.h — 零依赖轻量单元测试框架
//
//  不引入 googletest 等外部依赖，保证工程在离线环境下依然可完整构建与验证。
//  支持：用例注册、断言、异常捕获、按名称过滤、耗时统计。
// =============================================================================
#pragma once

#include <cmath>
#include <exception>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace adsim {
namespace test {

struct TestCase {
  std::string suite;
  std::string name;
  std::function<void()> fn;
};

std::vector<TestCase>& registry();

struct Registrar {
  Registrar(const char* suite, const char* name, std::function<void()> fn) {
    registry().push_back(TestCase{suite, name, std::move(fn)});
  }
};

/// 断言失败异常，由测试运行器捕获并计为失败
class AssertionFailure : public std::exception {
 public:
  explicit AssertionFailure(std::string message) : message_(std::move(message)) {}
  const char* what() const noexcept override { return message_.c_str(); }

 private:
  std::string message_;
};

[[noreturn]] void fail(const char* file, int line, const std::string& message);

/// 运行全部（或按过滤器筛选的）用例，返回失败数量
int runAll(int argc, char** argv);

}  // namespace test
}  // namespace adsim

// ---------------------------------------------------------------------------
// 用例注册宏
// ---------------------------------------------------------------------------
#define ADSIM_TEST(suite, name)                                                  \
  static void adsim_test_##suite##_##name();                                     \
  static ::adsim::test::Registrar adsim_reg_##suite##_##name(                    \
      #suite, #name, adsim_test_##suite##_##name);                               \
  static void adsim_test_##suite##_##name()

// ---------------------------------------------------------------------------
// 断言宏
// ---------------------------------------------------------------------------
#define ADSIM_CHECK(cond)                                                        \
  do {                                                                           \
    if (!(cond)) {                                                               \
      ::adsim::test::fail(__FILE__, __LINE__, "断言失败: " #cond);               \
    }                                                                            \
  } while (false)

#define ADSIM_CHECK_MSG(cond, msg)                                               \
  do {                                                                           \
    if (!(cond)) {                                                               \
      std::ostringstream _oss;                                                   \
      _oss << "断言失败: " #cond << " | " << msg;                                \
      ::adsim::test::fail(__FILE__, __LINE__, _oss.str());                       \
    }                                                                            \
  } while (false)

#define ADSIM_CHECK_EQ(a, b)                                                     \
  do {                                                                           \
    const auto& _va = (a);                                                       \
    const auto& _vb = (b);                                                       \
    if (!(_va == _vb)) {                                                         \
      std::ostringstream _oss;                                                   \
      _oss << "期望 " #a " == " #b "，实际为 " << _va << " vs " << _vb;          \
      ::adsim::test::fail(__FILE__, __LINE__, _oss.str());                       \
    }                                                                            \
  } while (false)

#define ADSIM_CHECK_NEAR(a, b, eps)                                              \
  do {                                                                           \
    const double _va = static_cast<double>(a);                                   \
    const double _vb = static_cast<double>(b);                                   \
    const double _ve = static_cast<double>(eps);                                 \
    if (!(std::fabs(_va - _vb) <= _ve)) {                                        \
      std::ostringstream _oss;                                                   \
      _oss << "期望 " #a " ≈ " #b " (容差 " << _ve << ")，实际为 " << _va        \
           << " vs " << _vb << "，偏差 " << std::fabs(_va - _vb);                \
      ::adsim::test::fail(__FILE__, __LINE__, _oss.str());                       \
    }                                                                            \
  } while (false)

#define ADSIM_CHECK_GT(a, b)                                                     \
  do {                                                                           \
    const auto& _va = (a);                                                       \
    const auto& _vb = (b);                                                       \
    if (!(_va > _vb)) {                                                          \
      std::ostringstream _oss;                                                   \
      _oss << "期望 " #a " > " #b "，实际为 " << _va << " vs " << _vb;           \
      ::adsim::test::fail(__FILE__, __LINE__, _oss.str());                       \
    }                                                                            \
  } while (false)

#define ADSIM_CHECK_LT(a, b)                                                     \
  do {                                                                           \
    const auto& _va = (a);                                                       \
    const auto& _vb = (b);                                                       \
    if (!(_va < _vb)) {                                                          \
      std::ostringstream _oss;                                                   \
      _oss << "期望 " #a " < " #b "，实际为 " << _va << " vs " << _vb;           \
      ::adsim::test::fail(__FILE__, __LINE__, _oss.str());                       \
    }                                                                            \
  } while (false)

#define ADSIM_CHECK_GE(a, b)                                                     \
  do {                                                                           \
    const auto& _va = (a);                                                       \
    const auto& _vb = (b);                                                       \
    if (!(_va >= _vb)) {                                                         \
      std::ostringstream _oss;                                                   \
      _oss << "期望 " #a " >= " #b "，实际为 " << _va << " vs " << _vb;          \
      ::adsim::test::fail(__FILE__, __LINE__, _oss.str());                       \
    }                                                                            \
  } while (false)

#define ADSIM_CHECK_LE(a, b)                                                     \
  do {                                                                           \
    const auto& _va = (a);                                                       \
    const auto& _vb = (b);                                                       \
    if (!(_va <= _vb)) {                                                         \
      std::ostringstream _oss;                                                   \
      _oss << "期望 " #a " <= " #b "，实际为 " << _va << " vs " << _vb;          \
      ::adsim::test::fail(__FILE__, __LINE__, _oss.str());                       \
    }                                                                            \
  } while (false)

#define ADSIM_CHECK_THROWS(stmt, exception_type)                                 \
  do {                                                                           \
    bool _thrown = false;                                                        \
    try {                                                                        \
      stmt;                                                                      \
    } catch (const exception_type&) {                                            \
      _thrown = true;                                                            \
    } catch (...) {                                                              \
      ::adsim::test::fail(__FILE__, __LINE__,                                    \
                          "抛出了非预期异常类型，期望 " #exception_type);        \
    }                                                                            \
    if (!_thrown) {                                                              \
      ::adsim::test::fail(__FILE__, __LINE__, "未抛出预期异常 " #exception_type); \
    }                                                                            \
  } while (false)
