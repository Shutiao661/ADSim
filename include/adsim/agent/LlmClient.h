// =============================================================================
//  LlmClient.h — 大语言模型客户端抽象
//
//  用途：对路测/仿真中复杂的交互场景（无保护左转、加塞、博弈让行）做自然语言
//  打标与归类，帮助策略工程师快速从海量数据中筛出高价值片段。
//
//  分两层：
//    * LlmClient        —— 抽象接口。调用方只依赖它，便于替换与测试。
//    * HttpLlmClient    —— 基于 libcurl 的真实实现（可选依赖 ADSIM_WITH_CURL）
//    * MockLlmClient    —— 确定性桩实现，离线可用，使上层逻辑与测试不依赖网络
//
//  设计取舍：把网络调用放在接口之后，是为了让打标流程可以完全离线跑通——
//  在 CI 与无网环境中依然能回归测试，同时保留了接入真实模型的通路。
// =============================================================================
#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace adsim {

/// 对话消息
struct LlmMessage {
  enum class Role { kSystem, kUser, kAssistant };
  Role role{Role::kUser};
  std::string content;

  static LlmMessage system(const std::string& text);
  static LlmMessage user(const std::string& text);
  static LlmMessage assistant(const std::string& text);
};

struct LlmRequest {
  std::vector<LlmMessage> messages;
  double temperature{0.2};      ///< 打标任务需要稳定输出，温度取低值
  int max_tokens{1024};
  /// 要求模型输出 JSON 时的提示（部分服务商支持强制 JSON 模式）
  bool json_mode{false};
  double timeout_seconds{30.0};
};

struct LlmResponse {
  bool ok{false};
  std::string text;
  std::string error;

  std::string model;
  int prompt_tokens{0};
  int completion_tokens{0};
  double latency_ms{0.0};
  bool from_cache{false};

  int totalTokens() const { return prompt_tokens + completion_tokens; }
};

/// 模型客户端接口
class LlmClient {
 public:
  virtual ~LlmClient() = default;

  virtual LlmResponse complete(const LlmRequest& request) = 0;

  /// 客户端标识，用于报告与日志
  virtual std::string name() const = 0;

  /// 是否可用（如网络可达、API Key 已配置）
  virtual bool isAvailable() const { return true; }
};

// ---------------------------------------------------------------------------
// 离线桩实现
//
//  不做任何网络调用，而是按关键词规则给出确定性回答。它的价值不在于"像 LLM"，
//  而在于让打标流程、报告生成、单元测试在无网络环境下完整可跑。
// ---------------------------------------------------------------------------
class MockLlmClient : public LlmClient {
 public:
  MockLlmClient() = default;

  LlmResponse complete(const LlmRequest& request) override;
  std::string name() const override { return "mock"; }

  /// 人为设置延迟，用于测试超时处理
  void setSimulatedLatency(double milliseconds) { latency_ms_ = milliseconds; }
  /// 人为制造失败，用于测试错误分支
  void setFailNext(bool fail) { fail_next_ = fail; }

  std::size_t callCount() const { return call_count_; }

 private:
  double latency_ms_{0.0};
  bool fail_next_{false};
  std::size_t call_count_{0};
};

// ---------------------------------------------------------------------------
// 真实 HTTP 实现（libcurl）
//
//  仅当以 ADSIM_WITH_CURL=ON 构建时可用；否则构造时抛出 std::runtime_error，
//  由调用方回退到 MockLlmClient。
// ---------------------------------------------------------------------------
struct HttpLlmConfig {
  std::string endpoint{"https://api.anthropic.com/v1/messages"};
  std::string api_key;                       ///< 为空时读取环境变量 ANTHROPIC_API_KEY
  std::string model{"claude-sonnet-5"};
  int max_retries{3};
  double connect_timeout_seconds{5.0};
  double request_timeout_seconds{60.0};
  /// 简单的内存响应缓存：相同请求不重复计费
  bool enable_cache{true};
  std::size_t cache_capacity{128};
};

class HttpLlmClient : public LlmClient {
 public:
  explicit HttpLlmClient(const HttpLlmConfig& config);
  ~HttpLlmClient() override;

  LlmResponse complete(const LlmRequest& request) override;
  std::string name() const override { return "http"; }
  bool isAvailable() const override;

  /// 当前构建是否包含 libcurl 支持
  static bool compiledIn();

  void clearCache();
  std::size_t cacheHits() const { return cache_hits_; }

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::size_t cache_hits_{0};
};

}  // namespace adsim
