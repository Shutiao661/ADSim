// =============================================================================
//  ScenarioTagger.h — 交互场景自然语言打标
//
//  解决的问题：一次路测产生数百 GB 数据，其中真正有分析价值的是极少数
//  复杂交互片段（无保护左转、加塞、鬼探头……）。人工逐段回看不可行。
//
//  做法：
//    1. **特征提取**：从对齐后的数据中抽取可量化的交互特征（相对距离、
//       相对速度、TTC、让行关系、轨迹交叠等）——这一步是纯数值计算，
//       保证结果客观、可复现、可解释。
//    2. **规则打标**：基于特征做确定性分类，作为一个始终可用的基线。
//    3. **LLM 增强**：把特征摘要交给大模型，产出自然语言描述与分类，
//       用于捕捉规则难以覆盖的语义（"这是一次被迫让行" vs "主动让行"）。
//
//  规则与 LLM 并行存在而非互相替代：规则保证底线与可复现性，
//  LLM 提供规则表达不了的语义层次。
// =============================================================================
#pragma once

#include "adsim/agent/LlmClient.h"
#include "adsim/common/Types.h"
#include "adsim/datapipeline/TimeAligner.h"
#include "adsim/sim/SimEngine.h"

#include <memory>
#include <string>
#include <vector>

namespace adsim {

/// 交互场景类别
enum class InteractionCategory {
  kUnknown,
  kCarFollowing,        ///< 常规跟车
  kCutIn,               ///< 旁车加塞
  kCutOut,              ///< 旁车切出
  kLeadBraking,         ///< 前车急刹
  kUnprotectedLeftTurn, ///< 无保护左转
  kUnprotectedRightTurn,
  kPedestrianCrossing,  ///< 行人横穿
  kIntersectionYield,   ///< 路口让行
  kLaneChange,          ///< 自车换道
  kOvertaking,          ///< 超车
  kEmergencyBraking,    ///< 紧急制动
  kFreeCruising,        ///< 自由巡航（无交互）
};

/// 人类可读的类别名（中文），用于报告与日志展示
const char* toString(InteractionCategory category);

/// 机器可读的类别名（英文，如 "cut_in"），用于 JSON 序列化与 LLM 交互。
/// 与 toString 的区别是刻意保留的：展示给人看要中文，进出机器要稳定标识符。
const char* toMachineName(InteractionCategory category);

/// 解析机器名。与 toMachineName 构成往返，无法识别时返回 kUnknown。
InteractionCategory categoryFromString(const std::string& name);

/// 从数据中抽取的可量化交互特征
struct InteractionFeatures {
  double duration{0.0};           ///< 片段时长 (s)
  double ego_distance{0.0};       ///< 自车行驶距离 (m)
  double ego_mean_speed{0.0};
  double ego_max_speed{0.0};
  double ego_min_speed{0.0};
  double ego_max_deceleration{0.0};
  double ego_max_lateral_acceleration{0.0};
  double max_curvature{0.0};

  double min_ttc{1e9};            ///< 最小碰撞时间 (s)
  double min_gap{1e9};            ///< 最近距离 (m)
  double min_headway{1e9};        ///< 最小车头时距 (s)
  double closing_speed{0.0};      ///< 最大接近速度 (m/s)

  int object_count{0};            ///< 交互对象数量
  bool has_collision{false};
  bool has_off_road{false};
  bool lane_change_detected{false};
  double max_lateral_offset{0.0}; ///< 相对车道中心的最大横向偏移 (m)

  /// 是否为危险工况——用于筛出高价值片段
  bool isCritical() const {
    return has_collision || min_ttc < 2.0 || min_gap < 2.0 ||
           ego_max_deceleration < -4.0;
  }

  /// 转成给 LLM 的自然语言摘要
  std::string toPromptText() const;

  /// 转成单行可读摘要
  std::string toString() const;
};

/// 打标结果
struct TaggingResult {
  InteractionCategory primary_category{InteractionCategory::kUnknown};
  std::vector<InteractionCategory> secondary_categories;
  double confidence{0.0};        ///< 置信度 [0,1]
  std::string rationale;         ///< 判定依据
  std::string description;       ///< 自然语言描述（可能来自 LLM）
  bool from_llm{false};
  bool critical{false};
  InteractionFeatures features;

  std::string toString() const;
};

class ScenarioTagger {
 public:
  struct Config {
    /// 最小 TTC 低于此值判为危险
    double critical_ttc{2.0};
    /// 最小间距低于此值判为危险
    double critical_gap{2.0};
    /// 触发加塞判定的横向侵入阈值 (m)
    double cut_in_lateral_threshold{1.5};
    /// 触发急刹判定的减速度阈值 (m/s²)
    double harsh_braking_threshold{-4.0};
    /// 是否启用 LLM 增强（未提供客户端时自动降级）
    bool enable_llm{true};
  };

  ScenarioTagger();
  explicit ScenarioTagger(const Config& config);

  /// 注入 LLM 客户端；传 nullptr 表示纯规则模式
  void setLlmClient(std::shared_ptr<LlmClient> client);
  LlmClient* llmClient() const { return llm_client_.get(); }

  // -------------------------------------------------------------------------
  // 特征提取
  // -------------------------------------------------------------------------

  /// 从仿真结果提取交互特征
  static InteractionFeatures extractFeatures(const SimulationResult& result);

  /// 从对齐后的路测数据提取交互特征
  static InteractionFeatures extractFeatures(const std::vector<AlignedFrame>& frames);

  // -------------------------------------------------------------------------
  // 打标
  // -------------------------------------------------------------------------

  /// 基于规则打标（确定性，始终可用）
  TaggingResult tagByRules(const InteractionFeatures& features) const;

  /// 完整打标：规则 + 可选 LLM 增强
  TaggingResult tag(const SimulationResult& result);
  TaggingResult tag(const std::vector<AlignedFrame>& frames);

  /// 批量打标并按价值排序，返回最值得人工复核的片段
  std::vector<TaggingResult> tagBatch(const std::vector<SimulationResult>& results);

  /// 构造交给 LLM 的提示词（公开以便测试与人工复核提示词质量）
  LlmRequest buildRequest(const InteractionFeatures& features) const;

  /// 解析 LLM 返回的 JSON；解析失败时返回 false 并保持原结果不变
  static bool parseLlmResponse(const std::string& text, TaggingResult& result);

  const Config& config() const { return config_; }

 private:
  Config config_;
  std::shared_ptr<LlmClient> llm_client_;
};

}  // namespace adsim
