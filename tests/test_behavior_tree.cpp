// =============================================================================
//  test_behavior_tree.cpp — 行为树单元测试
//  覆盖：黑板 / 组合节点三态语义（含断点续跑）/ 装饰节点 / 叶子节点 /
//        整树行为 / 一个贴近真实决策的场景
// =============================================================================
#include "TestFramework.h"

#include "adsim/planning/BehaviorTree.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace adsim;

namespace {

constexpr NodeStatus kOk = NodeStatus::kSuccess;
constexpr NodeStatus kFail = NodeStatus::kFailure;
constexpr NodeStatus kRun = NodeStatus::kRunning;

/// 状态断言：NodeStatus 没有 operator<<，断言宏在失败路径上需要把值写进
/// ostringstream，因此统一先经 toString() 转成字符串再比较——
/// 顺带把三态命名的转换也纳入验证。
#define CHECK_STATUS(actual, expected)                                        \
  ADSIM_CHECK_EQ(std::string(::adsim::toString(actual)),                      \
                 std::string(::adsim::toString(expected)))

/// 脚本化叶子节点：按预设脚本依次返回状态，脚本用尽后一直重复最后一个状态。
/// 有了它，"子节点何时成功 / 失败 / 运行中"完全由测试掌控，
/// 组合节点的断点位置与执行次数才能被精确验证。
class ScriptedNode : public Node {
 public:
  ScriptedNode(std::string name, std::vector<NodeStatus> script)
      : Node(std::move(name)), script_(std::move(script)) {}

  NodeStatus tick() override {
    beginTick();
    const NodeStatus status = next();
    endTick();
    recordTick(status);
    return status;
  }

  void halt() override {
    Node::halt();
    ++halt_count_;  // 记录中断次数，用于验证 halt() 是否真的传播下来
  }

  void reset() override {
    Node::reset();
    index_ = 0;
  }

  std::size_t haltCount() const { return halt_count_; }

 private:
  NodeStatus next() {
    if (script_.empty()) return NodeStatus::kFailure;
    if (index_ >= script_.size()) return script_.back();
    return script_[index_++];
  }

  std::vector<NodeStatus> script_;
  std::size_t index_{0};
  std::size_t halt_count_{0};
};

/// 造一个脚本化节点并同时留下裸指针（所有权随后交给父节点，
/// 但断言仍需观察子节点的 tick / halt 次数）
std::unique_ptr<ScriptedNode> makeScripted(const std::string& name,
                                          std::vector<NodeStatus> script) {
  return std::unique_ptr<ScriptedNode>(
      new ScriptedNode(name, std::move(script)));
}

constexpr double kEps = 1e-12;

// ===========================================================================
//  顺序节点
// ===========================================================================

ADSIM_TEST(BehaviorTree, Sequence_一次tick内依次执行全部子节点) {
  auto a = makeScripted("A", {kOk});
  auto b = makeScripted("B", {kOk});
  auto c = makeScripted("C", {kOk});
  ScriptedNode* a_ptr = a.get();
  ScriptedNode* b_ptr = b.get();
  ScriptedNode* c_ptr = c.get();

  Sequence seq("seq");
  seq.addChild(std::move(a));
  seq.addChild(std::move(b));
  seq.addChild(std::move(c));

  ADSIM_CHECK_EQ(seq.children().size(), std::size_t(3));
  CHECK_STATUS(seq.tick(), kOk);

  // 子节点成功不能只推进一个：同一次 tick 内要一路走到底
  ADSIM_CHECK_EQ(a_ptr->tickCount(), std::size_t(1));
  ADSIM_CHECK_EQ(b_ptr->tickCount(), std::size_t(1));
  ADSIM_CHECK_EQ(c_ptr->tickCount(), std::size_t(1));
  ADSIM_CHECK_EQ(seq.currentIndex(), std::size_t(0));  // 完成后下标归零
}

ADSIM_TEST(BehaviorTree, Sequence_中途运行中时下标不前移) {
  auto a = makeScripted("A", {kOk});
  auto b = makeScripted("B", {kRun, kRun, kOk});
  auto c = makeScripted("C", {kOk});
  ScriptedNode* a_ptr = a.get();
  ScriptedNode* b_ptr = b.get();
  ScriptedNode* c_ptr = c.get();

  Sequence seq("seq");
  seq.addChild(std::move(a));
  seq.addChild(std::move(b));
  seq.addChild(std::move(c));

  CHECK_STATUS(seq.tick(), kRun);
  ADSIM_CHECK_EQ(seq.currentIndex(), std::size_t(1));  // 停在 B 上
  ADSIM_CHECK_EQ(a_ptr->tickCount(), std::size_t(1));
  ADSIM_CHECK_EQ(b_ptr->tickCount(), std::size_t(1));
  ADSIM_CHECK_EQ(c_ptr->tickCount(), std::size_t(0));

  // 第二次 tick 必须继续问同一个子节点，而不是重跑 A
  CHECK_STATUS(seq.tick(), kRun);
  ADSIM_CHECK_EQ(seq.currentIndex(), std::size_t(1));
  ADSIM_CHECK_EQ(a_ptr->tickCount(), std::size_t(1));  // 关键：A 不被重复执行
  ADSIM_CHECK_EQ(b_ptr->tickCount(), std::size_t(2));
  ADSIM_CHECK_EQ(c_ptr->tickCount(), std::size_t(0));
}

ADSIM_TEST(BehaviorTree, Sequence_断点恢复后继续后续子节点) {
  auto a = makeScripted("A", {kOk});
  auto b = makeScripted("B", {kRun, kOk});
  auto c = makeScripted("C", {kOk});
  ScriptedNode* a_ptr = a.get();
  ScriptedNode* b_ptr = b.get();
  ScriptedNode* c_ptr = c.get();

  Sequence seq("seq");
  seq.addChild(std::move(a));
  seq.addChild(std::move(b));
  seq.addChild(std::move(c));

  CHECK_STATUS(seq.tick(), kRun);  // A 成功，B 挂起

  // B 在本轮完成 → 同一次 tick 内继续执行 C → 整条序列成功
  CHECK_STATUS(seq.tick(), kOk);
  ADSIM_CHECK_EQ(seq.currentIndex(), std::size_t(0));
  ADSIM_CHECK_EQ(a_ptr->tickCount(), std::size_t(1));  // 全程只执行过一次
  ADSIM_CHECK_EQ(b_ptr->tickCount(), std::size_t(2));
  ADSIM_CHECK_EQ(c_ptr->tickCount(), std::size_t(1));
}

ADSIM_TEST(BehaviorTree, Sequence_中途失败即整体重置) {
  auto a = makeScripted("A", {kOk});
  auto b = makeScripted("B", {kFail});
  auto c = makeScripted("C", {kOk});
  ScriptedNode* a_ptr = a.get();
  ScriptedNode* b_ptr = b.get();
  ScriptedNode* c_ptr = c.get();

  Sequence seq("seq");
  seq.addChild(std::move(a));
  seq.addChild(std::move(b));
  seq.addChild(std::move(c));

  CHECK_STATUS(seq.tick(), kFail);
  ADSIM_CHECK_EQ(seq.currentIndex(), std::size_t(0));  // 失败后下标归零
  ADSIM_CHECK_EQ(a_ptr->tickCount(), std::size_t(1));
  ADSIM_CHECK_EQ(b_ptr->tickCount(), std::size_t(1));
  ADSIM_CHECK_EQ(c_ptr->tickCount(), std::size_t(0));  // 失败即止，不碰后续分支

  // 再 tick 一次：整条序列从头开始，而不是从 B 继续
  CHECK_STATUS(seq.tick(), kFail);
  ADSIM_CHECK_EQ(a_ptr->tickCount(), std::size_t(2));
  ADSIM_CHECK_EQ(b_ptr->tickCount(), std::size_t(2));
}

ADSIM_TEST(BehaviorTree, Sequence_运行中转为失败同样整体重置) {
  auto a = makeScripted("A", {kOk});
  auto b = makeScripted("B", {kRun, kFail});
  auto c = makeScripted("C", {kOk});
  ScriptedNode* b_ptr = b.get();
  ScriptedNode* c_ptr = c.get();

  Sequence seq("seq");
  seq.addChild(std::move(a));
  seq.addChild(std::move(b));
  seq.addChild(std::move(c));

  CHECK_STATUS(seq.tick(), kRun);
  ADSIM_CHECK_EQ(seq.currentIndex(), std::size_t(1));

  // 挂起的分支最终失败：同样是整体失败 + 重置
  CHECK_STATUS(seq.tick(), kFail);
  ADSIM_CHECK_EQ(seq.currentIndex(), std::size_t(0));
  ADSIM_CHECK_EQ(b_ptr->tickCount(), std::size_t(2));
  ADSIM_CHECK_EQ(c_ptr->tickCount(), std::size_t(0));
}

ADSIM_TEST(BehaviorTree, Sequence_空子节点列表视为成功) {
  Sequence seq("empty");
  // 空序列是可复用构件（例如条件全部满足），不应崩溃
  CHECK_STATUS(seq.tick(), kOk);
  ADSIM_CHECK_EQ(seq.tickCount(), std::size_t(1));
}

// ===========================================================================
//  选择节点
// ===========================================================================

ADSIM_TEST(BehaviorTree, Selector_失败后换下一个分支) {
  auto a = makeScripted("A", {kFail});
  auto b = makeScripted("B", {kOk});
  auto c = makeScripted("C", {kOk});
  ScriptedNode* a_ptr = a.get();
  ScriptedNode* b_ptr = b.get();
  ScriptedNode* c_ptr = c.get();

  Selector sel("sel");
  sel.addChild(std::move(a));
  sel.addChild(std::move(b));
  sel.addChild(std::move(c));

  CHECK_STATUS(sel.tick(), kOk);
  ADSIM_CHECK_EQ(a_ptr->tickCount(), std::size_t(1));
  ADSIM_CHECK_EQ(b_ptr->tickCount(), std::size_t(1));
  ADSIM_CHECK_EQ(c_ptr->tickCount(), std::size_t(0));  // 已成功，不再尝试后续分支
  ADSIM_CHECK_EQ(sel.currentIndex(), std::size_t(0));
}

ADSIM_TEST(BehaviorTree, Selector_运行中时下标不动) {
  auto a = makeScripted("A", {kFail});
  auto b = makeScripted("B", {kRun, kRun, kOk});
  auto c = makeScripted("C", {kOk});
  ScriptedNode* a_ptr = a.get();
  ScriptedNode* b_ptr = b.get();
  ScriptedNode* c_ptr = c.get();

  Selector sel("sel");
  sel.addChild(std::move(a));
  sel.addChild(std::move(b));
  sel.addChild(std::move(c));

  CHECK_STATUS(sel.tick(), kRun);
  ADSIM_CHECK_EQ(sel.currentIndex(), std::size_t(1));  // 停在第 2 个分支
  ADSIM_CHECK_EQ(a_ptr->tickCount(), std::size_t(1));

  CHECK_STATUS(sel.tick(), kRun);
  ADSIM_CHECK_EQ(sel.currentIndex(), std::size_t(1));
  ADSIM_CHECK_EQ(a_ptr->tickCount(), std::size_t(1));  // 不回头重试已失败的分支
  ADSIM_CHECK_EQ(b_ptr->tickCount(), std::size_t(2));

  // 挂起的分支最终成功 → 本次决策完成
  CHECK_STATUS(sel.tick(), kOk);
  ADSIM_CHECK_EQ(sel.currentIndex(), std::size_t(0));
  ADSIM_CHECK_EQ(c_ptr->tickCount(), std::size_t(0));
}

ADSIM_TEST(BehaviorTree, Selector_运行中转为失败换下一分支) {
  auto a = makeScripted("A", {kRun, kFail});
  auto b = makeScripted("B", {kOk});
  ScriptedNode* a_ptr = a.get();
  ScriptedNode* b_ptr = b.get();

  Selector sel("sel");
  sel.addChild(std::move(a));
  sel.addChild(std::move(b));

  CHECK_STATUS(sel.tick(), kRun);
  // A 返回 Running：断点指向 A，下个周期先继续问它（而不是直接跳到 B）
  ADSIM_CHECK_EQ(sel.currentIndex(), std::size_t(0));

  // A 最终失败 → 立刻改用 B
  CHECK_STATUS(sel.tick(), kOk);
  ADSIM_CHECK_EQ(a_ptr->tickCount(), std::size_t(2));
  ADSIM_CHECK_EQ(b_ptr->tickCount(), std::size_t(1));
  ADSIM_CHECK_EQ(sel.currentIndex(), std::size_t(0));
}

ADSIM_TEST(BehaviorTree, Selector_全部分支失败返回失败并重置) {
  auto a = makeScripted("A", {kFail});
  auto b = makeScripted("B", {kFail});
  ScriptedNode* a_ptr = a.get();
  ScriptedNode* b_ptr = b.get();

  Selector sel("sel");
  sel.addChild(std::move(a));
  sel.addChild(std::move(b));

  CHECK_STATUS(sel.tick(), kFail);
  ADSIM_CHECK_EQ(sel.currentIndex(), std::size_t(0));
  ADSIM_CHECK_EQ(a_ptr->tickCount(), std::size_t(1));
  ADSIM_CHECK_EQ(b_ptr->tickCount(), std::size_t(1));

  // 重置后每个分支都还有机会重试
  CHECK_STATUS(sel.tick(), kFail);
  ADSIM_CHECK_EQ(a_ptr->tickCount(), std::size_t(2));
  ADSIM_CHECK_EQ(b_ptr->tickCount(), std::size_t(2));
}

// ===========================================================================
//  并行节点
// ===========================================================================

ADSIM_TEST(BehaviorTree, Parallel_全部成功策略) {
  auto a = makeScripted("A", {kOk});
  auto b = makeScripted("B", {kOk});
  ScriptedNode* a_ptr = a.get();
  ScriptedNode* b_ptr = b.get();

  Parallel par("par", Parallel::Policy::kRequireAll, 1);
  par.addChild(std::move(a));
  par.addChild(std::move(b));

  CHECK_STATUS(par.tick(), kOk);
  ADSIM_CHECK_EQ(a_ptr->tickCount(), std::size_t(1));
  ADSIM_CHECK_EQ(b_ptr->tickCount(), std::size_t(1));

  // 终态后内部计数与完成标记要清零，否则下一轮无法重新执行
  CHECK_STATUS(par.tick(), kOk);
  ADSIM_CHECK_EQ(a_ptr->tickCount(), std::size_t(2));
  ADSIM_CHECK_EQ(b_ptr->tickCount(), std::size_t(2));
}

ADSIM_TEST(BehaviorTree, Parallel_已完成子节点不再重复执行) {
  auto a = makeScripted("A", {kOk});
  auto b = makeScripted("B", {kRun, kRun, kOk});
  auto c = makeScripted("C", {kRun, kOk});
  ScriptedNode* a_ptr = a.get();
  ScriptedNode* b_ptr = b.get();
  ScriptedNode* c_ptr = c.get();

  Parallel par("par", Parallel::Policy::kRequireAll, 5);
  par.addChild(std::move(a));
  par.addChild(std::move(b));
  par.addChild(std::move(c));

  CHECK_STATUS(par.tick(), kRun);
  ADSIM_CHECK_EQ(a_ptr->tickCount(), std::size_t(1));
  ADSIM_CHECK_EQ(b_ptr->tickCount(), std::size_t(1));
  ADSIM_CHECK_EQ(c_ptr->tickCount(), std::size_t(1));

  // A 已完成：后续 tick 里不得再被执行（完成标记生效）
  CHECK_STATUS(par.tick(), kRun);
  ADSIM_CHECK_EQ(a_ptr->tickCount(), std::size_t(1));
  ADSIM_CHECK_EQ(b_ptr->tickCount(), std::size_t(2));
  ADSIM_CHECK_EQ(c_ptr->tickCount(), std::size_t(2));

  // 全部子节点跨周期完成后才返回成功（C 的脚本是 {Running, Success}，
  // 在第二次被 tick 时就已完成，之后同样不再执行）
  CHECK_STATUS(par.tick(), kOk);
  ADSIM_CHECK_EQ(a_ptr->tickCount(), std::size_t(1));
  ADSIM_CHECK_EQ(b_ptr->tickCount(), std::size_t(3));
  ADSIM_CHECK_EQ(c_ptr->tickCount(), std::size_t(2));
}

ADSIM_TEST(BehaviorTree, Parallel_失败数达到阈值即中止) {
  auto a = makeScripted("A", {kFail});
  auto b = makeScripted("B", {kFail});
  auto c = makeScripted("C", {kRun});
  ScriptedNode* c_ptr = c.get();

  Parallel par("par", Parallel::Policy::kRequireAll, 2);
  par.addChild(std::move(a));
  par.addChild(std::move(b));
  par.addChild(std::move(c));

  // 累计失败 2 次达到阈值：即使 C 仍在运行也判失败
  CHECK_STATUS(par.tick(), kFail);
  ADSIM_CHECK_EQ(c_ptr->tickCount(), std::size_t(1));

  // 计数已清零，重新来过
  CHECK_STATUS(par.tick(), kFail);
  ADSIM_CHECK_EQ(c_ptr->tickCount(), std::size_t(2));
}

ADSIM_TEST(BehaviorTree, Parallel_任一成功策略) {
  auto a = makeScripted("A", {kRun, kOk});
  auto b = makeScripted("B", {kFail});
  ScriptedNode* a_ptr = a.get();
  ScriptedNode* b_ptr = b.get();

  Parallel par("par", Parallel::Policy::kRequireOne, 2);
  par.addChild(std::move(a));
  par.addChild(std::move(b));

  // 尚无成功且失败数未达阈值 → 继续运行
  CHECK_STATUS(par.tick(), kRun);
  ADSIM_CHECK_EQ(a_ptr->tickCount(), std::size_t(1));
  ADSIM_CHECK_EQ(b_ptr->tickCount(), std::size_t(1));

  // A 成功即整体成功；已完成的 B 不再被 tick
  CHECK_STATUS(par.tick(), kOk);
  ADSIM_CHECK_EQ(a_ptr->tickCount(), std::size_t(2));
  ADSIM_CHECK_EQ(b_ptr->tickCount(), std::size_t(1));
}

ADSIM_TEST(BehaviorTree, Parallel_子节点均未完成时返回运行中) {
  auto a = makeScripted("A", {kRun});
  auto b = makeScripted("B", {kRun});

  Parallel par("par", Parallel::Policy::kRequireAll, 3);
  par.addChild(std::move(a));
  par.addChild(std::move(b));

  CHECK_STATUS(par.tick(), kRun);
  CHECK_STATUS(par.tick(), kRun);
}

ADSIM_TEST(BehaviorTree, Parallel_全部结束仍未成功时判失败) {
  // 全部子节点都已结束却没有凑够成功条件：再返回 Running 就会让整棵树永久卡死
  {
    auto a = makeScripted("A", {kOk});
    auto b = makeScripted("B", {kFail});
    Parallel par("par", Parallel::Policy::kRequireAll, 5);
    par.addChild(std::move(a));
    par.addChild(std::move(b));
    CHECK_STATUS(par.tick(), kFail);
  }
  {
    auto a = makeScripted("A", {kFail});
    auto b = makeScripted("B", {kRun, kFail});
    Parallel par("par", Parallel::Policy::kRequireOne, 5);
    par.addChild(std::move(a));
    par.addChild(std::move(b));
    CHECK_STATUS(par.tick(), kRun);   // B 还在跑
    CHECK_STATUS(par.tick(), kFail);  // B 也结束了，始终没有成功
  }
}

ADSIM_TEST(BehaviorTree, Parallel_空子节点列表) {
  Parallel require_all("all", Parallel::Policy::kRequireAll, 1);
  CHECK_STATUS(require_all.tick(), kOk);  // 空集合对"全部成功"是真空满足

  Parallel require_one("one", Parallel::Policy::kRequireOne, 1);
  CHECK_STATUS(require_one.tick(), kFail);  // 但不可能"任一成功"
}

ADSIM_TEST(BehaviorTree, Parallel_中断未完成的子节点) {
  auto a = makeScripted("A", {kRun});
  auto b = makeScripted("B", {kOk});
  ScriptedNode* a_ptr = a.get();
  ScriptedNode* b_ptr = b.get();

  Parallel par("par", Parallel::Policy::kRequireAll, 3);
  par.addChild(std::move(a));
  par.addChild(std::move(b));

  CHECK_STATUS(par.tick(), kRun);
  par.halt();
  ADSIM_CHECK_EQ(a_ptr->haltCount(), std::size_t(1));  // 未完成的被中断
  ADSIM_CHECK_EQ(b_ptr->haltCount(), std::size_t(0));  // 已完成的无需打扰
}

// ===========================================================================
//  装饰节点
// ===========================================================================

ADSIM_TEST(BehaviorTree, Inverter_三态语义) {
  {
    Inverter inv("inv", makeScripted("C", {kOk}));
    CHECK_STATUS(inv.tick(), kFail);  // 成功 → 失败
  }
  {
    Inverter inv("inv", makeScripted("C", {kFail}));
    CHECK_STATUS(inv.tick(), kOk);  // 失败 → 成功
  }
  {
    auto child = makeScripted("C", {kRun});
    ScriptedNode* child_ptr = child.get();
    Inverter inv("inv", std::move(child));
    CHECK_STATUS(inv.tick(), kRun);  // Running 是"正在执行"，不参与反转
    CHECK_STATUS(inv.tick(), kRun);
    ADSIM_CHECK_EQ(child_ptr->tickCount(), std::size_t(2));
  }
}

ADSIM_TEST(BehaviorTree, Inverter_中断传播到子节点) {
  auto child = makeScripted("C", {kRun});
  ScriptedNode* child_ptr = child.get();

  Inverter inv("inv", std::move(child));
  CHECK_STATUS(inv.tick(), kRun);
  inv.halt();
  ADSIM_CHECK_EQ(child_ptr->haltCount(), std::size_t(1));
}

ADSIM_TEST(BehaviorTree, Retry_失败后重试到上限) {
  auto child = makeScripted("C", {kFail});
  ScriptedNode* child_ptr = child.get();

  Retry retry("retry", std::move(child), 3);
  CHECK_STATUS(retry.tick(), kFail);
  ADSIM_CHECK_EQ(child_ptr->tickCount(), std::size_t(3));  // 恰好尝试 3 次

  // 额度用尽：不再执行子节点
  CHECK_STATUS(retry.tick(), kFail);
  ADSIM_CHECK_EQ(child_ptr->tickCount(), std::size_t(3));

  // reset 之后额度恢复
  retry.reset();
  CHECK_STATUS(retry.tick(), kFail);
  ADSIM_CHECK_EQ(child_ptr->tickCount(), std::size_t(6));
}

ADSIM_TEST(BehaviorTree, Retry_重试期间成功) {
  auto child = makeScripted("C", {kFail, kFail, kOk});
  ScriptedNode* child_ptr = child.get();

  Retry retry("retry", std::move(child), 3);
  // 上限内的重试在同一次 tick 内完成
  CHECK_STATUS(retry.tick(), kOk);
  ADSIM_CHECK_EQ(child_ptr->tickCount(), std::size_t(3));
  ADSIM_CHECK_EQ(retry.tickCount(), std::size_t(1));
}

ADSIM_TEST(BehaviorTree, Retry_上限为1时只尝试一次) {
  auto child = makeScripted("C", {kFail});
  ScriptedNode* child_ptr = child.get();

  Retry retry("retry", std::move(child), 1);
  CHECK_STATUS(retry.tick(), kFail);
  ADSIM_CHECK_EQ(child_ptr->tickCount(), std::size_t(1));
}

ADSIM_TEST(BehaviorTree, Retry_运行中不消耗尝试次数) {
  auto child = makeScripted("C", {kRun, kRun, kOk});
  ScriptedNode* child_ptr = child.get();

  // 上限取 1：若 Running 也算一次尝试，第二次 tick 就会被判失败
  Retry retry("retry", std::move(child), 1);
  CHECK_STATUS(retry.tick(), kRun);
  CHECK_STATUS(retry.tick(), kRun);
  CHECK_STATUS(retry.tick(), kOk);
  ADSIM_CHECK_EQ(child_ptr->tickCount(), std::size_t(3));
}

ADSIM_TEST(BehaviorTree, Retry_中断后额度重置) {
  auto child = makeScripted("C", {kFail});
  ScriptedNode* child_ptr = child.get();

  Retry retry("retry", std::move(child), 2);
  CHECK_STATUS(retry.tick(), kFail);
  ADSIM_CHECK_EQ(child_ptr->tickCount(), std::size_t(2));

  retry.halt();
  CHECK_STATUS(retry.tick(), kFail);  // 重新拿到额度，可以再试
  ADSIM_CHECK_EQ(child_ptr->tickCount(), std::size_t(4));
}

ADSIM_TEST(BehaviorTree, RepeatForever_成功转为运行中并反复执行) {
  auto child = makeScripted("C", {kOk});
  ScriptedNode* child_ptr = child.get();

  RepeatForever repeat("repeat", std::move(child));
  // 成功要变成 Running（"一直保持"），且子节点下轮要能重新执行
  CHECK_STATUS(repeat.tick(), kRun);
  CHECK_STATUS(repeat.tick(), kRun);
  CHECK_STATUS(repeat.tick(), kRun);
  ADSIM_CHECK_EQ(child_ptr->tickCount(), std::size_t(3));
}

ADSIM_TEST(BehaviorTree, RepeatForever_失败与运行中均透传) {
  auto child = makeScripted("C", {kRun, kFail});
  ScriptedNode* child_ptr = child.get();

  RepeatForever repeat("repeat", std::move(child));
  CHECK_STATUS(repeat.tick(), kRun);
  CHECK_STATUS(repeat.tick(), kFail);  // 失败必须透传，让上层能换策略
  ADSIM_CHECK_EQ(child_ptr->tickCount(), std::size_t(2));
}

ADSIM_TEST(BehaviorTree, Cooldown_冷却期内返回失败且不执行子节点) {
  Blackboard bb;
  int calls = 0;
  auto child = std::unique_ptr<ActionNode>(new ActionNode(
      "act", [&calls](Blackboard&) {
        ++calls;
        return NodeStatus::kSuccess;
      },
      &bb));

  Cooldown cooldown("cooldown", std::move(child), 0.05);
  CHECK_STATUS(cooldown.tick(), kOk);  // 第一次成功，进入冷却
  ADSIM_CHECK_EQ(calls, 1);

  // 冷却期内：直接失败，且子节点一次都不能被 tick（冷却的意义就在这里）
  CHECK_STATUS(cooldown.tick(), kFail);
  CHECK_STATUS(cooldown.tick(), kFail);
  ADSIM_CHECK_EQ(calls, 1);
}

ADSIM_TEST(BehaviorTree, Cooldown_冷却结束后恢复执行) {
  Blackboard bb;
  int calls = 0;
  auto child = std::unique_ptr<ActionNode>(new ActionNode(
      "act", [&calls](Blackboard&) {
        ++calls;
        return NodeStatus::kSuccess;
      },
      &bb));

  Cooldown cooldown("cooldown", std::move(child), 0.02);
  CHECK_STATUS(cooldown.tick(), kOk);
  CHECK_STATUS(cooldown.tick(), kFail);
  ADSIM_CHECK_EQ(calls, 1);

  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  CHECK_STATUS(cooldown.tick(), kOk);  // 冷却结束，重新执行
  ADSIM_CHECK_EQ(calls, 2);
}

ADSIM_TEST(BehaviorTree, Cooldown_零冷却等价于不冷却) {
  Blackboard bb;
  int calls = 0;
  auto child = std::unique_ptr<ActionNode>(new ActionNode(
      "act", [&calls](Blackboard&) {
        ++calls;
        return NodeStatus::kSuccess;
      },
      &bb));

  Cooldown cooldown("cooldown", std::move(child), 0.0);
  CHECK_STATUS(cooldown.tick(), kOk);
  CHECK_STATUS(cooldown.tick(), kOk);
  ADSIM_CHECK_EQ(calls, 2);
}

ADSIM_TEST(BehaviorTree, Cooldown_子节点失败不进入冷却) {
  Blackboard bb;
  int calls = 0;
  auto child = std::unique_ptr<ActionNode>(new ActionNode(
      "act", [&calls](Blackboard&) {
        ++calls;
        return NodeStatus::kFailure;
      },
      &bb));

  Cooldown cooldown("cooldown", std::move(child), 10.0);  // 冷却很长
  CHECK_STATUS(cooldown.tick(), kFail);
  CHECK_STATUS(cooldown.tick(), kFail);
  // 只有成功才起冷却，失败可以立即重试
  ADSIM_CHECK_EQ(calls, 2);
}

ADSIM_TEST(BehaviorTree, TimeLimit_超时中断子节点) {
  auto child = makeScripted("slow", {kRun});
  ScriptedNode* child_ptr = child.get();

  TimeLimit limit("limit", std::move(child), 0.02);
  CHECK_STATUS(limit.tick(), kRun);  // 未超时
  ADSIM_CHECK_EQ(child_ptr->tickCount(), std::size_t(1));

  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  CHECK_STATUS(limit.tick(), kFail);  // 超时 → 中断并失败
  ADSIM_CHECK_EQ(child_ptr->haltCount(), std::size_t(1));
  ADSIM_CHECK_EQ(child_ptr->tickCount(), std::size_t(2));

  // 计时已重置：下一轮重新开始计时，不会立刻又超时
  CHECK_STATUS(limit.tick(), kRun);
  ADSIM_CHECK_EQ(child_ptr->tickCount(), std::size_t(3));
}

ADSIM_TEST(BehaviorTree, TimeLimit_子节点给出终态后重新计时) {
  Blackboard bb;
  int calls = 0;
  auto child = std::unique_ptr<ActionNode>(new ActionNode(
      "act",
      [&calls](Blackboard&) {
        ++calls;
        return calls == 1 ? NodeStatus::kSuccess : NodeStatus::kRunning;
      },
      &bb));

  TimeLimit limit("limit", std::move(child), 0.02);
  CHECK_STATUS(limit.tick(), kOk);  // 第一轮：终态成功

  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  // 若终态后没有重置计时，这一轮会带着上一轮的时间立刻判定超时
  CHECK_STATUS(limit.tick(), kRun);
}

ADSIM_TEST(BehaviorTree, TimeLimit_未超时的运行中不中断) {
  auto child = makeScripted("slow", {kRun});
  ScriptedNode* child_ptr = child.get();

  TimeLimit limit("limit", std::move(child), 10.0);  // 上限很长
  CHECK_STATUS(limit.tick(), kRun);
  CHECK_STATUS(limit.tick(), kRun);
  ADSIM_CHECK_EQ(child_ptr->haltCount(), std::size_t(0));
  ADSIM_CHECK_EQ(child_ptr->tickCount(), std::size_t(2));
}

// ===========================================================================
//  黑板
// ===========================================================================

ADSIM_TEST(BehaviorTree, Blackboard_各类型存取) {
  Blackboard bb;
  ADSIM_CHECK(!bb.has("nothing"));

  bb.setBool("flag", true);
  bb.setDouble("speed", 12.5);
  bb.setInt("gear", 3);
  bb.setString("mode", "lane_change");

  ADSIM_CHECK(bb.has("flag"));
  ADSIM_CHECK_EQ(bb.getBool("flag"), true);
  ADSIM_CHECK_EQ(bb.getBool("flag", false), true);
  ADSIM_CHECK_NEAR(bb.getDouble("speed"), 12.5, kEps);
  ADSIM_CHECK_EQ(bb.getInt("gear"), 3);
  ADSIM_CHECK_EQ(bb.getString("mode"), std::string("lane_change"));
}

ADSIM_TEST(BehaviorTree, Blackboard_缺键返回默认值) {
  Blackboard bb;
  ADSIM_CHECK_EQ(bb.getBool("missing", true), true);
  ADSIM_CHECK_EQ(bb.getBool("missing"), false);
  ADSIM_CHECK_NEAR(bb.getDouble("missing", -3.5), -3.5, kEps);
  ADSIM_CHECK_NEAR(bb.getDouble("missing"), 0.0, kEps);
  ADSIM_CHECK_EQ(bb.getInt("missing", 42), 42);
  ADSIM_CHECK_EQ(bb.getInt("missing"), 0);
  ADSIM_CHECK_EQ(bb.getString("missing", "fallback"), std::string("fallback"));
  ADSIM_CHECK_EQ(bb.getString("missing"), std::string(""));
  ADSIM_CHECK(!bb.has("missing"));
}

ADSIM_TEST(BehaviorTree, Blackboard_类型不匹配返回默认值) {
  Blackboard bb;

  // 键存在但类型不同：必须回退默认值，而不是把 int 硬转成别的类型
  bb.setInt("count", 7);
  ADSIM_CHECK_EQ(bb.getInt("count", 0), 7);  // 同类型正常读到
  ADSIM_CHECK_EQ(bb.getBool("count", true), true);
  ADSIM_CHECK_NEAR(bb.getDouble("count", 1.5), 1.5, kEps);
  ADSIM_CHECK_EQ(bb.getString("count", "none"), std::string("none"));

  bb.setDouble("speed", 8.5);
  ADSIM_CHECK_NEAR(bb.getDouble("speed", 0.0), 8.5, kEps);
  ADSIM_CHECK_EQ(bb.getBool("speed", true), true);
  ADSIM_CHECK_EQ(bb.getInt("speed", -1), -1);
  ADSIM_CHECK_EQ(bb.getString("speed", "x"), std::string("x"));

  bb.setBool("flag", false);
  ADSIM_CHECK_EQ(bb.getBool("flag", true), false);
  ADSIM_CHECK_EQ(bb.getInt("flag", 42), 42);
  ADSIM_CHECK_NEAR(bb.getDouble("flag", 2.5), 2.5, kEps);
  ADSIM_CHECK_EQ(bb.getString("flag", "d"), std::string("d"));

  bb.setString("mode", "follow");
  ADSIM_CHECK_EQ(bb.getString("mode", "d"), std::string("follow"));
  ADSIM_CHECK_EQ(bb.getBool("mode", true), true);
  ADSIM_CHECK_EQ(bb.getInt("mode", 3), 3);
  ADSIM_CHECK_NEAR(bb.getDouble("mode", -1.0), -1.0, kEps);
}

ADSIM_TEST(BehaviorTree, Blackboard_覆盖写删除与清空) {
  Blackboard bb;
  bb.setInt("value", 1);
  bb.setInt("value", 2);  // 同键同类型覆盖
  ADSIM_CHECK_EQ(bb.getInt("value", 0), 2);

  // 同键换类型：读到新值，旧类型读取回退默认值（不残留旧值）
  bb.setDouble("value", 3.5);
  ADSIM_CHECK_NEAR(bb.getDouble("value", 0.0), 3.5, kEps);
  ADSIM_CHECK_EQ(bb.getInt("value", -1), -1);

  ADSIM_CHECK(bb.has("value"));
  bb.erase("value");
  ADSIM_CHECK(!bb.has("value"));
  ADSIM_CHECK_EQ(bb.getDouble("value", -1.0), -1.0);

  bb.setBool("a", true);
  bb.setBool("b", true);
  bb.clear();
  ADSIM_CHECK(!bb.has("a"));
  ADSIM_CHECK(!bb.has("b"));
}

// ===========================================================================
//  叶子节点
// ===========================================================================

ADSIM_TEST(BehaviorTree, ActionNode_空回调返回失败) {
  Blackboard bb;
  ActionNode node("empty", ActionNode::Action{}, &bb);

  CHECK_STATUS(node.tick(), kFail);
  CHECK_STATUS(node.tick(), kFail);
  ADSIM_CHECK_EQ(node.tickCount(), std::size_t(2));  // 空回调同样计入 tick
  ADSIM_CHECK(node.lastTickMs() >= 0.0);
}

ADSIM_TEST(BehaviorTree, ConditionNode_空回调返回失败) {
  Blackboard bb;
  ConditionNode node("empty", ConditionNode::Condition{}, &bb);

  CHECK_STATUS(node.tick(), kFail);
  ADSIM_CHECK_EQ(node.tickCount(), std::size_t(1));
}

ADSIM_TEST(BehaviorTree, ConditionNode_条件真假映射为成功失败) {
  Blackboard bb;
  bb.setBool("lane_free", true);
  ConditionNode cond(
      "lane_free", [](const Blackboard& b) { return b.getBool("lane_free", false); },
      &bb);

  CHECK_STATUS(cond.tick(), kOk);
  bb.setBool("lane_free", false);
  CHECK_STATUS(cond.tick(), kFail);
  // 条件节点永远不会是 Running
  ADSIM_CHECK_EQ(cond.tickCount(), std::size_t(2));
}

ADSIM_TEST(BehaviorTree, ActionNode_读写黑板并透传三态) {
  Blackboard bb;
  bb.setDouble("speed", 0.0);
  int calls = 0;

  ActionNode node(
      "accel",
      [&calls](Blackboard& b) {
        ++calls;
        b.setDouble("speed", b.getDouble("speed", 0.0) + 1.0);
        return calls < 3 ? NodeStatus::kRunning : NodeStatus::kSuccess;
      },
      &bb);

  CHECK_STATUS(node.tick(), kRun);
  CHECK_STATUS(node.tick(), kRun);
  CHECK_STATUS(node.tick(), kOk);
  ADSIM_CHECK_NEAR(bb.getDouble("speed", 0.0), 3.0, kEps);
  ADSIM_CHECK_EQ(node.tickCount(), std::size_t(3));
  ADSIM_CHECK(node.lastTickMs() >= 0.0);
}

// ===========================================================================
//  整树
// ===========================================================================

ADSIM_TEST(BehaviorTree, 空树tick返回失败) {
  Blackboard bb;
  BehaviorTree tree(&bb);

  ADSIM_CHECK(tree.root() == nullptr);
  CHECK_STATUS(tree.tick(), kFail);
  ADSIM_CHECK_EQ(tree.tickCount(), std::size_t(0));  // 空树不算一次有效 tick
  ADSIM_CHECK(tree.dump().empty());                  // 无结构可导出

  // 空树上调用 halt/reset 必须安全
  tree.halt();
  tree.reset();
}

ADSIM_TEST(BehaviorTree, tick计数累加) {
  Blackboard bb;
  int calls = 0;
  auto root = std::unique_ptr<ActionNode>(new ActionNode(
      "act", [&calls](Blackboard&) {
        ++calls;
        return NodeStatus::kSuccess;
      },
      &bb));

  BehaviorTree tree(&bb);
  ADSIM_CHECK(tree.root() == nullptr);
  tree.setRoot(std::move(root));
  ADSIM_CHECK(tree.root() != nullptr);
  ADSIM_CHECK_EQ(tree.root()->name(), std::string("act"));

  for (int i = 0; i < 5; ++i) CHECK_STATUS(tree.tick(), kOk);
  ADSIM_CHECK_EQ(tree.tickCount(), std::size_t(5));
  ADSIM_CHECK_EQ(calls, 5);

  // 黑板可通过树访问
  tree.blackboard().setInt("tick_marks", 3);
  ADSIM_CHECK_EQ(tree.blackboard().getInt("tick_marks", 0), 3);

  // reset 清的是执行状态，累计 tick 次数是诊断量，不清零
  tree.reset();
  ADSIM_CHECK_EQ(tree.tickCount(), std::size_t(5));
}

ADSIM_TEST(BehaviorTree, halt与reset传播到根) {
  auto root = std::make_unique<Sequence>("root");
  auto a = makeScripted("A", {kRun});
  auto b = makeScripted("B", {kOk});
  ScriptedNode* a_ptr = a.get();
  ScriptedNode* b_ptr = b.get();
  root->addChild(std::move(a));
  root->addChild(std::move(b));
  Sequence* root_ptr = root.get();

  Blackboard bb;
  BehaviorTree tree(&bb);
  tree.setRoot(std::move(root));

  CHECK_STATUS(tree.tick(), kRun);
  // A 返回 Running：断点仍指向 A（下标 0），B 尚未执行
  ADSIM_CHECK_EQ(root_ptr->currentIndex(), std::size_t(0));
  ADSIM_CHECK_EQ(b_ptr->tickCount(), std::size_t(0));

  // halt：中断正在执行的 A，并把断点清零
  tree.halt();
  ADSIM_CHECK_EQ(root_ptr->currentIndex(), std::size_t(0));
  ADSIM_CHECK_EQ(a_ptr->haltCount(), std::size_t(1));  // 中断传播到运行中的分支
  ADSIM_CHECK_EQ(b_ptr->haltCount(), std::size_t(0));

  // 中断后重新开始，仍从第一个子节点执行
  CHECK_STATUS(tree.tick(), kRun);
  ADSIM_CHECK_EQ(a_ptr->tickCount(), std::size_t(2));
  ADSIM_CHECK_EQ(b_ptr->tickCount(), std::size_t(0));

  // reset：整棵子树回到初始执行状态
  tree.reset();
  ADSIM_CHECK_EQ(root_ptr->currentIndex(), std::size_t(0));
  CHECK_STATUS(tree.tick(), kRun);
  ADSIM_CHECK_EQ(a_ptr->tickCount(), std::size_t(3));
}

ADSIM_TEST(BehaviorTree, reset级联到装饰节点) {
  auto child = makeScripted("C", {kFail});
  ScriptedNode* child_ptr = child.get();

  auto root = std::make_unique<Retry>("retry", std::move(child), 2);
  Blackboard bb;
  BehaviorTree tree(&bb);
  tree.setRoot(std::move(root));

  CHECK_STATUS(tree.tick(), kFail);
  ADSIM_CHECK_EQ(child_ptr->tickCount(), std::size_t(2));

  // 重试额度已耗尽：若 reset 不级联到装饰节点，这里一次都不会再执行
  tree.reset();
  CHECK_STATUS(tree.tick(), kFail);
  ADSIM_CHECK_EQ(child_ptr->tickCount(), std::size_t(4));
}

ADSIM_TEST(BehaviorTree, dump包含节点类型名与缩进) {
  Blackboard bb;
  bb.setDouble("obstacle_distance", 100.0);

  auto decision = std::make_unique<Selector>("决策");
  auto avoid = std::make_unique<Sequence>("避障换道");
  avoid->addChild(std::make_unique<ConditionNode>(
      "前方有障碍物",
      [](const Blackboard& b) { return b.getDouble("obstacle_distance", 1e9) < 30.0; },
      &bb));
  avoid->addChild(std::make_unique<ActionNode>(
      "执行换道", [](Blackboard&) { return NodeStatus::kSuccess; }, &bb));
  decision->addChild(std::move(avoid));
  decision->addChild(std::make_unique<ActionNode>(
      "减速跟车", [](Blackboard&) { return NodeStatus::kSuccess; }, &bb));

  BehaviorTree tree(&bb);
  tree.setRoot(std::move(decision));

  const std::string text = tree.dump();
  ADSIM_CHECK(!text.empty());
  ADSIM_CHECK(text.find("Selector") != std::string::npos);
  ADSIM_CHECK(text.find("决策") != std::string::npos);
  ADSIM_CHECK(text.find("Sequence") != std::string::npos);
  ADSIM_CHECK(text.find("避障换道") != std::string::npos);
  ADSIM_CHECK(text.find("ConditionNode") != std::string::npos);
  ADSIM_CHECK(text.find("前方有障碍物") != std::string::npos);
  ADSIM_CHECK(text.find("ActionNode") != std::string::npos);
  ADSIM_CHECK(text.find("减速跟车") != std::string::npos);

  // 缩进：每层两个空格
  ADSIM_CHECK(text.find("\n  Sequence") != std::string::npos);
  ADSIM_CHECK(text.find("\n    ConditionNode") != std::string::npos);
  ADSIM_CHECK_EQ(text.back(), '\n');
}

ADSIM_TEST(BehaviorTree, dump空树为空字符串) {
  Blackboard bb;
  BehaviorTree tree(&bb);
  ADSIM_CHECK(tree.dump().empty());
}

ADSIM_TEST(BehaviorTree, dump完整展开组合与装饰子树) {
  Blackboard bb;
  bb.setDouble("obstacle_distance", 100.0);
  bb.setBool("blind_spot_occupied", false);

  // 6 层结构，同时含组合节点 / 装饰节点 / 叶子节点：
  //   Selector"顶决策"
  //     Sequence"避障序列"
  //       ConditionNode"前方有障碍物"
  //       Retry"重试换道"
  //         Cooldown"换道冷却"
  //           TimeLimit"换道时限"
  //             ActionNode"执行换道"      ← 第 5 层，只能由装饰节点逐层展开得到
  //     Parallel"并行监视"
  //       ActionNode"监视左侧车道"
  //       Inverter"取反"
  //         ConditionNode"盲区有车"
  //     ActionNode"减速跟车"
  auto root = std::make_unique<Selector>("顶决策");

  auto avoid = std::make_unique<Sequence>("避障序列");
  avoid->addChild(std::make_unique<ConditionNode>(
      "前方有障碍物",
      [](const Blackboard& b) { return b.getDouble("obstacle_distance", 1e9) < 30.0; },
      &bb));
  avoid->addChild(std::make_unique<Retry>(
      "重试换道",
      std::make_unique<Cooldown>(
          "换道冷却",
          std::make_unique<TimeLimit>(
              "换道时限",
              std::make_unique<ActionNode>(
                  "执行换道", [](Blackboard&) { return NodeStatus::kRunning; }, &bb),
              2.5),
          1.0),
      3));
  root->addChild(std::move(avoid));

  auto watch = std::make_unique<Parallel>("并行监视", Parallel::Policy::kRequireAll, 2);
  watch->addChild(std::make_unique<ActionNode>(
      "监视左侧车道", [](Blackboard&) { return NodeStatus::kSuccess; }, &bb));
  watch->addChild(std::make_unique<Inverter>(
      "取反",
      std::make_unique<ConditionNode>(
          "盲区有车",
          [](const Blackboard& b) { return b.getBool("blind_spot_occupied", false); },
          &bb)));
  root->addChild(std::move(watch));

  root->addChild(std::make_unique<ActionNode>(
      "减速跟车", [](Blackboard&) { return NodeStatus::kSuccess; }, &bb));

  BehaviorTree tree(&bb);
  tree.setRoot(std::move(root));

  const std::string text = tree.dump();
  ADSIM_CHECK(!text.empty());

  // 逐个断言节点名都在：只要有一层装饰子树没被展开，深层的名字就会缺失
  const char* names[] = {"顶决策", "避障序列", "前方有障碍物", "重试换道",
                         "换道冷却", "换道时限", "执行换道", "并行监视",
                         "监视左侧车道", "取反", "盲区有车", "减速跟车"};
  for (const char* name : names) {
    ADSIM_CHECK_MSG(text.find(name) != std::string::npos,
                    std::string("dump 中缺少节点: ") + name);
  }

  // 每一层缩进递增两个空格，且组合与装饰节点都被展开到了下一层
  ADSIM_CHECK(text.find("Selector\"顶决策\"\n") == 0);  // 根节点无缩进
  ADSIM_CHECK(text.find("\n  Sequence\"避障序列\"") != std::string::npos);
  ADSIM_CHECK(text.find("\n    ConditionNode\"前方有障碍物\"") != std::string::npos);
  // Retry / Cooldown / TimeLimit / Parallel 的标注要同时带上配置值与运行态
  ADSIM_CHECK(text.find("\n    Retry\"重试换道\" max_attempts=3 attempts=0") !=
              std::string::npos);
  ADSIM_CHECK(text.find("\n      Cooldown\"换道冷却\" cooldown_seconds=1.000 cooling=false") !=
              std::string::npos);
  ADSIM_CHECK(text.find("\n        TimeLimit\"换道时限\" limit_seconds=2.500") !=
              std::string::npos);
  ADSIM_CHECK(text.find("\n          ActionNode\"执行换道\"") != std::string::npos);
  ADSIM_CHECK(text.find("\n  Parallel\"并行监视\" policy=RequireAll failure_threshold=2") !=
              std::string::npos);
  ADSIM_CHECK(text.find("\n    ActionNode\"监视左侧车道\"") != std::string::npos);
  ADSIM_CHECK(text.find("\n    Inverter\"取反\"") != std::string::npos);
  ADSIM_CHECK(text.find("\n      ConditionNode\"盲区有车\"") != std::string::npos);
  ADSIM_CHECK(text.find("\n  ActionNode\"减速跟车\"") != std::string::npos);

  // 每个节点恰好一行
  ADSIM_CHECK_EQ(std::count(text.begin(), text.end(), '\n'), std::ptrdiff_t(12));
  ADSIM_CHECK_EQ(text.back(), '\n');

  // 深度优先顺序：子树整棵展开完才轮到兄弟节点
  ADSIM_CHECK(text.find("顶决策") < text.find("避障序列"));
  ADSIM_CHECK(text.find("避障序列") < text.find("前方有障碍物"));
  ADSIM_CHECK(text.find("执行换道") < text.find("并行监视"));
  ADSIM_CHECK(text.find("取反") < text.find("盲区有车"));
  ADSIM_CHECK(text.find("盲区有车") < text.find("减速跟车"));
}

ADSIM_TEST(BehaviorTree, dump标注装饰节点运行状态) {
  Blackboard bb;

  // Retry：尝试 3 次仍失败后额度耗尽，dump 应反映出累计尝试次数
  auto retry = std::unique_ptr<Retry>(new Retry(
      "重试",
      std::unique_ptr<Node>(new ActionNode(
          "失败动作", [](Blackboard&) { return NodeStatus::kFailure; }, &bb)),
      3));
  CHECK_STATUS(retry->tick(), kFail);
  ADSIM_CHECK_EQ(retry->attempts(), std::size_t(3));
  ADSIM_CHECK(retry->child() != nullptr);  // 新增的装饰节点访问器可用

  BehaviorTree retry_tree(&bb);
  retry_tree.setRoot(std::move(retry));
  ADSIM_CHECK(retry_tree.dump().find("Retry\"重试\" max_attempts=3 attempts=3") !=
              std::string::npos);

  // Cooldown：成功后进入冷却，dump 应反映出冷却中
  auto cooldown = std::unique_ptr<Cooldown>(new Cooldown(
      "冷却",
      std::unique_ptr<Node>(new ActionNode(
          "成功动作", [](Blackboard&) { return NodeStatus::kSuccess; }, &bb)),
      10.0));
  CHECK_STATUS(cooldown->tick(), kOk);
  ADSIM_CHECK(cooldown->cooling());
  ADSIM_CHECK(cooldown->child() != nullptr);

  BehaviorTree cooldown_tree(&bb);
  cooldown_tree.setRoot(std::move(cooldown));
  ADSIM_CHECK(cooldown_tree.dump().find("Cooldown\"冷却\" cooldown_seconds=10.000 cooling=true") !=
              std::string::npos);

  // TimeLimit：只导出配置的时限（started_/剩余时间属于内部计时，不进 dump）
  auto time_limit = std::unique_ptr<TimeLimit>(new TimeLimit(
      "时限",
      std::unique_ptr<Node>(new ActionNode(
          "长时间动作", [](Blackboard&) { return NodeStatus::kRunning; }, &bb)),
      2.5));
  CHECK_STATUS(time_limit->tick(), kRun);
  ADSIM_CHECK(time_limit->child() != nullptr);

  BehaviorTree limit_tree(&bb);
  limit_tree.setRoot(std::move(time_limit));
  ADSIM_CHECK(limit_tree.dump().find("TimeLimit\"时限\" limit_seconds=2.500") !=
              std::string::npos);

  // Parallel：策略与失败阈值都要写进标注
  auto parallel = std::unique_ptr<Parallel>(
      new Parallel("并行", Parallel::Policy::kRequireOne, 2));
  parallel->addChild(std::unique_ptr<Node>(
      new ActionNode("分支甲", [](Blackboard&) { return NodeStatus::kFailure; }, &bb)));
  parallel->addChild(std::unique_ptr<Node>(
      new ActionNode("分支乙", [](Blackboard&) { return NodeStatus::kRunning; }, &bb)));
  CHECK_STATUS(parallel->tick(), kRun);
  // 枚举类没有 operator<<，断言必须转成整型再比较（否则宏里的流输出无法编译）
  ADSIM_CHECK_EQ(static_cast<int>(parallel->successPolicy()),
                 static_cast<int>(Parallel::Policy::kRequireOne));
  ADSIM_CHECK_EQ(parallel->failureThreshold(), std::size_t(2));

  BehaviorTree parallel_tree(&bb);
  parallel_tree.setRoot(std::move(parallel));
  ADSIM_CHECK(parallel_tree.dump().find("Parallel\"并行\" policy=RequireOne failure_threshold=2") !=
              std::string::npos);
}

// ===========================================================================
//  贴近真实决策的场景
//  若前方有障碍物则尝试换道；换道不成就降级为减速跟车
// ===========================================================================

struct DecisionRig {
  Blackboard bb;
  int obstacle_checks{0};       // "前方有障碍物"被评估的次数
  int lane_change_attempts{0};  // "尝试换道"被执行的次数
  int lane_change_steps{0};     // "执行换道"推进了多少个周期
  int decel_calls{0};           // "减速跟车"被执行的次数
  BehaviorTree tree{&bb};

  DecisionRig() { build(); }

  void build() {
    auto decision = std::make_unique<Selector>("决策");
    auto avoid = std::make_unique<Sequence>("避障换道");

    avoid->addChild(std::make_unique<ConditionNode>(
        "前方有障碍物",
        [this](const Blackboard& b) {
          ++obstacle_checks;
          return b.getDouble("obstacle_distance", 1e9) < 30.0;
        },
        &bb));
    avoid->addChild(std::make_unique<ActionNode>(
        "尝试换道",
        [this](Blackboard& b) {
          ++lane_change_attempts;
          return b.getBool("left_lane_free", false) ? NodeStatus::kSuccess
                                                    : NodeStatus::kFailure;
        },
        &bb));
    avoid->addChild(std::make_unique<ActionNode>(
        "执行换道",
        [this](Blackboard& b) {
          ++lane_change_steps;
          if (lane_change_steps < 3) return NodeStatus::kRunning;  // 需要连续 3 个周期
          b.setBool("lane_changed", true);
          return NodeStatus::kSuccess;
        },
        &bb));

    auto follow = std::make_unique<Sequence>("跟车降级");
    follow->addChild(std::make_unique<ActionNode>(
        "减速跟车",
        [this](Blackboard& b) {
          ++decel_calls;
          b.setDouble("target_speed", 5.0);
          return NodeStatus::kSuccess;
        },
        &bb));
    follow->addChild(std::make_unique<ActionNode>(
        "保持安全距离",
        [](Blackboard& b) {
          b.setBool("safe_gap", true);
          return NodeStatus::kSuccess;
        },
        &bb));

    decision->addChild(std::move(avoid));
    decision->addChild(std::move(follow));
    tree.setRoot(std::move(decision));
  }
};

ADSIM_TEST(BehaviorTree, 场景_前方无障碍时保持跟车) {
  DecisionRig rig;
  rig.bb.setDouble("obstacle_distance", 120.0);
  rig.bb.setBool("left_lane_free", true);

  CHECK_STATUS(rig.tree.tick(), kOk);
  // 条件不满足，整个换道分支被跳过
  ADSIM_CHECK_EQ(rig.obstacle_checks, 1);
  ADSIM_CHECK_EQ(rig.lane_change_attempts, 0);
  ADSIM_CHECK_EQ(rig.lane_change_steps, 0);
  // 走跟车降级分支
  ADSIM_CHECK_EQ(rig.decel_calls, 1);
  ADSIM_CHECK_NEAR(rig.bb.getDouble("target_speed", 0.0), 5.0, kEps);
  ADSIM_CHECK(rig.bb.getBool("safe_gap", false));
  ADSIM_CHECK(!rig.bb.getBool("lane_changed", false));
}

ADSIM_TEST(BehaviorTree, 场景_有障碍且左侧空闲时换道) {
  DecisionRig rig;
  rig.bb.setDouble("obstacle_distance", 12.0);
  rig.bb.setBool("left_lane_free", true);

  // 换道需要连续 3 个周期，中途整体处于 Running
  CHECK_STATUS(rig.tree.tick(), kRun);
  ADSIM_CHECK_EQ(rig.obstacle_checks, 1);
  ADSIM_CHECK_EQ(rig.lane_change_attempts, 1);
  ADSIM_CHECK_EQ(rig.lane_change_steps, 1);
  CHECK_STATUS(rig.tree.tick(), kRun);
  // 关键：断点续跑——"尝试换道"与前提条件都不会因为 Running 被反复重放
  ADSIM_CHECK_EQ(rig.obstacle_checks, 1);
  ADSIM_CHECK_EQ(rig.lane_change_attempts, 1);
  ADSIM_CHECK_EQ(rig.lane_change_steps, 2);

  CHECK_STATUS(rig.tree.tick(), kOk);
  ADSIM_CHECK_EQ(rig.lane_change_steps, 3);
  ADSIM_CHECK(rig.bb.getBool("lane_changed", false));
  ADSIM_CHECK_EQ(rig.decel_calls, 0);  // 换道成功，未降级
}

ADSIM_TEST(BehaviorTree, 场景_换道失败降级为减速跟车) {
  DecisionRig rig;
  rig.bb.setDouble("obstacle_distance", 8.0);
  rig.bb.setBool("left_lane_free", false);  // 左侧车道被占用

  // 换道分支失败后，选择节点在同一次 tick 内改用跟车分支
  CHECK_STATUS(rig.tree.tick(), kOk);
  ADSIM_CHECK_EQ(rig.obstacle_checks, 1);
  ADSIM_CHECK_EQ(rig.lane_change_attempts, 1);
  ADSIM_CHECK_EQ(rig.lane_change_steps, 0);  // 换道动作根本没启动
  ADSIM_CHECK_EQ(rig.decel_calls, 1);
  ADSIM_CHECK_NEAR(rig.bb.getDouble("target_speed", 0.0), 5.0, kEps);
  ADSIM_CHECK(rig.bb.getBool("safe_gap", false));
  ADSIM_CHECK(!rig.bb.getBool("lane_changed", false));
}

ADSIM_TEST(BehaviorTree, 场景_换道进行中不被新输入打断且完成后重新决策) {
  DecisionRig rig;
  rig.bb.setDouble("obstacle_distance", 10.0);
  rig.bb.setBool("left_lane_free", true);

  CHECK_STATUS(rig.tree.tick(), kRun);
  ADSIM_CHECK_EQ(rig.lane_change_steps, 1);

  // 换道执行途中障碍物被移走：已经启动的分支会跑完（行为树的执行语义是
  // "正在执行的分支优先"，前提条件不会在 Running 期间被重新评估），
  // 若需要"随时可打断"，应把前提条件做成单独的并行监视分支。
  rig.bb.setDouble("obstacle_distance", 200.0);
  CHECK_STATUS(rig.tree.tick(), kRun);
  ADSIM_CHECK_EQ(rig.obstacle_checks, 1);  // 前提条件未被重新评估
  ADSIM_CHECK_EQ(rig.lane_change_steps, 2);

  CHECK_STATUS(rig.tree.tick(), kOk);
  ADSIM_CHECK_EQ(rig.lane_change_steps, 3);
  ADSIM_CHECK(rig.bb.getBool("lane_changed", false));

  // 换道完成后本轮决策结束，下一周期重新评估：前方已无障碍 → 跟车分支
  CHECK_STATUS(rig.tree.tick(), kOk);
  ADSIM_CHECK_EQ(rig.obstacle_checks, 2);
  ADSIM_CHECK_EQ(rig.decel_calls, 1);
  ADSIM_CHECK_EQ(rig.lane_change_attempts, 1);  // 不会重复发起换道
}

}  // namespace
