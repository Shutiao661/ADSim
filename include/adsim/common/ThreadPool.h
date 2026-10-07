// =============================================================================
//  ThreadPool.h — 固定线程数任务池
//
//  为路测数据管道提供并行执行能力：多传感器数据解包、点云降噪、场景批量
//  回放等任务均通过该池调度。
//
//  特性：
//    * 任务队列 + 条件变量唤醒，空闲线程不占用 CPU
//    * enqueue 返回 std::future，支持返回值与异常传播
//    * parallelFor 支持按粒度自动切分的并行循环
//    * waitIdle 用于管道各阶段之间的同步屏障
// =============================================================================
#pragma once

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace adsim {

class ThreadPool {
 public:
  struct Stats {
    std::size_t thread_count{0};
    std::size_t pending_tasks{0};
    std::size_t completed_tasks{0};
    std::size_t active_tasks{0};
  };

  /// @param thread_count 工作线程数；传 0 表示使用 hardware_concurrency()
  explicit ThreadPool(std::size_t thread_count = 0);
  ~ThreadPool();

  ThreadPool(const ThreadPool&) = delete;
  ThreadPool& operator=(const ThreadPool&) = delete;

  /// 提交任务，返回 future 以获取结果或异常。
  ///
  /// 采用 lambda 按移动捕获参数，而非 std::bind：
  /// std::bind 会要求参数可拷贝，导致内存池块这类 move-only 资源无法入队。
  template <typename F, typename... Args>
  auto enqueue(F&& f, Args&&... args)
      -> std::future<typename std::invoke_result<F, Args...>::type> {
    using ReturnType = typename std::invoke_result<F, Args...>::type;

    // C++17 无 pack init-capture，改用 tuple + std::apply 保存参数
    auto bound = [fn = std::forward<F>(f),
                  args = std::make_tuple(std::forward<Args>(args)...)]() mutable -> ReturnType {
      return std::apply(fn, std::move(args));
    };

    auto task = std::make_shared<std::packaged_task<ReturnType()>>(std::move(bound));

    std::future<ReturnType> result = task->get_future();
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (!accepting_) {
        throw std::runtime_error("ThreadPool: 已停止接收新任务");
      }
      tasks_.emplace([task]() { (*task)(); });
    }
    condition_.notify_one();
    return result;
  }

  /// 并行 for 循环：将 [begin, end) 按 grain 切分后分发
  /// @param grain 单个任务处理的最小元素数，过小会导致调度开销主导
  template <typename IndexFn>
  void parallelFor(std::size_t begin, std::size_t end, std::size_t grain, IndexFn&& fn) {
    if (begin >= end) return;
    if (grain == 0) grain = 1;

    const std::size_t total = end - begin;
    // 元素较少时直接串行执行，避免调度开销超过收益
    if (total <= grain || thread_count_ <= 1) {
      for (std::size_t i = begin; i < end; ++i) fn(i);
      return;
    }

    const std::size_t task_count =
        std::min(thread_count_, (total + grain - 1) / grain);
    const std::size_t chunk = (total + task_count - 1) / task_count;

    std::vector<std::future<void>> futures;
    futures.reserve(task_count);

    for (std::size_t t = 0; t < task_count; ++t) {
      const std::size_t lo = begin + t * chunk;
      if (lo >= end) break;
      const std::size_t hi = std::min(lo + chunk, end);
      futures.emplace_back(enqueue([&fn, lo, hi]() {
        for (std::size_t i = lo; i < hi; ++i) fn(i);
      }));
    }

    for (auto& f : futures) {
      f.get();
    }
  }

  /// 阻塞直到队列清空且所有任务执行完毕
  void waitIdle();

  /// 停止接收新任务并等待已入队任务完成
  void shutdown();

  std::size_t size() const { return thread_count_; }
  std::size_t pendingTasks() const;
  Stats stats() const;

 private:
  void workerLoop();

  mutable std::mutex queue_mutex_;
  std::condition_variable condition_;
  std::condition_variable idle_condition_;
  std::queue<std::function<void()>> tasks_;
  std::vector<std::thread> workers_;

  std::size_t thread_count_{0};
  bool accepting_{true};
  bool stopping_{false};
  std::size_t active_tasks_{0};
  std::size_t completed_tasks_{0};
};

}  // namespace adsim
