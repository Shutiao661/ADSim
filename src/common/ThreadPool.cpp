#include "adsim/common/ThreadPool.h"

#include <algorithm>
#include <stdexcept>

namespace adsim {

ThreadPool::ThreadPool(std::size_t thread_count) {
  if (thread_count == 0) {
    thread_count = std::max<std::size_t>(1, std::thread::hardware_concurrency());
  }
  thread_count_ = thread_count;

  workers_.reserve(thread_count);
  for (std::size_t i = 0; i < thread_count; ++i) {
    workers_.emplace_back([this]() { workerLoop(); });
  }
}

ThreadPool::~ThreadPool() {
  shutdown();
}

void ThreadPool::workerLoop() {
  for (;;) {
    std::function<void()> task;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      condition_.wait(lock, [this]() { return stopping_ || !tasks_.empty(); });

      if (stopping_ && tasks_.empty()) {
        return;
      }

      task = std::move(tasks_.front());
      tasks_.pop();
      ++active_tasks_;
    }

    task();

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      --active_tasks_;
      ++completed_tasks_;
      if (tasks_.empty() && active_tasks_ == 0) {
        idle_condition_.notify_all();
      }
    }
  }
}

void ThreadPool::waitIdle() {
  std::unique_lock<std::mutex> lock(queue_mutex_);
  idle_condition_.wait(lock, [this]() { return tasks_.empty() && active_tasks_ == 0; });
}

void ThreadPool::shutdown() {
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (stopping_) return;
    accepting_ = false;
    stopping_ = true;
  }
  condition_.notify_all();

  for (std::thread& worker : workers_) {
    if (worker.joinable()) {
      worker.join();
    }
  }
  workers_.clear();
}

std::size_t ThreadPool::pendingTasks() const {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  return tasks_.size();
}

ThreadPool::Stats ThreadPool::stats() const {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  Stats s;
  s.thread_count = thread_count_;
  s.pending_tasks = tasks_.size();
  s.completed_tasks = completed_tasks_;
  s.active_tasks = active_tasks_;
  return s;
}

// ---------------------------------------------------------------------------
// 全局线程池
// ---------------------------------------------------------------------------

}  // namespace adsim
