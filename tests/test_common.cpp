// =============================================================================
//  test_common.cpp — 基础设施层单元测试
//  覆盖：类型系统 / 内存池 / 线程池 / 无锁环形缓冲 / 性能剖析
// =============================================================================
#include "TestFramework.h"

#include "adsim/common/MemoryPool.h"
#include "adsim/common/Profiler.h"
#include "adsim/common/RingBuffer.h"
#include "adsim/common/ThreadPool.h"
#include "adsim/common/Types.h"

#include <atomic>
#include <cstdint>
#include <cstring>
#include <numeric>
#include <set>
#include <thread>
#include <vector>

using namespace adsim;

// ===========================================================================
//  类型系统
// ===========================================================================

ADSIM_TEST(Types, 角度归一化) {
  ADSIM_CHECK_NEAR(normalizeAngle(0.0), 0.0, 1e-12);
  ADSIM_CHECK_NEAR(normalizeAngle(kPi * 0.5), kPi * 0.5, 1e-12);
  ADSIM_CHECK_NEAR(normalizeAngle(kPi * 2.5), kPi * 0.5, 1e-12);
  ADSIM_CHECK_NEAR(normalizeAngle(-kPi * 2.5), -kPi * 0.5, 1e-12);
  // 结果必须落在 (-pi, pi]
  for (double a = -20.0; a <= 20.0; a += 0.37) {
    const double n = normalizeAngle(a);
    ADSIM_CHECK(n > -kPi - 1e-9 && n <= kPi + 1e-9);
  }
}

ADSIM_TEST(Types, 向量基本运算) {
  const Vec2 a(3.0, 4.0);
  const Vec2 b(1.0, 2.0);

  ADSIM_CHECK_NEAR(a.norm(), 5.0, 1e-12);
  ADSIM_CHECK_NEAR(a.dot(b), 11.0, 1e-12);
  ADSIM_CHECK_NEAR(a.cross(b), 3.0 * 2.0 - 4.0 * 1.0, 1e-12);

  const Vec2 sum = a + b;
  ADSIM_CHECK_NEAR(sum.x, 4.0, 1e-12);
  ADSIM_CHECK_NEAR(sum.y, 6.0, 1e-12);

  // 逆时针旋转 90 度
  const Vec2 r = Vec2(1.0, 0.0).rotated(kPi * 0.5);
  ADSIM_CHECK_NEAR(r.x, 0.0, 1e-12);
  ADSIM_CHECK_NEAR(r.y, 1.0, 1e-12);

  // 零向量归一化不应产生 NaN
  const Vec2 z = Vec2(0.0, 0.0).normalized();
  ADSIM_CHECK_NEAR(z.x, 0.0, 1e-12);
  ADSIM_CHECK_NEAR(z.y, 0.0, 1e-12);
}

ADSIM_TEST(Types, 位姿坐标变换往返一致) {
  const Pose2 pose(10.0, -5.0, deg2rad(37.0));
  const std::vector<Vec2> points = {
      {0.0, 0.0}, {1.0, 2.0}, {-3.5, 4.25}, {100.0, -200.0}};

  for (const Vec2& p : points) {
    const Vec2 local = pose.toLocal(p);
    const Vec2 back = pose.toGlobal(local);
    ADSIM_CHECK_NEAR(back.x, p.x, 1e-9);
    ADSIM_CHECK_NEAR(back.y, p.y, 1e-9);
  }
}

ADSIM_TEST(Types, 有向包围盒) {
  const Obb2 box(Pose2(0.0, 0.0, 0.0), 4.0, 2.0);
  const auto corners = box.corners();
  ADSIM_CHECK_EQ(corners.size(), std::size_t(4));

  const BoundingBox2 aabb = box.aabb();
  ADSIM_CHECK_NEAR(aabb.min.x, -2.0, 1e-9);
  ADSIM_CHECK_NEAR(aabb.max.x, 2.0, 1e-9);
  ADSIM_CHECK_NEAR(aabb.min.y, -1.0, 1e-9);
  ADSIM_CHECK_NEAR(aabb.max.y, 1.0, 1e-9);

  // 旋转 90 度后长宽应在 AABB 中互换
  const Obb2 rotated(Pose2(0.0, 0.0, kPi * 0.5), 4.0, 2.0);
  const BoundingBox2 raabb = rotated.aabb();
  ADSIM_CHECK_NEAR(raabb.min.x, -1.0, 1e-9);
  ADSIM_CHECK_NEAR(raabb.max.x, 1.0, 1e-9);
  ADSIM_CHECK_NEAR(raabb.min.y, -2.0, 1e-9);
  ADSIM_CHECK_NEAR(raabb.max.y, 2.0, 1e-9);
}

ADSIM_TEST(Types, 包围盒相交判断) {
  BoundingBox2 a;
  a.expand({0.0, 0.0});
  a.expand({2.0, 2.0});

  BoundingBox2 b;
  b.expand({1.0, 1.0});
  b.expand({3.0, 3.0});

  BoundingBox2 c;
  c.expand({5.0, 5.0});
  c.expand({6.0, 6.0});

  ADSIM_CHECK(a.overlaps(b));
  ADSIM_CHECK(b.overlaps(a));
  ADSIM_CHECK(!a.overlaps(c));
  ADSIM_CHECK(!c.overlaps(a));
  ADSIM_CHECK(a.contains(Vec2(1.0, 1.0)));
  ADSIM_CHECK(!a.contains(Vec2(3.0, 3.0)));
}

ADSIM_TEST(Types, 经纬度投影往返一致) {
  const GeoProjector proj(31.2304, 121.4737, 10.0);  // 上海

  const double lat = 31.2350;
  const double lon = 121.4800;
  const double alt = 25.0;

  const Vec3 local = proj.toLocal(lat, lon, alt);
  ADSIM_CHECK_NEAR(local.z, 15.0, 1e-6);

  double lat2 = 0.0, lon2 = 0.0, alt2 = 0.0;
  proj.toGeodetic(local, lat2, lon2, alt2);

  ADSIM_CHECK_NEAR(lat2, lat, 1e-9);
  ADSIM_CHECK_NEAR(lon2, lon, 1e-9);
  ADSIM_CHECK_NEAR(alt2, alt, 1e-6);

  // 100m 量级的位移应被正确还原
  const double dlat = 100.0 / GeoProjector::kEarthRadius;
  const Vec3 north = proj.toLocal(31.2304 + rad2deg(dlat), 121.4737, 10.0);
  ADSIM_CHECK_NEAR(north.y, 100.0, 0.01);
}

ADSIM_TEST(Types, 统计量计算) {
  const std::vector<double> values = {1.0, 2.0, 3.0, 4.0, 5.0};
  const Statistics s = computeStatistics(values);

  ADSIM_CHECK_EQ(s.count, std::size_t(5));
  ADSIM_CHECK_NEAR(s.min, 1.0, 1e-12);
  ADSIM_CHECK_NEAR(s.max, 5.0, 1e-12);
  ADSIM_CHECK_NEAR(s.mean, 3.0, 1e-12);
  ADSIM_CHECK_NEAR(s.stddev, std::sqrt(2.0), 1e-12);

  // 空输入不应崩溃
  const Statistics empty = computeStatistics({});
  ADSIM_CHECK_EQ(empty.count, std::size_t(0));
}

// ===========================================================================
//  内存池
// ===========================================================================

ADSIM_TEST(MemoryPool, 基本分配与归还) {
  MemoryPool pool(256, 16);

  void* a = pool.allocate();
  void* b = pool.allocate();
  ADSIM_CHECK(a != nullptr);
  ADSIM_CHECK(b != nullptr);
  ADSIM_CHECK(a != b);
  ADSIM_CHECK_EQ(pool.blocksInUse(), std::size_t(2));

  pool.deallocate(a);
  pool.deallocate(b);
  ADSIM_CHECK_EQ(pool.blocksInUse(), std::size_t(0));

  // nullptr 归还必须安全
  pool.deallocate(nullptr);
}

ADSIM_TEST(MemoryPool, 块地址被复用) {
  MemoryPool pool(128, 8);

  void* first = pool.allocate();
  pool.deallocate(first);
  void* second = pool.allocate();

  // 同一线程内归还后应立即复用同一块，证明无泄漏且链表工作正常
  ADSIM_CHECK_EQ(first, second);
}

ADSIM_TEST(MemoryPool, 内存块满足最大对齐) {
  MemoryPool pool(37, 8);  // 故意使用非对齐大小，验证向上取整

  std::vector<void*> blocks;
  for (int i = 0; i < 32; ++i) {
    void* p = pool.allocate();
    ADSIM_CHECK(p != nullptr);
    // 必须满足 max_align_t 对齐，才能安全承载任意基本类型
    ADSIM_CHECK_EQ(reinterpret_cast<std::uintptr_t>(p) % alignof(std::max_align_t),
                   std::uintptr_t(0));
    blocks.push_back(p);
  }

  // 块之间不得重叠
  std::set<void*> unique(blocks.begin(), blocks.end());
  ADSIM_CHECK_EQ(unique.size(), blocks.size());

  for (void* p : blocks) pool.deallocate(p);
}

ADSIM_TEST(MemoryPool, 写入读出内容正确) {
  MemoryPool pool(512, 16);

  void* p = pool.allocate();
  auto* data = static_cast<std::uint32_t*>(p);
  for (std::size_t i = 0; i < 128; ++i) {
    data[i] = static_cast<std::uint32_t>(i * 7 + 1);
  }
  for (std::size_t i = 0; i < 128; ++i) {
    ADSIM_CHECK_EQ(data[i], static_cast<std::uint32_t>(i * 7 + 1));
  }
  pool.deallocate(p);
}

ADSIM_TEST(MemoryPool, 超出单Chunk容量触发扩容) {
  MemoryPool pool(64, 8);
  ADSIM_CHECK_EQ(pool.blockCapacity(), std::size_t(0));

  std::vector<void*> blocks;
  for (int i = 0; i < 50; ++i) {
    blocks.push_back(pool.allocate());
  }
  // 50 块 / 每 Chunk 8 块 => 至少 7 个 Chunk
  ADSIM_CHECK(pool.blockCapacity() >= 50);
  ADSIM_CHECK(pool.stats().chunk_count >= 7);

  for (void* p : blocks) pool.deallocate(p);
}

ADSIM_TEST(MemoryPool, 统计信息正确) {
  MemoryPool pool(256, 32);
  pool.resetStats();

  std::vector<void*> blocks;
  for (int i = 0; i < 100; ++i) blocks.push_back(pool.allocate());

  MemoryPool::Stats s = pool.stats();
  ADSIM_CHECK_EQ(s.allocate_calls, std::size_t(100));
  ADSIM_CHECK_EQ(s.blocks_in_use, std::size_t(100));
  ADSIM_CHECK(s.peak_blocks_in_use >= 100.0);
  ADSIM_CHECK_NEAR(s.block_size, 256.0, 1e-9);

  // 全部归还后占用应归零
  for (void* p : blocks) pool.deallocate(p);
  s = pool.stats();
  ADSIM_CHECK_EQ(s.blocks_in_use, std::size_t(0));
  ADSIM_CHECK_EQ(s.deallocate_calls, std::size_t(100));
  ADSIM_CHECK(s.memory_footprint_bytes > 0);
}

ADSIM_TEST(MemoryPool, 线程本地缓存命中率) {
  MemoryPool pool(256, 64);
  pool.resetStats();

  // 反复申请释放，第二次起应命中线程本地缓存
  for (int round = 0; round < 20; ++round) {
    std::vector<void*> blocks;
    for (int i = 0; i < 10; ++i) blocks.push_back(pool.allocate());
    for (void* p : blocks) pool.deallocate(p);
  }

  const MemoryPool::Stats s = pool.stats();
  ADSIM_CHECK_GT(s.thread_cache_hits, std::size_t(0));
  ADSIM_CHECK_GT(s.thread_cache_hit_rate, 0.5);
}

ADSIM_TEST(MemoryPool, 多线程并发分配无泄漏) {
  MemoryPool pool(256, 128);
  pool.resetStats();

  constexpr int kThreads = 8;
  constexpr int kIters = 2000;

  std::atomic<int> errors{0};
  std::vector<std::thread> workers;
  workers.reserve(kThreads);

  for (int t = 0; t < kThreads; ++t) {
    workers.emplace_back([&pool, &errors]() {
      for (int i = 0; i < kIters; ++i) {
        void* a = pool.allocate();
        void* b = pool.allocate();
        if (a == nullptr || b == nullptr || a == b) {
          errors.fetch_add(1);
          continue;
        }
        // 写入以验证内存可安全访问
        *static_cast<std::uint64_t*>(a) = 0xDEADBEEF;
        *static_cast<std::uint64_t*>(b) = 0xCAFEBABE;
        pool.deallocate(a);
        pool.deallocate(b);
      }
    });
  }

  for (std::thread& w : workers) w.join();

  ADSIM_CHECK_EQ(errors.load(), 0);
  // 所有线程已退出，其本地缓存已归还，占用必须回到 0
  ADSIM_CHECK_EQ(pool.blocksInUse(), std::size_t(0));
}

ADSIM_TEST(MemoryPool, 池化块RAII句柄) {
  MemoryPool pool(128, 8);
  pool.resetStats();

  {
    PooledBlock block(pool);
    ADSIM_CHECK(block.valid());
    ADSIM_CHECK_EQ(pool.blocksInUse(), std::size_t(1));
    std::memset(block.data(), 0xAB, block.capacity());
  }
  // 离开作用域自动归还
  ADSIM_CHECK_EQ(pool.blocksInUse(), std::size_t(0));
}

ADSIM_TEST(MemoryPool, 池化块移动语义) {
  MemoryPool pool(128, 8);

  {
    PooledBlock a(pool);
    void* raw = a.data();
    PooledBlock b(std::move(a));

    ADSIM_CHECK(!a.valid());   // 被移动后失效
    ADSIM_CHECK(b.valid());
    ADSIM_CHECK_EQ(b.data(), raw);
    ADSIM_CHECK_EQ(pool.blocksInUse(), std::size_t(1));
  }
  ADSIM_CHECK_EQ(pool.blocksInUse(), std::size_t(0));
}

ADSIM_TEST(MemoryPool, 注册表按大小共享实例) {
  MemoryPool& a = MemoryPoolRegistry::get(300);
  MemoryPool& b = MemoryPoolRegistry::get(256);  // 300 向上取整即 512? 否——见下

  // 256 与 300 分别对齐到各自 2 的幂，应落在不同池；二次取用必须命中同一实例
  MemoryPool& c = MemoryPoolRegistry::get(300);
  ADSIM_CHECK_EQ(&a, &c);
  ADSIM_CHECK(&a != &b);
}

// ===========================================================================
//  线程池
// ===========================================================================

ADSIM_TEST(ThreadPool, 任务返回结果) {
  ThreadPool pool(4);

  auto f1 = pool.enqueue([]() { return 42; });
  auto f2 = pool.enqueue([](int a, int b) { return a + b; }, 3, 4);

  ADSIM_CHECK_EQ(f1.get(), 42);
  ADSIM_CHECK_EQ(f2.get(), 7);
}

ADSIM_TEST(ThreadPool, 异常被正确传播) {
  ThreadPool pool(2);

  auto f = pool.enqueue([]() -> int { throw std::runtime_error("预期异常"); });
  ADSIM_CHECK_THROWS(f.get(), std::runtime_error);
}

ADSIM_TEST(ThreadPool, 大量任务全部执行) {
  ThreadPool pool(8);
  constexpr int kTasks = 5000;

  std::atomic<int> counter{0};
  std::vector<std::future<void>> futures;
  futures.reserve(kTasks);

  for (int i = 0; i < kTasks; ++i) {
    futures.emplace_back(pool.enqueue([&counter]() { counter.fetch_add(1); }));
  }
  for (auto& f : futures) f.get();

  ADSIM_CHECK_EQ(counter.load(), kTasks);
  pool.waitIdle();
  ADSIM_CHECK_EQ(pool.pendingTasks(), std::size_t(0));
}

ADSIM_TEST(ThreadPool, 并行循环结果正确) {
  ThreadPool pool(8);
  constexpr std::size_t kN = 100000;

  std::vector<int> data(kN, 0);
  pool.parallelFor(0, kN, 512, [&data](std::size_t i) { data[i] = static_cast<int>(i * 2); });

  long long sum = 0;
  bool correct = true;
  for (std::size_t i = 0; i < kN; ++i) {
    sum += data[i];
    if (data[i] != static_cast<int>(i * 2)) correct = false;
  }

  ADSIM_CHECK(correct);
  ADSIM_CHECK_EQ(sum, static_cast<long long>(kN) * (static_cast<long long>(kN) - 1));
}

ADSIM_TEST(ThreadPool, 并行循环边界情况) {
  ThreadPool pool(4);

  // 空区间不应执行任何元素
  int calls = 0;
  pool.parallelFor(10, 10, 1, [&calls](std::size_t) { ++calls; });
  ADSIM_CHECK_EQ(calls, 0);

  // 逆序区间不应执行
  pool.parallelFor(10, 5, 1, [&calls](std::size_t) { ++calls; });
  ADSIM_CHECK_EQ(calls, 0);

  // 区间小于粒度时应回退串行执行
  pool.parallelFor(0, 3, 100, [&calls](std::size_t) { ++calls; });
  ADSIM_CHECK_EQ(calls, 3);
}

ADSIM_TEST(ThreadPool, 停机后拒绝新任务) {
  ThreadPool pool(2);
  pool.shutdown();
  ADSIM_CHECK_THROWS(pool.enqueue([]() { return 1; }), std::runtime_error);
}

// ===========================================================================
//  无锁环形缓冲
// ===========================================================================

ADSIM_TEST(RingBuffer, 先进先出语义) {
  SpscRingBuffer<int> ring(8);

  for (int i = 0; i < 5; ++i) {
    ADSIM_CHECK(ring.push(i));
  }
  ADSIM_CHECK_EQ(ring.size(), std::size_t(5));

  for (int i = 0; i < 5; ++i) {
    int value = -1;
    ADSIM_CHECK(ring.pop(value));
    ADSIM_CHECK_EQ(value, i);
  }
  ADSIM_CHECK(ring.empty());
}

ADSIM_TEST(RingBuffer, 容量向上取整为2的幂) {
  SpscRingBuffer<int> ring(5);
  ADSIM_CHECK_EQ(ring.capacity(), std::size_t(8));

  SpscRingBuffer<int> ring2(16);
  ADSIM_CHECK_EQ(ring2.capacity(), std::size_t(16));
}

ADSIM_TEST(RingBuffer, 缓冲区满时丢弃并计数) {
  SpscRingBuffer<int> ring(4);  // 容量 4

  for (int i = 0; i < 4; ++i) {
    ADSIM_CHECK(ring.push(i));
  }
  // 第 5 个应被丢弃
  ADSIM_CHECK(!ring.push(99));
  ADSIM_CHECK(!ring.push(100));
  ADSIM_CHECK_EQ(ring.droppedCount(), std::uint64_t(2));

  int value = -1;
  ring.pop(value);
  ADSIM_CHECK_EQ(value, 0);

  // 腾出空间后可继续写入
  ADSIM_CHECK(ring.push(42));
}

ADSIM_TEST(RingBuffer, 批量读取) {
  SpscRingBuffer<int> ring(16);
  for (int i = 0; i < 10; ++i) ring.push(i);

  int out[4] = {0, 0, 0, 0};
  const std::size_t n = ring.popBatch(out, 4);

  ADSIM_CHECK_EQ(n, std::size_t(4));
  for (int i = 0; i < 4; ++i) ADSIM_CHECK_EQ(out[i], i);
  ADSIM_CHECK_EQ(ring.size(), std::size_t(6));
}

ADSIM_TEST(RingBuffer, 生产者消费者并发无丢失) {
  constexpr std::uint64_t kCount = 200000;
  SpscRingBuffer<std::uint64_t> ring(1024);

  std::atomic<bool> producer_done{false};
  std::uint64_t expected = 0;
  std::uint64_t received = 0;
  bool sequence_ok = true;

  std::thread producer([&]() {
    for (std::uint64_t i = 0; i < kCount; ++i) {
      // 缓冲满时自旋重试，保证不丢数据
      while (!ring.push(i)) {
        std::this_thread::yield();
      }
    }
    producer_done.store(true, std::memory_order_release);
  });

  std::thread consumer([&]() {
    while (received < kCount) {
      std::uint64_t value = 0;
      if (ring.pop(value)) {
        if (value != expected) sequence_ok = false;
        ++expected;
        ++received;
      } else if (producer_done.load(std::memory_order_acquire) && ring.empty()) {
        break;
      } else {
        std::this_thread::yield();
      }
    }
  });

  producer.join();
  consumer.join();

  ADSIM_CHECK_EQ(received, kCount);
  ADSIM_CHECK(sequence_ok);  // 顺序必须严格保持
  // droppedCount 统计的是"失败的写入尝试"而非丢失数据：生产者自旋重试时
  // 该值会大于 0，但上面的 received 已证明数据一条未丢。
}

// ===========================================================================
//  性能剖析
// ===========================================================================

ADSIM_TEST(Profiler, 记录与聚合) {
  Profiler& profiler = Profiler::instance();
  profiler.reset();

  profiler.record("stage_a", 10.0);
  profiler.record("stage_a", 20.0);
  profiler.record("stage_b", 5.0);

  const auto entries = profiler.entries();
  ADSIM_CHECK_EQ(entries.size(), std::size_t(2));

  // 按总耗时降序排列，stage_a 应在前
  ADSIM_CHECK_EQ(entries[0].name, std::string("stage_a"));
  ADSIM_CHECK_EQ(entries[0].count, std::size_t(2));
  ADSIM_CHECK_NEAR(entries[0].total_ms, 30.0, 1e-9);
  ADSIM_CHECK_NEAR(entries[0].mean_ms, 15.0, 1e-9);
  ADSIM_CHECK_NEAR(entries[0].min_ms, 10.0, 1e-9);
  ADSIM_CHECK_NEAR(entries[0].max_ms, 20.0, 1e-9);

  ADSIM_CHECK_EQ(entries[1].name, std::string("stage_b"));
  ADSIM_CHECK_NEAR(entries[1].percentage, 100.0 * 5.0 / 35.0, 1e-6);

  ADSIM_CHECK(!profiler.report().empty());
  profiler.reset();
  ADSIM_CHECK_EQ(profiler.entries().size(), std::size_t(0));
}

ADSIM_TEST(Profiler, 作用域计时器) {
  Profiler::instance().reset();

  {
    ADSIM_PROFILE_SCOPE("scoped_stage");
    volatile double x = 0.0;
    for (int i = 0; i < 10000; ++i) x += i * 0.5;
  }

  const auto entries = Profiler::instance().entries();
  ADSIM_CHECK_EQ(entries.size(), std::size_t(1));
  ADSIM_CHECK_EQ(entries[0].name, std::string("scoped_stage"));
  ADSIM_CHECK_EQ(entries[0].count, std::size_t(1));
  ADSIM_CHECK(entries[0].total_ms >= 0.0);
  Profiler::instance().reset();
}
