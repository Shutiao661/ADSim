# 架构设计

## 分层与依赖

工程按"依赖只能向下"的原则分为五层。箭头表示编译期依赖方向，
**没有任何反向依赖**——这是各模块可独立测试的前提。

```
                    ┌──────────────────────────────────┐
   应用层           │  adsim_cli                       │
                    └────────────┬─────────────────────┘
                                 │
                    ┌────────────▼─────────────────────┐
   策略层           │  PlanningPolicy                  │
                    │  ScenarioTagger   SimVisualizer  │
                    └────────────┬─────────────────────┘
                                 │
        ┌────────────────────────┼────────────────────────┐
        │                        │                        │
┌───────▼───────┐      ┌─────────▼────────┐     ┌─────────▼────────┐
│  规划决策      │      │   仿真内核        │     │   数据管道        │
│  Fsm          │      │   VehicleModel   │     │   RosBagReader   │
│  BehaviorTree │      │   World          │     │   TimeAligner    │
│  PathPlanner  │      │   SimEngine      │     │   PointCloudFilter│
│  SpeedPlanner │      │   Scenario       │     │   GpsFilter      │
│  geometry/*   │      │                  │     │   DataPipeline   │
│  optimizer/*  │      │                  │     │                  │
└───────┬───────┘      └─────────┬────────┘     └─────────┬────────┘
        │                        │                        │
        └────────────────────────┼────────────────────────┘
                                 │
                    ┌────────────▼─────────────────────┐
   基础设施层       │  Types  MemoryPool  ThreadPool    │
                    │  RingBuffer  Logger  Profiler     │
                    └──────────────────────────────────┘
```

**为什么规划层不依赖仿真层？**
`PathPlanner` 依赖 `World`（车道几何），但 `World` 属于仿真层。这个依赖是
刻意的：车道网络是**几何事实**而非仿真概念，规划器需要它来生成路径。
反过来，仿真层完全不感知规划层的存在——它只通过 `IPolicy` 接口调用算法。
这种单向依赖让"用同一套规划器跑仿真"和"用同一个仿真跑不同规划器"都成立。

---

## 数据流

### 离线路径：路测数据 → 清洗 → 回放

```
bag 文件
   │
   │ RosBagReader::forEachMessage（流式，回调拿到零拷贝视图）
   ▼
[读取线程] ──拷贝入内存池──► SpscRingBuffer ──► [派发] ──► ThreadPool worker
                                                              │
                                                              │ 解码 + 降噪
                                                              ▼
                                                       收集区（按时间排序）
                                                              │
                                                              ▼
                                              GpsFilter（剔跳点 + 平滑）
                                                              │
                                                              ▼
                                              TimeAligner（多传感器对齐）
                                                              │
                                                              ▼
                                                   RosBagWriter（落盘）
```

**为什么读取线程要拷贝？**
`RosBagReader` 按 Chunk 载入并复用同一块缓冲区，回调拿到的 `MessageView::data`
只在回调期间有效。要跨线程传递就必须拷贝——这里的取舍是"用一次拷贝换取
与文件大小无关的内存占用"，对 100GB 级数据这是唯一可行的方案。
拷贝走内存池，避免每条消息一次 `malloc`。

**背压如何形成？**
环形缓冲容量固定。读取线程 `push` 失败时自旋等待，自然形成背压，
不需要额外的流量控制逻辑。实测中把缓冲槽位从 256 降到 4，
只是变慢，不会丢数据（见 `test_pipeline.cpp` 的背压用例）。

### 在线路径：仿真 → 决策 → 规划 → 控制

```
SimEngine::step
   │
   ├─► Scenario::update          场景脚本更新其他交通参与者
   │
   ├─► IPolicy::computeCommand   调用被测算法
   │      │
   │      ├─ World::project           定位自车所在车道
   │      ├─ DecisionMaker::perceive  提取前车/邻道/路口信息
   │      ├─ DecisionMaker::decide    FSM 定模式 + 行为树定动作
   │      ├─ PathPlanner::plan        车道采样/换道/Voronoi 绕障 + 数值优化
   │      ├─ SpeedPlanner::plan       曲率限速 + 跟车 + 可行性修正
   │      └─ 纯跟踪 + 纵向控制         输出加速度与转角
   │
   ├─► VehicleModel::stepByAcceleration   推进物理
   │
   └─► 采集指标 / 判定安全事件 / 记录
```

---

## 关键接口

### `IPolicy` —— 算法与仿真的唯一契约

```cpp
class IPolicy {
 public:
  struct Command {
    double acceleration;
    double steering;
    bool valid;   // false 表示"保持上一步指令"
  };
  virtual Command computeCommand(const VehicleState& ego, const World& world, double dt) = 0;
  virtual std::string name() const = 0;
  virtual void reset() {}
  virtual std::vector<Vec2> lastPlannedPath() const { return {}; }
};
```

仿真内核不认识任何具体算法，只认识这个接口。这带来了两个直接好处：

1. **横向对比**：`adsim_cli scenario <名称> --policy=cruise` 用定速巡航策略跑同一场景，
   与 `--policy=planning` 的结果对照，可以量化决策与优化带来的收益。
2. **回归测试**：算法改动后重跑场景库，比对安全指标是否退化，
   不需要人工介入，也不需要真实车辆。

### `ResidualBlock` —— 与 Ceres 同构的代价函数接口

```cpp
class ResidualBlock {
 public:
  virtual int residualDimension() const = 0;
  virtual std::vector<int> parameterBlockSizes() const = 0;
  virtual void evaluate(const double* const* params, double* residuals,
                        double* jacobian) = 0;
};
```

**为什么支持多个参数块？**
一个平滑残差天然同时依赖相邻三个路径点。如果接口只允许单个参数块，
就只能把它拆成三个独立残差，而这样会丢失点与点之间的耦合信息
（雅可比里的交叉项全部为零），优化效果会显著变差。

### `MessageView` —— 零拷贝消息视图

```cpp
struct MessageView {
  std::uint32_t connection_id;
  Timestamp stamp;
  const ConnectionInfo* connection;
  const std::uint8_t* data;   // 仅在回调期间有效
  std::size_t size;
};
```

生命周期写在注释里也写在这里：`data` 指向读取器内部的 Chunk 缓冲区，
回调返回后即失效。这是为吞吐量做的取舍——100GB 级数据下逐条拷贝不可接受。

---

## 内存与并发模型

| 组件 | 解决的问题 | 关键设计 |
|------|-----------|---------|
| `MemoryPool` | 高频消息的逐条 `malloc` 开销 | Chunk 批量申请 + 中心空闲链表 + **线程本地缓存** |
| `SpscRingBuffer` | 读取与消费速度不匹配 | 无锁 SPSC，head/tail 分处不同 cache line 避免伪共享 |
| `ThreadPool` | 点云降噪的计算密集 | 任务队列 + 条件变量；支持 move-only 任务参数 |

**内存池的线程本地缓存为什么必要？**
16 线程下所有分配都走中心链表会让互斥锁成为瓶颈。线程本地缓存一次批量取块，
把加锁频率降低到 1/64。实测命中率见 `adsim_cli bench`。

**踩过的坑：线程本地缓存不能用对象地址做键。**
池对象析构后其栈/堆地址可能被新池复用，用地址做键会命中已失效的缓存，
进而返回指向已释放 Chunk 的悬垂块。改用全局唯一递增编号后解决。
这个缺陷在单元测试里表现为"新池的 blockCapacity 不为 0"。

---

## 可复现性

仿真内核**不使用任何随机数**。相同输入必得相同输出，这是回归测试比对的前提。
`test_sim.cpp` 中有专门用例验证同一场景跑两次轨迹逐点完全相同。

需要随机性的地方（点云合成、路径噪声）一律用固定种子的 `std::mt19937`，
且把种子作为显式参数而非全局状态。

---

## 可选依赖的接入方式

重型依赖（CARLA / Ceres / Qt / ROS / libcurl）全部通过 CMake 开关控制，
默认关闭。关闭时：

- 对应的 `.cpp` 不参与编译（在根 `CMakeLists.txt` 中从 glob 结果里剔除）
- 调用方通过 `#if defined(ADSIM_HAS_XXX)` 分支降级到可用实现
- 头文件仍然可以被包含，因此调用方代码不需要预处理器条件编译

例如路径优化：

```cpp
#if defined(ADSIM_HAS_CERES)
  ceresOptimizePath(coords, initial_path, options_, &local_report);
#else
  // 内置 LM 求解器，代价函数定义完全相同
#endif
```

代价函数（平滑/贴合/长度/曲率铰链）在两个后端中逐个对应，
切换后端不改变问题的语义，只影响收敛速度与数值精度。
