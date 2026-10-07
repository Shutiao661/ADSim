# 更新日志

本文件记录各版本的重要变更。格式参考 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，
版本号遵循 [语义化版本](https://semver.org/lang/zh-CN/)。

---

## [1.0.0] — 2026-10-07

首个完整版本。

### 数据管道

- 自研 ROS Bag v2.0 格式读写器
  - 流式读取，内存占用与文件大小无关
  - 零拷贝消息视图（回调期间有效）
  - 支持 bz2 / lz4 压缩 Chunk，压缩库自动探测、缺失时优雅降级
  - 写入器补写完整索引区，产出可被 ROS 官方工具读取的 bag
  - 格式已与 ROS 官方 `roslz4` 及 Python 标准库 `bz2` **双向交叉验证**
- `MessageCodec`：`NavSatFix` / `Odometry` / `PointCloud2` 的真实 ROS 序列化编解码
- `TimeAligner`：批量与流式两种对齐器；航向插值走最短路径
- `PointCloudFilter`：距离裁剪 → 车体剔除 → 体素降采样 → 统计离群点去除 → RANSAC 地面分离
- `GpsFilter`：常速度模型卡尔曼滤波 + 卡方门控 + 自适应观测噪声
- `DataPipeline`：内存池 + 无锁环形缓冲 + 线程池三层编排，天然背压

### 规划决策

- `Fsm`：驾驶模式状态机，转移带守卫与动作
- `BehaviorTree`：Sequence / Selector / Parallel + 五种装饰节点
- `DecisionMaker`：状态机（模式层）× 行为树（任务层）双层决策
- `geometry/Delaunay`：Bowyer-Watson 三角剖分 + Voronoi 路线图
- `geometry/Spline`：三次 B 样条（De Boor）+ 最小二乘拟合 + 曲率约束平滑
- `optimizer/NonlinearSolver`：自研 LM / 高斯-牛顿求解器，接口与 Ceres 同构
- `optimizer/PathOptimizer`：四项代价模型 + 外层约束迭代
- `PathPlanner`：车道采样 / 换道（五次多项式）/ **B 样条与数值优化双平滑后端**
- `SpeedPlanner`：曲率限速 + IDM 跟车 + 加速度/jerk 可行性修正
- `PlanningPolicy`：串联全流程，实现 `IPolicy` 接入仿真

### 仿真内核

- `VehicleModel`：运动学自行车模型，圆弧精确积分
- `World`：车道网络、静态障碍、动态物体
- `SimEngine`：固定步长推进、OBB 碰撞检测（SAT）、TTC 计算、安全事件记录
- `Scenario`：五个危险工况 + 场景注册表

### 可视化与打标

- `SvgCanvas`：零依赖 SVG 画布，轨迹按曲率着色
- `SimVisualizer`：俯视图、时序图、HTML 报告、回放对比
- `QtVisualizer`：Qt 交互式复盘前端（可选依赖）
- `LlmClient` / `ScenarioTagger`：规则打标 + LLM 语义增强，含离线桩实现

### 可选依赖

| 开关 | 依赖 | 状态 |
|---|---|---|
| `-DADSIM_WITH_COMPRESSION=ON` | bz2 / lz4 | 自动探测，默认开启 |
| `-DADSIM_WITH_CERES=ON` | Ceres Solver | 已实测（比内置求解器快约 30 倍） |
| `-DADSIM_WITH_QT=ON` | Qt5 Widgets | 已实测 |
| `-DADSIM_WITH_CURL=ON` | libcurl | 已实测 |
| `-DADSIM_WITH_CARLA=ON` | CARLA 0.9.x | 按真实 API 编写，无环境未验证 |
| `-DADSIM_WITH_ROS=ON` | ROS | 按真实 API 编写，无环境未验证 |

### 工程

- 276 项单元测试，自带零依赖测试框架
- 5 种构建配置（默认 / 无压缩 / Ceres / Qt / 全开）全部零告警通过
- 中文注释 + 完整设计文档（架构、算法、参数标定依据）
- CI：GitHub Actions，覆盖三种开关组合的构建、测试与冒烟验证

### 实测数据

| 指标 | 数值 |
|---|---|
| 数据管道吞吐（未压缩） | 329 MB/s |
| 数据管道吞吐（lz4） | 251 MB/s |
| 点云降噪多核加速比 | 5.26×（16 线程） |
| 路径平滑曲率下降 | 87%（0.417 → 0.054 1/m） |
| 决策层对照（三场景） | 完整策略碰撞 0，纯跟踪基准全部碰撞 |

---

## 已知限制

- **Voronoi 绕障尚未接入主流程**：缺少产生障碍物点集的感知模块，
  `PathPlanner` 的绕障分支在端到端流程中始终不启用（详见 README）
- **未在真实 rosbag 上验证**：压缩格式已与独立实现交叉验证，
  但尚未读取过 `rosbag record` 实际录制的文件
- **无预测模块**：其他交通参与者由解析式脚本驱动，行为决策的输入是仿真真值
- **感知仅到预处理**：点云只做降噪，无聚类/检测/跟踪
