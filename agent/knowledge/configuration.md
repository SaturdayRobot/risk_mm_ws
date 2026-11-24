# 当前配置与实际加载

核对日期：2026-11-24。本页数值来自 **src 源配置**，不是运行实测或安装配置的保证。seed42 DEV-04 选择构建后，`src` 与 `install` patrol YAML 哈希一致（SHA256 `8efb9f54b41d4fe35efc5866f6b7cd3a97f1efe049775030ba842bec7f15174c`），优化预算均为 100 ms；`remani.yaml` 哈希也一致（SHA256 `0d32414a81b6585beb9e3921b5111bd72c4835b46fc7cdc737e523f720d7bfee`），seed 均为 42。seed42 搜索路径和 MINCO 点复放一致，但 100 ms 墙钟优化停止点不同、优化候选略有差异；两次都通过首段采样验证。DEV-09 已修正 MINCO 种子时间与返回变量同步：固定 seed42 首段无回退，完整目标梯度相对误差 9.44e-6，公共采样通过。正常路线首段规划 260.1 ms、验证链 101.7 ms，第二段仍因转向率拒绝；500 ms 诊断也未通过。DEV-10 正构建与公共模型一致的四轮软约束，尚未验证该新版本；完整路线仍未通过。旧会话的运行输入仍按各自启动快照解释，不能由新配置追认。

## 配置责任

除另注明，下表 YAML 位于 `src/mm_bringup/config/`。

| 来源 | 负责内容 | 不应混入的责任 |
| --- | --- | --- |
| [robot.yaml](../../src/mm_bringup/config/robot.yaml) | 模型名、六关节顺序、唯一 stow 和展示初态 | 模式姿态库、动态限速 |
| [manual_control.yaml](../../src/mm_bringup/config/manual_control.yaml) | 控制方向标定、GUI 默认速度、手动限速/超时 | 公共规划安全结论 |
| [remani.yaml](../../src/mm_bringup/config/remani.yaml) | 原搜索/优化、显式可复现随机 seed、软代价、ESDF 与四轮模型扩展 | 执行器硬限制、应用巡检调度 |
| [coupled_execution.yaml](../../src/mm_bringup/config/coupled_execution.yaml) | 公共验证硬限制、单目标显式时间策略 | 原算法收敛保证 |
| [patrol.yaml](../../src/mm_bringup/config/patrol.yaml) | 世界身份、显式巡检点、通过速度、预算、前进约束 | 自动模式切换、已验证路线声明 |
| [goal_presets.yaml](../../src/mm_bringup/config/goal_presets.yaml) | GUI 单目标任务预设，world/base_control/rad | 自动运动、ARM_FIRST 姿态库 |
| [simulation_collision.yaml](../../src/mm_bringup/config/simulation_collision.yaml) | 仿真接触代理、轮胎、摩擦 | 修改原 URDF collision 网格 |
| [ros2_control.yaml](../../src/mm_bringup/config/ros2_control.yaml) | 500 Hz、六臂位置、四转向位置、四轮速度控制器 | 轨迹规划或模式选择 |
| [supervisor.yaml](../../src/mm_bringup/config/supervisor.yaml) | 研究内核的风险/策略/门控参数 | 已完成在线风险切换的证据；当前 ROS 链未接通 |
| [MoveIt joint_limits.yaml](../../src/rangermini_moveit_config/config/joint_limits.yaml) | 独立收臂服务的关节限速 | 覆盖 COUPLED 的公共限速 |

## 加载顺序和身份

[gazebo.launch.py](../../src/mm_bringup/launch/gazebo.launch.py) 从 **install/share** 查找资源，参数按 `remani.yaml → coupled_execution.yaml → 启动组装字典` 注入。组装字典包含 URDF 导出的安装/轴/限位/碰撞点和轮位、方向标定、文件路径、模型/世界指纹及 use_sim_time；同名后项覆盖前项。

手动/GUI 参数由 C++ 按已安装配置读取。GUI 提交三项速度 `[底盘 m/s, 偏航 rad/s, 关节 rad/s]`；后端核对并冻结到任务，零值使用配置默认，非零不能超过硬上限。修改输入作废旧 Execute，执行期间不能在线改变已验证轨迹的限速。

核对实际效果要区分：`src` 的意图 → `install` 的已安装值 → `reports/<session>/inputs` 与任务目录的冻结输入 → 进程启动参数/实际反馈。只改源码 YAML 不会更新已运行节点；只看某份 YAML 也不能确定有效值。

`model_signature` 由 URDF、robot/remani/simulation_collision/coupled_execution/manual_control 配置、SRDF 和导出数据组成。当前 patrol 输入另在任务目录保存快照；模型/地图版本不匹配时旧候选与执行代号不可复用。

最近选择构建已令源/安装 patrol YAML 一致，但没有当前运行进程或新的启动快照；固定失败输入应从历史实测状态重建并单独标记新的配置快照。

## 不同层的速度限制

| 参数 | REMANI 源优化配置 | 公共验证硬上限 | 手动 GUI 上限 / 默认 |
| --- | --- | --- | --- |
| 底盘线速度 | 从模型/原轮约束派生，不等同 GUI 速度 | 1.0 m/s，任务可更低 | 0.8 / 0.4 m/s |
| 偏航速度 | 原平坦输出切线推导与可行性代价 | 1.2 rad/s，任务可更低 | 1.0 / 0.6 rad/s |
| 臂关节速度 | 0.35 rad/s | 0.8 rad/s，任务可更低 | 0.8 / 0.5 rad/s |
| 臂关节加速度 | 0.7 rad/s² | 1.6 rad/s² | 无单独 GUI 输入 |
| 轮速 | 原差速软参数 3.5 rad/s | 当前四轮 10 rad/s | 8 rad/s |
| 轮加速度 | 软参数 4.0 rad/s²，预留余量 | 当前四轮 5.0 rad/s² | 4.0 rad/s² |
| 转向关节速率 | 四轮可行性另查 | pi/2 rad/s | 1.2 rad/s |

URDF 与上述三层不同：臂 3.14 rad/s、轮 10 rad/s、转向 2 rad/s。MoveIt 单独收臂服务为 0.35 rad/s、0.7 rad/s²。不要把某层的低值或通过结果移作另一层的保证。

手动控制频率 50 Hz，命令超时 0.30 s，反馈超时 0.50 s；方向标定为 pi。耦合驱动另有 0.15 s ROS 时间命令看门狗，见 `src/mm_bringup/src/sim_control.cpp`。

## 地图、碰撞与原算法参数

| 参数 | 当前源值 | 含义 |
| --- | --- | --- |
| grid_map.resolution | 0.10 m | 距离场分辨率 |
| grid_map.obstacles_inflation | 0.05 m | 按体素离散膨胀，非精确连续 5 cm |
| 底盘采样半径 / 臂采样半径 | 0.04 / 0.03 m | URDF 导出与 MMConfig |
| optimization.safe_margin | 0.13 m | 源优化底盘障碍代价在 0.17 m 激活；此前 0.03 的旧值不再适用 |
| safe_margin_mani / self_safe_margin | 0.02 / 0.01 m | 臂环境与自碰撞余量 |
| ground_safe_dis / ground_safe_margin | 0.03 / 0.02 m | 臂最低高度与地面代价 |
| validation.collision_padding | 0.01 m | 公共完整网格外部 padding |
| search.min_turning_radius | 0.30 m | 搜索基础参数 |
| optimization.four_wheel_min_turning_radius | 0.30 m | 明确增加的四轮曲率软项，代码默认 0 关闭 |
| search.max_search_time | 2.0 s | 搜索预算，不能当作实际耗时或成功保证 |
| search.max_mani_search_time | 0.08 s | 机械臂采样搜索配置 |
| optimization.weight_obstacle / weight_manipulator_self | 20000 / 1000 | 保留的碰撞代价权重 |
| weight_base_feasibility / weight_manipulator_feasibility / weight_time | 100 / 150 / 1 | 可行性与时间权重 |
| constrain_points_perPiece / dense_sample_resolution | 8 / 4 | 原优化采样相关配置，不为提速随意削减 |

底盘源几何阈值为 `0.04 + 0.10 + (safe ? 0.13 : 0)` m；硬源检查 safe=false 时仍为 0.14 m。优化代价为 `0.04+0.13=0.17` m。两条路径不同，提前代价后转弯仍曾失败，当前不是已解决的效果。

REMANI 来源及新增项见 [provenance](../../docs/provenance.md)。普通单目标保留源差速硬/软运动学检查；巡检显式保留源几何检查，硬运动学由公共四轮验证负责。DEV-10 源码将巡检软代价同步为同几何的四轮速度、轮加速度与转向率（此前仍为差速软项）；软阈值为 3.5 rad/s、4.0 rad/s²、公共转向率上限的 0.9，基座线速和原曲率项保留。四轮轮位来自 controller.wheel_positions，按 planning frame 重表达；未提供当前几何则拒绝。13 包已构建，固定第二段四轮梯度/公共模型误差 2.70e-9/8.83e-9、完整目标梯度误差 4.02e-6；100 ms 候选仍未通过转向率，硬限制未改。这是预先声明的固定模型选择，禁止拒绝后隐式切换。

## 巡检与时间处理

用户最新要求允许几百毫秒响应，本轮成功任务完整规划与验证以 1 秒内为工程目标；不把原 100 ms 优化预算当作整体响应上限。机械臂主动协同要求记录于 agent.md；当前生产关节限速/巡检目标尚未改变。显式离线 `test_coupled_planning.py --check-initial-gradient` 设置 `optimization.check_initial_gradient=true`，在优化计时开始前核对全部初始变量的解析/有限差分梯度，日志含独立诊断耗时；默认 false，生产 YAML 不启用。

patrol 源配置：通过速度 0.15 m/s、优化预算 100 ms、forward_only=true、Dubins shot 半径 0.40 m；三个目标见 [模型与世界](robot_model.md)。REMANI 使用的 `search.random_seed` 位于 remani.yaml，源码与安装当前均为 42。省略此参数会保留随机设备初始化。DEV-06 首段显式加入 0.2 s、0.002 m 的 C² 起步曲线到达 REMANI 非奇异 0.02 m/s 切向边界；起点 p/v/a 约束吻合，DEV-09 修复后 seed42 首段已通过公共采样；全路线的转弯段仍未通过。中间点配置有非零通过速度，最终返回任务记录的实测起态；全路线仍未验收。

单目标时间策略由 coupled_execution.yaml 明确启用 `local`，最大倍数 8、代码局部最小 0.5、端点至少 1；`uniform` 仅显式选择，无失败回退。巡检使用独立 `through` 映射，端点一阶时间导数为 1、二阶为 0，最大段内放慢倍数 **3，当前写在 patrol.cpp**。两者不能混用来破坏移动交接导数。

DEV-14 将 through 过渡窗口从固定 T/4 改为 `min(T/4,max(0.2,2*max(端点线速)/(wheel_acceleration*wheel_radius)))` 秒，当前 0.15 m/s 移动边界对应 0.6 s；四节点 C2 形式、端点导数、最大倍率与 2 ms/20 ms 验证均保留。窗口是代码中的物理尺度初始化，不是可行性保证，待构建/完整路线验证。

预算到达可产生待检查候选，但不代表收敛或可执行。计时须包含搜索/优化/源检查，并另外记录时间映射、公共验证及端到端墙钟。

DEV-09 源码修正 MINCO 初始时间：关节限速所需时间和首末 1.5 倍余量真正传入生成器；非停止通过目标的最终片采用弦长/指定末端切向速度，保留关节时间下限，避免套用搜索停止轮廓。优化返回后按 L-BFGS 返回的变量重建多项式；它是同一候选状态同步，不重新搜索。已构建，固定 seed42 首段通过完整采样与梯度核对（最大相对误差 9.44e-6），回退消失；完整路线待验证。

DEV-11/13 对四轮最终转弯目标（通过或停车切向）准备单个 MINCO seed：末内点与目标 heading 相差超过 1e-3 rad 时，先合并末端过密点，末片弦长至少覆盖 `four_wheel_min_turning_radius`（当前 0.30 m、至少保留两片），合并搜索时间并保留 1.5 倍末片余量；再沿粗 MINCO 曲线等时细分末片、恢复原优化片数和每片 K=8 的总代价样点数。A* 输出、目标 p/v/a、源/公共/时间硬验证间隔不变，直行保留 DEV-09。DEV-12 版正常离线三个巡检点已通过、返航未通过；覆盖停车切向终点的 DEV-13 待构建。这是初始化规则，不是安全容差。

DEV-13 校正 MMConfig 低速几何导数：原 yaw/旋转函数的方向在 norm>=1e-6 保持 atan2，因此其导数使用实际速度而非抬高到 0.02 m/s 的速度；既有 norm<1e-6 的常量朝向分支使用零导数。模型几何、方向标定、非奇异速度参数与硬限不改变。新增 probe 独立低速有限差分核对，待构建验证。

## 尚未配置化的代码常量

这些值目前在 C++ 中，不应声称都能从 YAML 调节。改动时同时更新来源和本表。

| 来源 | 当前约束 |
| --- | --- |
| `risk_mm_runtime/include/risk_mm_runtime/validation.hpp` | 公共网格采样间隔 0.02 s；缺少覆盖时的臂默认值仍为 0.35/0.7，正常 launch 覆盖为 0.8/1.6 |
| `.../execution.hpp` | 到位容差 XY 0.04 m、yaw 0.05 rad、臂 0.03 rad；跟踪门限 0.10 m/0.35 rad/0.08 rad；对齐 8 s、终点收敛 5 s |
| `.../src/execution.cpp` | 队列交接 p/v/a 容差 1e−4 / 1e−4 / 0.002；实测停车连续确认 3 次 |
| `.../src/patrol.cpp` | 巡检点位置 0.08 m、臂 0.04 rad；持续低于 0.005 m/s 超过 0.2 s 记一次停车；新段起始 jerk 为零、保持 C2 |
| `.../src/runtime.cpp` | 场景租期 0.5 s、执行 tick 20 ms |
| `scripts/validation/test_patrol.py` | 验收还要求通过速度>0.02 m/s、返航误差<0.04 m；默认候选规划最大 500 ms，并非完整端到端目标 |
| `src/planning/remani_adapter/src/planner.cpp` | 静止 forward-only 起步曲线时长 0.2 s、位移为 REMANI 非奇异速度乘时长的一半（当前 0.002 m）；不是 YAML 参数 |

`remani.yaml` 中的 `fsm.global_plan` / `fsm.planning_horizon` 仍被原搜索适配读取，不代表旧 FSM 已编译或内部模式决策已启用。`ros2_control.yaml` 的“执行适配器尚待接通”注释是历史过期文字，当前已有单目标执行证据；判断接入状态要看代码和进度。
