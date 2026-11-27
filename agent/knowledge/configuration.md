# 当前配置与实际加载

核对日期：2026-11-26。本页描述源码与安装事实，运行按冻结快照解释。当前 patrol.yaml 源/安装 SHA256 均为 `8efb9f54b41d4fe35efc5866f6b7cd3a97f1efe049775030ba842bec7f15174c`（优化预算100 ms），remani.yaml 均为 `0d32414a81b6585beb9e3921b5111bd72c4835b46fc7cdc737e523f720d7bfee`（seed42）。DEV-27数值/执行结果见[固定简单路线验收汇总](../../reports/patrol_numerics_20261124_01/numerical_model_acceptance_dev27.json)，DEV-29独立显示层的适用范围见本页末节；最新服务要求、已通过范围和未达总目标的差距统一见[当前进度](../current_work_progress.md)，不把固定路线成功解释为复杂环境或泛化完成。

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

核对记录中的DEV-27会话已冻结安装输入，见 `reports/gazebo_20261124_192255/inputs`；DEV-29显示配置另存该会话的 `rviz_display/20261124_194750/inputs`，不回写原启动快照。服务存活与是否允许运行按进度和用户最新指令判断，源码/安装一致不能替代实际进程和输入身份。

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

底盘源几何阈值为 `0.04 + 0.10 + (safe ? 0.13 : 0)` m；硬源检查 safe=false 时仍为 0.14 m。优化代价为 `0.04+0.13=0.17` m。两条路径不同，仅提前障碍代价未解决历史转弯失败；后续初始化、四轮软模型和时间映射修复的效果限于报告所覆盖的固定任务，不构成任意转弯可行性保证。

REMANI 来源及新增项见 [provenance](../../docs/provenance.md)。普通单目标保留源差速硬/软运动学检查；巡检显式保留源几何检查，硬运动学由公共四轮验证负责。DEV-10 源码将巡检软代价同步为同几何的四轮速度、轮加速度与转向率（此前仍为差速软项）；软阈值为 3.5 rad/s、4.0 rad/s²、公共转向率上限的 0.9，基座线速和原曲率项保留。四轮轮位来自 controller.wheel_positions，按 planning frame 重表达；未提供当前几何则拒绝。13 包已构建，固定第二段四轮梯度/公共模型误差 2.70e-9/8.83e-9、完整目标梯度误差 4.02e-6；四轮软模型已同 DEV-11/13/14 初始化和 through 修正完成第一次全路线离线验证，硬限制未改。这是预先声明的固定模型选择，禁止拒绝后隐式切换。

## 巡检与时间处理

用户最新要求允许几百毫秒响应，本轮成功任务完整规划与验证以 1 秒内为工程目标；不把原 100 ms 优化预算当作整体响应上限。机械臂主动协同要求记录于 agent.md；当前生产关节限速/巡检目标尚未改变。显式离线 `test_coupled_planning.py --check-initial-gradient` 设置 `optimization.check_initial_gradient=true`，在优化计时开始前核对全部初始变量的解析/有限差分梯度，日志含独立诊断耗时；默认 false，生产 YAML 不启用。

patrol 源配置：通过速度 0.15 m/s、优化预算 100 ms、forward_only=true、Dubins shot 半径 0.40 m；三个目标见 [模型与世界](robot_model.md)。REMANI 使用的 `search.random_seed` 位于 remani.yaml，源码与安装当前均为 42。省略此参数会保留随机设备初始化。DEV-06 首段显式加入 0.2 s、0.002 m 的 C² 起步曲线到达 REMANI 非奇异 0.02 m/s 切向边界；中间点保留非零底盘通过速度，最终返回任务捕获的实测起态。实际执行以DEV-27相应GUI报告为证，历史DEV-14/16离线通过不能代替执行或新版验收。当前目标与边界仍偏保守，参见模型知识中的任务能力边界。

单目标时间策略由 coupled_execution.yaml 明确启用 `local`，最大倍数 8、代码局部最小 0.5、端点至少 1；`uniform` 仅显式选择，无失败回退。巡检使用独立 `through` 映射，端点一阶时间导数为 1、二阶为 0，最大段内放慢倍数 **3，当前写在 patrol.cpp**。两者不能混用来破坏移动交接导数。

当前through窗口对每端分别采用 `min(T/4,max(0.2,2*该端线速/(wheel_acceleration*wheel_radius)))` 秒，0.15 m/s 的起窗为 0.6 s，返航 0.02 m/s 末窗为 0.2 s；四节点 C2、端点导数、最大倍率 3 与 2 ms/20 ms 验证均保留。它来自DEV-16对旧共用端点窗口的修复，不是可行性保证；其离线及后续执行验收分别见进度原始节点，不能从参数本身推断复杂路线稳定。

预算到达可产生待检查候选，但不代表收敛或可执行。计时须包含搜索/优化/源检查，并另外记录时间映射、公共验证及端到端墙钟。

离线 probe 的 `planning_validation_wall_s` 从 C++ planner 调用前到最终公共验证返回后直接取 steady_clock；包含搜索、优化、源检查、候选诊断构造、原始公共检查、时间映射和最终公共验证。进程/模型/地图初始化与独立回归不在此字段内，不能据此声称 GUI 请求端到端或实际执行已达标。仅原生记录该字段的报告提供直接计时，旧报告的分项和另列。

DEV-09 修正 MINCO 初始时间：关节限速所需时间和首末 1.5 倍余量真正传入生成器；非停止通过目标的最终片采用弦长/指定末端切向速度，保留关节时间下限，避免套用搜索停止轮廓。优化返回后按 L-BFGS 返回的变量重建多项式；它是同一候选状态同步，不重新搜索。普通停止短末片使用后述DEV-27合并与恢复片数规则；实施与验证效果统一见进度，不据此宣称所有初值无内部反向。

DEV-11/13 对四轮最终转弯目标（通过或停车切向）准备单个 MINCO seed：末内点与目标 heading 相差超过 1e-3 rad 时，先合并末端过密点，末片弦长至少覆盖 `four_wheel_min_turning_radius`（当前 0.30 m、至少保留两片），合并搜索时间并保留 1.5 倍末片余量；再沿粗 MINCO 曲线等时细分末片、恢复原优化片数和每片 K=8 的总代价样点数。A* 输出、目标 p/v/a、源/公共/时间硬验证间隔不变，直行保留 DEV-09。DEV-12 版正常离线三个巡检点已通过、返航未通过；DEV-13 已构建，DEV-14 配合时间映射后返航离线通过。这是初始化规则，不是安全容差。

DEV-13 校正 MMConfig 低速几何导数：原 yaw/旋转函数的方向在 norm>=1e-6 保持 atan2，因此其导数使用实际速度而非抬高到 0.02 m/s 的速度；既有 norm<1e-6 的常量朝向分支使用零导数。模型几何、方向标定、非奇异速度参数与硬限不改变。probe 独立低速有限差分已通过，yaw/旋转最大相对误差约 4.01e-9/2.09e-9。

## 尚未配置化的代码常量

这些值目前在 C++ 中，不应声称都能从 YAML 调节。改动时同时更新来源和本表。

| 来源 | 当前约束 |
| --- | --- |
| `risk_mm_runtime/include/risk_mm_runtime/validation.hpp` | 公共网格采样间隔 0.02 s；缺少覆盖时的臂默认值仍为 0.35/0.7，正常 launch 覆盖为 0.8/1.6 |
| `.../execution.hpp` | 到位容差 XY 0.04 m、yaw 0.05 rad、臂 0.03 rad；跟踪门限 0.10 m/0.35 rad/0.08 rad；对齐 8 s、终点收敛 5 s |
| `.../src/execution.cpp` | 队列交接 p/v/a 容差 1e−4 / 1e−4 / 0.002；实测停车连续确认 3 次 |
| `.../src/patrol.cpp` | 巡检点位置 0.08 m、臂 0.04 rad；持续低于 0.005 m/s 超过 0.2 s 记一次停车；新段起始 jerk 为零、保持 C2 |
| `.../src/runtime.cpp` | 场景租期 0.5 s、执行 tick 20 ms |
| `scripts/validation/test_patrol.py` | 验收还要求通过速度>0.02 m/s、返航误差<0.04 m；默认每段规划至最终验证完整直接响应最大 1000 ms，另要求初始臂速度<0.02 rad/s 连续0.2秒ROS时间且反馈时间戳一致 |
| `src/planning/remani_adapter/src/planner.cpp` | 静止 forward-only 起步曲线时长 0.2 s、位移为 REMANI 非奇异速度乘时长的一半（当前 0.002 m）；不是 YAML 参数 |

`remani.yaml` 中的 `fsm.global_plan` / `fsm.planning_horizon` 仍被原搜索适配读取，不代表旧 FSM 已编译或内部模式决策已启用。`ros2_control.yaml` 的“执行适配器尚待接通”注释是历史过期文字，当前已有单目标执行证据；判断接入状态要看代码和进度。

DEV-20 执行接管保持：`SimControl::reserve` 保留接管前已发布的臂位置目标，避免把重力下的实测偏差变成指令阶跃；实测停车与起态/所有权检查保留。执行起态仍由真实反馈捕获；ALIGNING 必须再次等到停止，不放宽门限。构建与实测结果见进度。

DEV-22 运行层为批量候选验证和执行每tick实测碰撞检查在启动时各建独立 MeshScene 工作实例，采用完全相同的冻结 WorldScene、URDF/SRDF 与 padding，避免共享可变MoveIt状态锁阻塞执行；场景/版本门控、mesh采样、0.15秒命令看门狗均不变。巡检报告 `planning_validation_ms` 以steady_clock直接测量planner调用至最终公共验证返回（含through），不含启动初始化、异步调度排队或GUI请求传输；验收使用该字段，失败也保留耗时。

DEV-23 静止转向对齐：任务保留实测 start_steer；共用 initialSteeringAlignment 从首个非零平坦速度确定初始轮角，静止任务先固定整机位姿/轮位置、以原转向率上限完成对齐并按20 ms完整mesh采样，移动边界不插入。执行ALIGNING使用同一目标并等待实测停车/轮角；轨迹p/v/a和全部后续验证不变。报告 initial_alignment_s、initial_steer，巡检另存 measured_start_steer；不再把旧轮角至切向的角差挤进首个运动样本。

uniform时间估计也从共用初始对齐目标开始计算运动段轮角速率；对齐阶段仍独立按同一硬限制验证，不把静止轮角调整重复计入首个运动样本，独立缩放回归断言和倍率上限保持。

DEV-26单纯拉长普通stop末片的试改经固定输入失败后已撤回；当前DEV-27复用已有合并/粗MINCO/恢复片数规则，普通差速模型最终停止任务也在末端空间不足0.30m时使用，四轮巡检选择条件保持。目的是避免短末片内部反向与低速奇异；单个初值、K=8总采样、差速和公共硬gate保留，效果以新报告为准。

普通源差速硬检查沿用上游 feasibilityCheck 的既有5%容差：轮速阈值为3.5×1.05=3.675rad/s，轮加速度为4×1.05；软代价目标仍3.5/4。此次未修改该门控，3.688rad/s仍被正确拒绝。公共整机四轮硬限制独立保持10/5及pi/2。

## RViz巡检显示层

Gazebo启动使用 `mm_bringup/rviz/inspection.rviz`（Fixed Frame=world），独立模型展示仍用 `rangermini_description/rviz/display.rviz`（base_link）。`mm_bringup/config/visualization.yaml` 只由独立C++ `inspection_visualization` 消费：local_window_m=6、distance_range_m=1、publish_hz=2、max_trace_points=10000；世界、巡检文件和实际网格分辨率由同一launch组装传入。它没有运动发布器或规划/执行服务客户端，参数不进入规划、公共验证或执行，也不参与机器人模型签名。

全局 `/inspection/global_costmap` 反解实际 `/grid_map/esdf` 的强度（原发布器将[-3,3]m距离编码为[0,1]），将距离按显示范围归一化为0..100；局部 `/inspection/local_costmap` 是同一全局数据随车裁剪，窗外/未知为-1。它们是显示用ESDF距离热力图，不是Nav2或新的规划代价栈。原ESDF话题切片高度来自 `grid_map.esdf_slice_height`（当前-0.1m），GridMap查询会夹取到现有网格首层；显示不改这个规划网格、源几何或任何hard gate。实际三维几何由同WorldScene的79个盒轮廓补充，不能用二维显示代替整机验证。

`/inspection/validated_route` 累积同任务连续收到的已验证段，`active_path` 按实际状态目标选择当前执行段，`measured_path` 来自真实ground_truth/odom，均world并以低频latched话题供RViz晚加入。`world_markers`/`task_markers` 显示真实世界轮廓、巡检点/方向、捕获起点、局部窗和状态。状态/地图过期或map_ready=false时，距离热力图全部变为未知并显示UNKNOWN/STALE；不把历史轨迹当作正在执行。

验收边界：DEV-29已构建、接入现有会话并实际加载新RViz配置；现有报告只覆盖静止状态的地图数值、局部裁剪一致性、路径/标记消息和截图。局部随车窗口、当前运动段与整条巡检路线的动态显示尚未验收，用户随后要求停止开发。新增显示不增加避障、感知或泛化能力；该限制对应2026-11-24暂停；2026-11-25用户已授权继续，动态结果仍须新增证据，见进度。

## 复杂任务离线验证入口（2026-11-25）

`test_coupled_planning.py` 的 `--world-file` 与 `--world-metadata-file` 必须成对显式提供，加载本地SDF及完整空间元数据并核对SHA256；缺省仍使用原工厂世界，不在失败后换地图。原始文件路径/世界身份写入报告，机械臂、四轮硬限制、完整源/公共采样不变。模型签名描述机器人，所选世界通过独立world SHA256/map_revision标识。

新增C++离线 `scene_state_probe input.yaml report.yaml` 使用现有WorldScene＋MeshScene与调用方明确提供的URDF/SRDF/padding批量检查九维状态；输出逐状态有效性及拒绝原因、模型/世界文件哈希、检查墙钟，状态拒绝属于数据，由验收脚本核对预期正反例。该程序没有规划/执行接口，不把离散状态检查称为路径安全；用途是复杂变姿任务几何必要性与固定臂同轨迹对照。运行层和生产YAML不加载任何新增姿态库。

机械臂采样适配参数（2026-11-25，生产YAML尚未启用）：`search.manipulator_reachable_sampling`默认false，仅在同一采样器中裁剪提议区间到原关节速度给定时间内可达的URDF范围；`search.manipulator_extension_fraction`默认0.3、必须在(0,1]，只改变树扩展的保守步长，原0.35rad/s关节速度、0.7rad/s²及边/源/公共限制不变。离线 `--arm-sampling global|reachable` 和 `--arm-extension-fraction` 有明确消费者并写入报告；切换在任务开始前显式完成，不因失败自动改变。

DEV-20261125-04的截断正态提议未改善固定三seed结果，已撤回；当前reachable仍为可达盒均匀分布。原失败报告保留，不能以试改期间构建报告宣称当前通过。

DEV-20261125-06：既有短末片合并/粗MINCO细分规则覆盖四轮直行最终停止端（此前只覆盖四轮末yaw变化或普通差速停止）；移动通过端保持原判据。仍为单种子/单次优化，恢复原片数与K=8，所有硬验证保持。

DEV-20261125-05：Gazebo launch支持成对显式world_file/world_metadata_file、patrol_file和预先显式arm_sampling/arm_extension_fraction；缺省仍为原工厂/生产参数。启动脚本先核对源/安装YAML和launch一致，将自定义世界与任务复制到本次inputs再传入；仿真出生位姿用--pose明确给出，不标记为实测。GUI和显示读取同一所选世界/任务，采样覆盖进入model_signature的model_data，启动命令/输入保留有效值。离线路线入口同样支持成对世界/metadata和patrol，逐段传递完整p/v/a边界。已构建并贯通原/自定义离线路线；尚未新场景仿真，不作执行成功声明。

DEV-20261125-07：同一粗MINCO/细分恢复规则覆盖初速度不高于0.02m/s且空间短于现有0.30m模型尺度的起步首片，按原搜索时间与关节时间下限合并；末片规则保留。逐合并片记录原片数量，再等时细分还原原总片数及K=8密度，只有单个初始化与一次L-BFGS。离线--arm-search-budget-ms显式映射既有search.max_mani_search_time（生产80ms不改），与100ms优化预算分开报告。报告措辞改为当前REMANI移植/物理数值适配，不将其当作锁定原版的同条件计时。

DEV-20261125-08取代DEV-07一般短首片试改：只有单段、严格共线、零平面a/jerk边界且主动臂偏移>0.15rad的四轮任务，使用同一个满足精确p/v/a/jerk的七次标量进度反求原几何配对点时间。其速度Bernstein系数为[k0,k0,k0,7-3k0-3k1,k1,k1,k1]，要求中间系数至少为min(k0,k1)，保证初值不低于端点较小切向速度；总时间满足原关节时间下限/基座软速度，无法准备则明确拒绝。该域内首末短片可合并再细分同一粗曲线，恢复原片数/K=8；域外仅保留DEV-06之前的终端初始化。它不是新增候选回退，优化/源几何/公共/through全部照常，尚未用构建或代码事实替代验收。

离线--arm-soft-margin-m只覆盖既有optimization.safe_margin_mani（生产仍0.02m，source球半径0.03m与完整mesh padding0.01m不变），用于同条件检查更早障碍代价是否改善有限预算候选；它不修改源几何拒绝阈值，也不因失败自动选择余量。

DEV-20261125-10：coupled_execution.yaml新增有消费者validation.mesh_workers=4（1..4），只控制候选批量完整网格检查的独立实例数。四轮逆解/轮位积分及全部20ms/边界样点仍按时间生成，再全部送同URDF/SRDF/padding/world的FCL实例并行核对，拒绝按原时间顺序报告；静止对齐/终端制动检查保留，执行tick仍使用独立单实例。probe可用--mesh-workers显式对照1/4。scene_state_probe支持batch_workers与initial_wheel_position以作同状态正反例对照。线程/并行不构成连续安全保证；51正反状态与1648冻结状态串并行判定逐项一致，批量几何约3倍加速，完整任务与执行仍按进度验收。

DEV-20261125-11：源safe=false自碰撞使用同一双精度原点树的布尔半径查询，保留车臂0.07m/臂臂0.06m阈值及全部SRDF/owner排除，先车臂后臂臂；safe=true参考邻居列表路径保留。环境ESDF、优化目标、原点数及公共网格不变。--compare-self-collision只在离线probe以seed20261125、4002个全范围/局部/任务状态逐项比较原查询bool/类别及独立耗时，普通任务不运行诊断。删除无消费者的planningEnvironmentCollision/obstacleHit，非删除任何在用的验证。4002状态bool/类别对照0差异，独立365.09→17.96ms；该查询对照不代替完整规划/执行验收。

DEV-20261125-13：cooperation/route工具的--optimizer-budget-ms显式覆盖既有solve_budget_ms，保存冻结有效预算与override；生产patrol仍100ms，只有明确选定的离线实验使用200ms。硬限制、采样/源检查和through倍率不变，无失败后自动增加预算。

DEV-20261125-14：已有check_initial_gradient离线分支对同一次初始化x用1e-6/1e-7/1e-8/1e-9相对差分步长检查全部变量，报告各尺度最大误差与独立诊断耗时后恢复x。生产仍默认false，未改变目标项、优化预算、硬门控或正常响应；多尺度用于区分病态初值的有限差分误差与解析梯度错误。

DEV-20261125-15取代共线专用准备：对单方向四轮、零平面a/jerk边界且臂中间状态超出起终关节区间>0.15rad的任务，按累积基座搜索弦长准备同一七次共同进度；首末空间跨度还覆盖原关节时间下限×端点速度×1.5，合并后按原重数细分恢复片数/K=8。标量速度下界为min(0.02,起速,末速)的Bernstein充分条件，总时间只在该构造域内求解；非正弦长/非有限时间/无法准备明确拒绝，未证明整个时间域无解。曲线XY不宣称由此保证单调或硬可行，仍须原优化、源10ms、公共20ms及through2ms全核对。普通端点臂插值保留原初始化，不引入姿态库/拒绝后回退。

DEV-16/19显式仿真参数：arm_soft_margin_m/arm_search_budget_ms由launch覆盖实际后端，进入model_signature；启动脚本冻结ELF及对应源码SHA到inputs/code_identity.json。实验环线patrol_execution200.yaml明确200ms，生产patrol仍100ms；当前新任务性能与有限失败按实际会话记录。

本轮实验边界（DEV-21）：生产配置仍100ms优化、global/0.3、臂软余量0.02m；实际DART会话显式200ms/reachable0.6/0.08m。固定实测起态改0.12m仍源拒绝，未部署。成功离线与真实GUI失败分别保留，不据此放宽源/公共限制。

DEV-22离线源几何回放：已有coupled_probe的显式probe.source_geometry_input_file读取冻结validation.json三层状态，先核对world_signature/model_signature，再以同一MMConfig/完整ESDF逐点给出源臂环境见证及同点软代价，并与现有源环境checker逐状态对照。仅诊断、无搜索/优化/执行；diagnostic_wall_s不能用于规划性能。搜索节点无时间覆盖，记录的轨迹离散点也不能等同10ms完整多项式复验。


DEV-24取消首端合并试改因北侧/理想首段回归失败已撤回，现恢复DEV-15合并规则。DEV-25仅增加remani_objective日志器的首/返回目标函数分项DEBUG记录，默认关闭；不改变计算或候选选择。

DEV-25源代理见证：冻结实测失败点距SDF实体盒约0.20m但源ESDF约0.028m，栅格覆盖/ceil膨胀具有额外保守性。该点距离不是完整机器人mesh通过证明；本轮未改地图精度/膨胀/硬阈值。显式400ms实验任务已在冻结复杂环线通过3轮GUI，适用范围仍限定该任务；生产仍100ms，泛化/完整原REMANI对照未完成，实际版本与时延见进度。



DEV-28两种倍率过渡窗试验均失败已撤回，through仍原固定2v/a窗口、最大3及原全验证。DEV-29复用种子空间准备覆盖无大臂绕障（detour≤0.15rad）的四轮移动起边界，起速须>non_singular；主动臂原common-progress条件保持，粗MINCO后恢复原片数/K8。

静态链接构建事实（DEV-32）：REMANI/运行库默认.a，修改库必须串行重建并安装包括mm_bringup在内的消费者；仅重建coupled_probe不更新simulation_runtime。start_gazebo --check-build只检查、不启动/停止服务；正常启动也检查CMake实际.a依赖与ELF Build ID/时间关系，将库SHA记入code_identity。缺少build证据或库更新但前端未重链拒绝启动。配置一致和源/ELF各自快照不能替代链接一致。

DEV-33巡检mission逐段task_*字段冻结原九维起态及v/a/jerk、实际轮角/轮位、限速、模型/地图和移动/停止边界；planned_goal继续保留。只报告保存，不改运动接口或规划行为。DEV-30 GUI测量CSV使用Python float/int落JSON，并按GUI接受的mission过滤，防止ROS numpy标量导致报告失败；--patrol-timeout-s只调观察等待，原响应/安全验收不变。

DEV-35 RViz动态验收可显式--mission-id指定本轮编号，避免重复任务观察误用旧完成状态；当前三轮明确id并保留原报告。原生RViz相机全景操作仅作用当前会话，未写回install或改变规划显示数据。

单目标与巡检预算须区分：普通requestPlan恢复optimization.solve_budget_ms（未配置时0，即不以预算终止）；巡检从所选patrol文件设置预算。当前3轮594–848ms证据仅巡检完整规划/through/最终验证；随后操作者coupled_4/5单目标实际规划3.263/5.032s，均执行成功但未毫秒化，不混入巡检统计。

DEV-20261126-01实现（有限离线结果以进度为准）：external四轮种子先尝试简化冗余关节往返，保留全部base节点及原搜索累计时间；按0.01m/0.01rad及至少4分段检查替代关节连接的源几何，只修改同一种子的臂内点。不是新增姿态库，MINCO后的源/公共/through门控不变。KinoAstar非法起终几何直接返回，source_validation分别记start_geometry_rejected/goal_geometry_rejected，不再浪费后续搜索预算。

DEV-20261126-02：非法起终源几何覆盖近距调臂/不走base search入口；候选不生成时probe也保存planning_validation_wall_s，范围为planner调用至源/搜索拒绝（含诊断构造、不含初始化），公共验证未运行仍为null。成功任务的原完整计时范围不变。种子简化耗时计入既有search_wall_s，未新增配置。

种子简化的0.05rad常量位于poly_traj_optimizer.cpp：只在原累计逐边关节L∞变化比拟用直连至少多0.05rad时进行该替代检查；它是冗余度判据，不是碰撞或执行容差。该步骤不改底盘节点和原搜索时间，之后仍执行既有首末片空间准备及原片数恢复规则。
