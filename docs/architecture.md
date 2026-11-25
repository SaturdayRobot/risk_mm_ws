# 程序结构与数据契约

首版只保留两模式，避免沿用旧工程的巡检任务、三模式调度、姿态库、参数自适应与多套模式裁决。

```text
src/
  description/
    rangermini_description/   当前整机 URDF、底盘网格、展示入口
    xarm_description/         当前机械臂和末端网格
    mm_model/                 URDF 导出、几何与四轮运动学
    mm_config/                规划器模型接口、FK 与碰撞查询
  common/mm_utils/            ROS 2 数值工具
  common/risk_mm_interfaces/  C++ 服务与阶段状态消息
  runtime/risk_mm_runtime/    完整世界、公共验证与执行组装
  perception/plan_env/        ROS 2 栅格/ESDF 适配
  planning/
    path_searching/           上游原始 Hybrid A* / RRT / 机械臂采样
    traj_utils/               上游多项式与最小 snap 表示
    traj_opt/                 上游目标函数与 L-BFGS
    remani_adapter/           抽象规划接口、原始后端调用和候选导出
  risk_mm/                    无 ROS 算法内核与 MoveIt 服务适配
  rangermini_moveit_config/   arm 组、唯一 stow、KDL、OMPL 与限位
  mm_bringup/                 参数和组装入口
third_party/remani_original/  锁定的上游源码子集，不参与 colcon
scripts/validation/           首版验证入口
```

## 责任划分

| 层       | 输入 → 输出                                                     | 首版实现                                                                                                               |
| -------- | ---------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------- |
| 环境几何 | 同一坐标系的完整 AABB 快照、整机碰撞球 → 距离特征               | `features.BoxClearance`；不选择模式；空/不完整环境无效                                                               |
| 风险评分 | 距离、接近速度、时间与地图版本 → [0,1]                          | `risk.ClearanceRisk`；静态几何适配当前接近速度为零                                                                   |
| 模式策略 | 风险、实际模式 → 期望模式                                       | `risk.HysteresisPolicy`；高低阈值之间保持，映射可配置                                                                |
| 切换监督 | 期望、停车、候选、验证和执行反馈 → PREPARED/EXECUTING/IDLE/HOLD | `supervisor.Supervisor`；实际模式只在执行开始反馈时更新                                                              |
| 耦合规划 | 整机 Task → 耦合候选                                            | C++ `Planner` / `RemaniPlanner`；原始搜索失败计数固定为零，不触发跨模式回退                                        |
| 解耦规划 | 同一 Task → 缩回＋移动候选                                      | Python `ArmFirstPlanner`；`ArmPlanner`、`BasePlanner`、`Validator` 由组装层注入                                |
| 收臂规划 | 实测起态、唯一缩回目标 → 固定底盘关节轨迹                       | `moveit_stow.MoveItStowPlanner`，真实服务客户端；静态场景规划与障碍拒绝已验证                                        |
| 执行准入 | 实测臂位置/速度、底盘速度、反馈时间 → 允许第二阶段或拒绝        | `execution_gate.StowGate`；必须由实际执行端调用                                                                      |
| 公共验证 | 完整候选、同一模型与地图 → 可行性结果                           | Python Protocol；C++ `SampledValidator` 使用精确多项式、URDF 网格和四轮约束，已接入 COUPLED                          |
| 执行适配 | 验证通过的同一候选、监督代号 → 实测执行反馈                     | C++ `Executor` / `ActuatorPort` 抽象接口；`CoupledExecutor` 与 Gazebo 驱动适配器闭环，ARM_FIRST 在线执行仍待接通 |

风险公式和“高风险先缩回”只是初始可替换工程策略，未标定为研究结论。可替换 `FeatureExtractor`、`RiskAssessor`、`ModePolicy`；没有在 ROS 回调或原始 FSM 中加入算法。

## 共同任务与轨迹出口

共享边界状态为九维 `[x,y,yaw,q1,q2,q3,q4,q5,q6]`，长度单位 m、角度 rad。`Task` 携带 task_id、模型身份（Python model_id / C++ model_signature）、map_revision、start、goal；C++ 还携带起始转向和轮角。规划函数不得调用 Gazebo 或控制器。

公开九维状态中的 yaw、URDF、里程计和四轮运动学保留现有模型 `base_link` 基准。用户实测确认的正前方使用 `base_control +X`；`base_control` 相对 `base_link` 绕 Z 转 pi，由 `manual_control.yaml:control_frame_yaw_in_base_rad` 唯一配置并发布静态 TF。GUI 的 Forward、目标“前方”和目标朝向采用该控制基准；世界 X/Y 不变，提交任务时 yaw 转回公开模型基准。现有关节后缀沿用 CAD 标识，具体轮位从 URDF 与轮胎偏移解析。

REMANI 的内部车头朝向采用同一控制基准，参数 `mm.planning_frame_yaw_in_base_rad` 从上述配置注入并进入模型指纹。C++ `MMConfig` 将臂安装矩阵和底盘碰撞点重表达到内部基准，保持世界几何不变；`RemaniPlanner` 在入口转换起终 yaw，在多项式出口转换 yaw 和纵向速度符号。世界平面 p/v/a、关节值和角速度不变；公共网格验证、四轮检查和执行仍使用公开 base_link 基准。只允许 0/pi 的纵轴配置，不支持借此启用横移。

`Plan` 携带同一任务、实际算法 mode、相对时间与九维状态点、单调墙钟 planning_wall_s。点序列是候选显示出口，不直接作为控制插值。C++ `Trajectory` 保留原始优化多项式的独立副本，提供精确 p/v/a、yaw/角速度、时长与阶段边界，公共验证和执行均使用同一不可变对象。跨进程只发送带模型/地图版本的任务、执行代号和阶段状态，不序列化近似点列来替代原始轨迹。

离线 C++ probe 可显式启用诊断轨迹，保存 REMANI 简化搜索路径、MINCO 初始多项式和优化后候选采样，包括源检查拒绝的优化结果。该数据仅用于失败定位，不参与代价、验证或执行；正常运行默认不采集。离线巡检验证器按段传递上一段的时间映射终态 p/v/a 与四轮状态，起始 jerk 依巡检调度约定置零，并对每段分别运行公共完整采样验证。

REMANI 内部仍使用 `[x,y,q1..q6]` 八维平坦输出，yaw 由路径切线和前后向决定，不能把任意独立 yaw 当作已支持。原始差速轮可行性目标函数项保留；当前优化器另启用明确记录的四轮最小转弯半径软约束，避免有限转向范围的分支跳变；当前机器人是四轮独立转向，完整四轮限值必须由模型适配和公共验证补齐后再接受运动候选。原始优化器内部碰撞规则也不替代全模型公共复核。

ARM_FIRST 的终态只允许同一个缩回构型。输入任意其他机械臂目标即失败，不追加末端恢复阶段。组合规划会检查阶段起终点和锁定部件；实际执行还必须停车收臂，不能用“已规划到缩回位置”代替实测到位。底盘规划器必须在相同地图和完整缩回整机几何上检查可行性，不可只检查底盘中心。

组装入口 `risk_mm.assembly` 从 YAML 读取并实例化风险、模式策略、监督器、两阶段规划器与收臂门控。它要求调用方显式注入机械臂/底盘规划和公共验证实现，不提供假成功的默认后端。

## 配置边界

- `mm_bringup/config/robot.yaml`：模型身份、六关节名称、唯一缩回姿态。
- `mm_bringup/config/remani.yaml`：原始搜索和优化参数、地图参数；删去旧姿态初值、重定时、内部自适应扩展。
- `mm_bringup/config/supervisor.yaml`：初始风险尺度、迟滞和门控要求；ROS 监督组装端尚待读取。
- `rangermini_moveit_config/config/`：SRDF、KDL、OMPL、限位；第六连杆与末端附件的刚性连接显式标为 FixedAttachment；启动时读取本仓库整机 URDF，保持车体碰撞几何参与机械臂规划。
- `mm_bringup/config/ros2_control.yaml`：四个控制器已在 Gazebo 激活；唯一驱动适配器在手动和 COUPLED 独占控制之间仲裁。
- `mm_bringup/config/simulation_collision.yaml`：C++ Gazebo 接触代理、轮胎与摩擦参数；不改写规划 URDF。
- `mm_bringup/config/coupled_execution.yaml`：公共采样验证与耦合执行的硬限制；不修改原始优化器软代价。
- `mm_bringup/config/manual_control.yaml`：C++ 手动接口的限速和超时；没有模式选择或规划回退。

新增仿真运行层为 `ament_cmake` 的 C++ 程序：`sim_model.cpp` 生成接触模型，`sim_control.cpp` 处理受限手动指令、独占耦合指令和实测反馈，`sim_control_gui.cpp` 提供 Qt 面板，launch 只负责组装。`sim_control` 在无新指令时保持固定关节目标；未来关节消息进入有界队列，ROS 时间追上后才可成为有效反馈。Gazebo 世界、模型和配置均为本工作空间本地资源。碰撞表示的差异和完整数值见 [碰撞参数与配置](robot_collision_configuration.md)。

不完整/过期/未来时间/地图版本不一致的环境输入进入 HOLD。准备候选要实际停车、同任务同地图且完成验证；迟到执行确认、地图更新和模式改变会使旧代号失效。COUPLED C++ 运行节点已接通场景 0.5 s 租期、驱动命令 0.15 s ROS 时间看门狗、20 ms 执行调度与 Gazebo 停车。风险驱动的在线模式切换、ARM_FIRST 全链和真实硬件适配未接通；内核测试不作为这些功能的执行证据。

## C++ COUPLED 组装

`simulation_runtime` 使用三个执行线程托管 ROS 反馈/驱动、运行门控与后端地图回调；原始搜索/优化另在异步线程计算，不阻塞驱动保持。完整世界只从当前 SDF 构造，在 `/gazebo/model_states` 完整匹配且反馈新鲜后初始化 ESDF，未知物体和姿态变化均使计划版本失效。

`/coupled/plan` 获取实测停车起态并冻结模型/地图身份，驱动适配器在规划期间保持；得到原始多项式后执行公共验证，发布 READY。`/coupled/execute` 再检查身份、起态和停车，先对齐四轮，再按 ROS 时间评价同一多项式。手动消息不能覆盖已占有的执行器，`/simulation/stop` 可作废代号并停车。实际模式只在观察到物理位移后更新。

候选生成、验证、开始、完成字段独立上报；正常完成必须实测到位和停车，中止保留失败执行报告。场景变化后的失效事件另写文件，不篡改已经完成的任务结果。当前仅显式运行 COUPLED，不在这里加入风险评分、姿态选择、ARM_FIRST 回退或原始 FSM。

显式物理时间适配位于规划候选与最终公共验证之间，独立于原始 REMANI 和模式策略。原候选先验证；仅几何有效的候选进入计时。YAML 显式选 `local`：对原多项式参数 u 构造单调 C2 时间映射 dt/du=s(u)，s 使用分段三次平滑插值；速度为 q′/s，加速度为 q″/s²−q′s′/s³。从可行整体倍数出发，局部缩短时间时逐次检查原解析导数，随后密集核对四轮与关节约束并完整公共验证。`uniform` 仍可显式选择作为对照，不做隐式回退。

局部最小倍数 0.5（最多加快到源参数速度的两倍），端点倍数至少 1，最大倍数 8，总时长上限 120 s；不缩减几何采样或放宽四轮硬限制。几何、版本、端点或连续性错误不能由计时修复，复核失败仍拒绝。planning.yaml 保存原候选、原拒绝原因、平均时间比、最大局部倍数以及源时间/执行时间/局部倍数节点，validation_wall_s 包含全部计时和复核。

PlanCoupled.speed_limits 为 [底盘线速度、偏航速度、关节速度]，零值兼容配置默认，非零必须为有限正数且不超过配置硬上限。每次请求冻结自己的速度边界，公共验证、时间映射与执行使用该快照。GUI 编辑后清除 accepted plan 并禁用旧 Execute；执行中速度控件禁用，不在线压缩已验证轨迹。手动关节消息可带六项 velocity 上限，驱动端再次核验并限速。

优化器的碰撞代价使用保守空间剔除加速：局部点对查询按原索引排序，仅跳过原代价必为零的远点；环境包围盒只有整个插值区域可证明零代价时才跳过点查询。保留全部原点集、阈值、项、采样数量和累加顺序；独立 C++ 对照同时检查代价与解析梯度，记录 search/optimization/source_check/public validation 的单调墙钟耗时。

任务预设是带世界指纹的显式九维操作目标，由 C++ 严格加载；不会触发运动、选择模式或被 ARM_FIRST 当作缩回姿态库。紧凑水平相机的 display_arm 仅用于初始化与明确的操作任务，不修改唯一 stow。

## 自主连续巡检接口（正在验收）

共享 Task 补充移动起态的速度/加速度/jerk 和目标通过速度。普通单目标保留停车终态；巡检中间点不制动。CoupledExecutor 队列只接受模型/地图一致且交接 p/v/a 连续的候选，交接沿 ROS 时间直接切换；无下一段时失效停车，不把缺失轨迹当作可执行空间。

Patrol 独立调度下一段真实 REMANI 规划，运行当前段时在后台生成并完整验证下一段，随后送入同一执行器。任务配置来自 patrol.yaml，进入时记录实测起点，按巡检点顺序执行后回到该起点。采图不参与此调度，原 FSM 不编译/不启用。共享任务 YAML 加载已统一到 risk_mm_runtime，GUI 与巡检不各写一套解析。

StartPatrol 使用相同模型/地图身份和三项速度上限；GUI 复用既有停车起步准备流程，进入巡检后自动规划与执行。巡检状态区分候选/验证/实测运动，保存每点通过速度、误差、规划耗时和返回误差。时间映射失败另记录首个违反约束/时刻/比值及最大比值候选；公共验证是否运行单独标记，避免把时间映射拒绝误报成网格校验。优化预算、位置区域容差和前进约束是显式配置，具体闭环结果以最终验收为准。

巡检的源几何检查与机器人运动学检查各有边界：保留全部源几何采样，巡检软代价使用当前四轮轮位的转速、轮加速度与转向率，公共四轮验证器负责硬拒绝；单目标保留原差速硬/软检查。该物理模型选择在 StartPatrol 显式开启，不因拒绝而改变。巡检 through 时间映射两端 dt/du=1 且 d²t/du²=0，四个节点的边界过渡按端点速度与物理轮加速度尺度确定（上限 T/4），只搜索一个段内放慢倍数（最大 3）。保留源几何和移动 p/v/a 边界，按原 2 ms 检查四轮导数后再做完整公共 20 ms 网格检查；新 MINCO 段起始 jerk 设零，交接保持 C2，不复用会改变端点速度的单目标时间映射。

运行层的批量轨迹公共验证与执行tick的实测碰撞检查使用独立的 MeshScene 可变工作状态；两者在启动时从相同冻结模型、世界及 padding 构建，仍调用同一完整网格实现。版本/动态场景门控统一，避免批量采样持锁阻塞执行命令。巡检 `planning_validation_ms` 使用单调墙钟记录一次候选规划、through 映射及最终公共验证的完整工作耗时；进程初始化和GUI请求传输另计。

完整网格批量检查（DEV-20261125-10）：验证器先按时间顺序计算全部p/v/a约束和轮级状态，将同一组完整状态交给有界独立FCL实例批量检查；不共用可变RobotState/PlanningScene，不省略任何已接受轨迹的原采样点。所有检查结果按原索引回收，任何异常为拒绝，首个几何拒绝仍按时间报告。执行实测场景为独立单实例，不等待批量场景锁。



DEV-28两种倍率过渡窗试验均失败已撤回，through仍原固定2v/a窗口、最大3及原全验证。DEV-29复用种子空间准备覆盖无大臂绕障（detour≤0.15rad）的四轮移动起边界，起速须>non_singular；主动臂原common-progress条件保持，粗MINCO后恢复原片数/K8。
