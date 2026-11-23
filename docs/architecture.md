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

| 层 | 输入 → 输出 | 首版实现 |
|---|---|---|
| 环境几何 | 同一坐标系的完整 AABB 快照、整机碰撞球 → 距离特征 | `features.BoxClearance`；不选择模式；空/不完整环境无效 |
| 风险评分 | 距离、接近速度、时间与地图版本 → [0,1] | `risk.ClearanceRisk`；静态几何适配当前接近速度为零 |
| 模式策略 | 风险、实际模式 → 期望模式 | `risk.HysteresisPolicy`；高低阈值之间保持，映射可配置 |
| 切换监督 | 期望、停车、候选、验证和执行反馈 → PREPARED/EXECUTING/IDLE/HOLD | `supervisor.Supervisor`；实际模式只在执行开始反馈时更新 |
| 耦合规划 | 整机 Task → 耦合候选 | C++ `Planner` / `RemaniPlanner`；原始搜索失败计数固定为零，不触发跨模式回退 |
| 解耦规划 | 同一 Task → 缩回＋移动候选 | Python `ArmFirstPlanner`；`ArmPlanner`、`BasePlanner`、`Validator` 由组装层注入 |
| 收臂规划 | 实测起态、唯一缩回目标 → 固定底盘关节轨迹 | `moveit_stow.MoveItStowPlanner`，真实服务客户端；静态场景规划与障碍拒绝已验证 |
| 执行准入 | 实测臂位置/速度、底盘速度、反馈时间 → 允许第二阶段或拒绝 | `execution_gate.StowGate`；必须由实际执行端调用 |
| 公共验证 | 完整候选、同一模型与地图 → 可行性结果 | Protocol / C++ 抽象接口已定义，完整实现待接入 |
| 执行适配 | 验证通过的同一候选、监督代号 → 实测执行反馈 | 待接入；首版不发布运动指令 |

风险公式和“高风险先缩回”只是初始可替换工程策略，未标定为研究结论。可替换 `FeatureExtractor`、`RiskAssessor`、`ModePolicy`；没有在 ROS 回调或原始 FSM 中加入算法。

## 共同任务与轨迹出口

共享边界状态为九维 `[x,y,yaw,q1,q2,q3,q4,q5,q6]`，长度单位 m、角度 rad。`Task` 携带 task_id、model_id（Python 固定检查）、map_revision、start、goal。规划函数不得调用 Gazebo 或控制器。

`Plan` 携带同一任务、实际算法 mode、相对时间与九维状态点、单调墙钟 planning_wall_s。这是**候选采样出口**，尚未定义可供控制器执行的插值和导数编码；不得直接把点序列作为已验证控制轨迹。原始多项式仍保留在 REMANI 后端内，C++ 适配器以不超过 0.02 s 的间隔导出起终点和段边界。Python / C++ 跨进程消息编解码尚待接入。

REMANI 内部仍使用 `[x,y,q1..q6]` 八维平坦输出，yaw 由路径切线和前后向决定，不能把任意独立 yaw 当作已支持。原始差速轮可行性目标函数保留；当前机器人是四轮独立转向，完整四轮限值必须由模型适配和公共验证补齐后再接受运动候选。原始优化器内部碰撞规则也不替代全模型公共复核。

ARM_FIRST 的终态只允许同一个缩回构型。输入任意其他机械臂目标即失败，不追加末端恢复阶段。组合规划会检查阶段起终点和锁定部件；实际执行还必须停车收臂，不能用“已规划到缩回位置”代替实测到位。底盘规划器必须在相同地图和完整缩回整机几何上检查可行性，不可只检查底盘中心。

组装入口 `risk_mm.assembly` 从 YAML 读取并实例化风险、模式策略、监督器、两阶段规划器与收臂门控。它要求调用方显式注入机械臂/底盘规划和公共验证实现，不提供假成功的默认后端。

## 配置边界

- `mm_bringup/config/robot.yaml`：模型身份、六关节名称、唯一缩回姿态。
- `mm_bringup/config/remani.yaml`：原始搜索和优化参数、地图参数；删去旧姿态初值、重定时、内部自适应扩展。
- `mm_bringup/config/supervisor.yaml`：初始风险尺度、迟滞和门控要求；ROS 监督组装端尚待读取。
- `rangermini_moveit_config/config/`：SRDF、KDL、OMPL、限位；第六连杆与末端附件的刚性连接显式标为 FixedAttachment；启动时读取本仓库整机 URDF，保持车体碰撞几何参与机械臂规划。
- `mm_bringup/config/ros2_control.yaml`：机器人所引用的本地控制配置，暂不代表执行链已接通。

不完整/过期/未来时间/地图版本不一致的环境输入进入 HOLD。准备候选要实际停车、同任务同地图且完成验证；迟到执行确认、地图更新和模式改变会使旧代号失效。运行节点、心跳/超时调度和硬件停车调用尚待组装，不能把该内核测试当作实际停车证据。
