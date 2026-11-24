# 项目知识库

用途：让新 agent 从仓库直接恢复目标、模型、配置和工作状态。这里记录稳定事实及事实来源；进行中的方案和效果只在 [当前进度](../current_work_progress.md) 维护。

本知识库归属独立 `agent/` 目录；每次任务/运行前的必读流程见 [目录入口](../README.md)。

## 阅读入口

| 文档 | 内容与维护责任 |
| --- | --- |
| [agent.md](../agent.md) | 用户最终目标、优先级、完成标准；目标发生变化时更新 |
| [AGENTS.md](../../AGENTS.md) | 每个 agent 必须遵守的开发及记录约定 |
| [模型知识](robot_model.md) | 当前 URDF、坐标、关节、相机、姿态与世界资源 |
| [配置知识](configuration.md) | 配置责任、源码快照、加载顺序、限速层级与硬编码限制 |
| [结构设计](../../docs/architecture.md) | 模块边界、Task/Plan/Trajectory 契约及数据流 |
| [碰撞配置](../../docs/robot_collision_configuration.md) | 三种碰撞表示、忽略对、接触参数和验证边界 |
| [算法来源](../../docs/provenance.md) | 锁定 REMANI、原版与物理适配的差别、patch 与许可 |
| [当前进度与方案](../current_work_progress.md) | 唯一动态进度账本；顶部快照、当前方案、每次改动及实际效果 |
| [协作规则](../collaboration.md) | 分工、文件归属、共享资源、集成和交接格式 |

## 从代码定位责任

| 任务 | 主要位置 |
| --- | --- |
| 整机模型与网格 | `src/description/rangermini_description`、`xarm_description` |
| URDF 导出和规划几何 | `src/description/mm_model`、`mm_config` |
| 地图与 ESDF | `src/perception/plan_env` |
| Hybrid A*、机械臂采样与原多项式优化 | `src/planning/path_searching`、`traj_utils`、`traj_opt` |
| C++ 抽象接口和原后端适配 | `src/planning/remani_adapter` |
| 完整世界、公共验证、时间映射、执行和巡检 | `src/runtime/risk_mm_runtime` |
| C++ GUI、仿真驱动、launch 和参数 | `src/mm_bringup` |
| ROS 消息与服务 | `src/common/risk_mm_interfaces` |
| 唯一缩回配置与已有研究内核 | `src/rangermini_moveit_config`、`src/risk_mm`；完整在线 ARM_FIRST 未完成 |
| 验收入口、原始报告 | `scripts/validation`、`reports` |

实际 COUPLED 数据流：GUI/服务 → 运行层冻结 Task/模型/地图/限速 → REMANI 候选 → 明确配置的时间处理与公共验证 → 执行器 → 同一 Gazebo 驱动适配器 → 实测反馈。规划器不直接控制机器人，GUI 不代替验证器判断可执行性。

## 运行接口速查

| 接口 | 作用与边界 |
| --- | --- |
| `/coupled/plan`，PlanCoupled | 九维目标＋任务/模型/地图身份＋三项速度；接受请求后仍需生成和验证候选 |
| `/coupled/execute`，ExecutePlan | 执行同编号的有效已验证单目标，重新检查环境、实测起态和执行所有权 |
| `/coupled/start_patrol`，StartPatrol | 从实测起态启动配置的巡检任务，后续自动规划/验证/执行；闭环当前未验收 |
| `/simulation/stop`，Trigger | 保持运动、作废待执行代号；不等同退出进程 |
| `/coupled/status`，CoupledStatus | 版本、阶段、候选/验证/实测开始/完成、位姿和原因 |
| `/coupled/patrol_status`，String JSON | 巡检编号、阶段、planned/visited、耗时、停车次数及返航完成字段 |
| `/simulation/gui_status`，String JSON | 原生 GUI 验收的按钮/输入状态；测试不能借此绕过 GUI 发布运动指令 |

字段定义查 [risk_mm_interfaces](../../src/common/risk_mm_interfaces)，具体 JSON 发布查 GUI/运行层源码。服务 accepted、轨迹 READY、实际开始和最终完成是不同事件；通用状态中已有段通过也不表示下一段通过，逐段以 mission 报告为准。

## 事实与证据的优先顺序

1. 用户最新指令和已确认的物理约定决定目标与约束。
2. 对“当前源码是什么”查 `src`；对“实际跑了什么”查该次会话的启动参数、输入快照、二进制版本和实测报告。两者不一致时先记录差异。
3. 知识库中的值是带日期的源码摘要，不是第二份配置；修改应落到对应唯一源文件并同步摘要。
4. `build`/`install`/`log` 是生成目录，不在其中修逻辑。`reports` 是证据，不从某个 passed 文件推断整个版本通过。
5. 文档过期时保留历史事实，在当前快照说明适用版本；失败报告不可改为 passed 或删除来制造成功率。

## 使用与维护

只读/文档任务不启动 ROS 或 Gazebo。需要实施时通过 `scripts/in_env` 构建和运行，具体复现/启动/停止命令见 [README](../../README.md) 和验收脚本；遵守用户最新服务状态要求。

修改坐标、模型或安装关系更新模型知识；修改参数、覆盖关系或代码常量更新配置知识；修改接口/责任更新结构设计；修改上游衍生代码更新来源与 patch。所有逻辑修改同时记录到当前进度，包含未测试或失败的效果。
