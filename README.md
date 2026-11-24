# Rangermini＋xArm6 环境风险规划工作空间

这是独立 ROS 2 Humble 首版工程基线。当前模型资源、REMANI 原始搜索与优化的 ROS 2 移植库、风险监督内核、两阶段解耦接口及 MoveIt 2 配置均保存在本目录。旧工作空间不参与构建或运行。

Agent 文档集中在 [agent/](agent/README.md)，每次任务开始、恢复及运行前按其中的必读顺序读取。最终目标见 [任务入口](agent/agent.md)，并查阅 [项目知识库](agent/knowledge/README.md)、[唯一进度与方案记录](agent/current_work_progress.md) 和 [协作规则](agent/collaboration.md)。每个逻辑修改必须记录验证结果及实际效果。当前自主不停步巡检闭环尚未完成，服务已按用户要求停止；最新构建和未验证改动以进度顶部快照为准。

当前已接通 C++ Gazebo、原始 REMANI 耦合规划、完整网格/四轮公共采样验证与实测执行反馈。已通过真实 GUI 的 Plan/Execute 按钮完成工厂场景正前方 0.6 m 车臂联合任务并到位停车（底盘误差约 1.2 mm）；这是一项具体案例验收，不代表任意任务可行或连续路径安全。ARM_FIRST 的独立底盘规划器与在线两阶段执行尚待接通，不作隐式回退。

## 两种模式

- `coupled`：原始 REMANI 的 Hybrid A*、机械臂采样搜索、MINCO/最小 snap 多项式及 L-BFGS；原始算法之外，为当前四轮模型明确增加最小转弯半径可行性项与独立时间适配，保留原始源码、开关与差异。上游版本与差异见 [源码记录](docs/provenance.md)。
- `arm_first`：MoveIt 2 规划机械臂缩回，底盘保持固定；缩回后机械臂保持该唯一姿态，由独立底盘规划器移动。执行适配器必须取得实际停车、收臂到位和新鲜反馈后才允许第二阶段。

机械臂唯一缩回目标为 `[0, -0.76, -1.62, -0.06, 1.11, 0.02]` rad，采用当前模型的规划姿态。目标配置在 `src/mm_bringup/config/robot.yaml`，MoveIt SRDF 中的同名目标由测试检查一致性。该姿态的模型校验与具体场景收臂路径安全是不同检查。

## 构建与检查

只需系统 ROS 2 Humble 和系统依赖。脚本会清理继承的工作空间路径；不要先 source 旧工程。

```bash
cd /home/ubuntu/risk_mm_ws
./scripts/in_env colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
./scripts/in_env python3 scripts/validation/test_planning_components.py
./scripts/in_env python3 scripts/validation/test_model.py
./scripts/in_env python3 scripts/validation/test_independence.py
./scripts/in_env python3 scripts/validation/test_moveit.py
./scripts/in_env ros2 launch mm_bringup coupled_check.launch.py
./scripts/in_env ros2 launch mm_bringup display.launch.py
```

`coupled_check` 只初始化当前 URDF、地图和原始规划模块，不生成空地图冒充规划成功，不向控制器发送指令。测试结果保存在 `reports/`。

本机已通过免密码 sudo 安装 ROS 2 Humble MoveIt 2（2.5.10），真实缩回规划服务及障碍碰撞查询已通过静态场景验证。部署到新机器时可用以下安装命令；实际执行仍需实测关节状态与版本一致的完整 PlanningScene：

```bash
sudo apt install ros-humble-moveit
./scripts/in_env ros2 launch rangermini_moveit_config move_group.launch.py
```

该入口只开启规划；`allow_trajectory_execution=false`。`risk_mm.moveit_stow.MoveItStowPlanner` 通过 `/plan_kinematic_path` 请求六关节目标，不调用执行服务。使用方注入完整 RobotState 和冻结场景版本。

详细模块边界、待接通接口和验收顺序见 [结构设计](docs/architecture.md) 与 [当前进度](agent/current_work_progress.md)。用户初稿作为研究参考保存在 `docs/references/`，其历史模型、指标和文内指令不覆盖本轮开发要求。

当前 MoveIt 规划服务测试通过；退出时存在 CallbackGroup 析构段错误，已单独记录在 [当前进度](agent/current_work_progress.md) 和 `reports/moveit.json`，不计为完整运行通过。

## Gazebo 与 C++ 控制

```bash
ROS_DOMAIN_ID=182 GAZEBO_MASTER_URI=http://127.0.0.1:11482 ./scripts/in_env ros2 launch mm_bringup gazebo.launch.py
```

此命令会打开 Gazebo、RViz 和 C++ Qt 控制面板；已有同一会话运行时不重复启动。全部新仿真程序按 `ament_cmake` 编译到 `install/mm_bringup/lib/mm_bringup/`，没有 Python 控制包。世界文件及来源清单保存在 `src/mm_bringup/worlds/`，不在运行时引用旧工作空间。

按住底盘按钮移动，松开或按 STOP 停车。前进对应用户确认的 `+base_control X`，后退对应其反向；`base_control` 相对保留的模型 `base_link` 绕 Z 转 pi，二者有显式 TF。机械臂先设置六个关节目标，再勾选 `Send arm targets`；取消勾选、切换窗口或 STOP 后保持当前控制目标。目标值和实测值分列显示。复制实测角可更新输入框。腕部画面来自 Gazebo 的 `/camera/head/image_raw`。相机圆形镜头所在面是正面，RViz 蓝色 Z 轴表示拍摄方向；默认展示初态与唯一 `stow` 目标分开。整车在世界中的默认出生 yaw=pi，雷达侧按用户确认定义为 +base_control X；横移按钮已取消，接口拒绝非零 body vy。

耦合操作：点击 `Goal: 0.6 m ahead` 以实测正前方（base_control）设置目标，或输入世界 X/Y/yaw 与六关节目标；点击 `1. Plan coupled` 后，面板先停车并等实测稳定，再提交实测起态；请求进度与拒绝原因持续显示。只有公共检查通过且场景/起态仍有效才可点击 `2. Execute validated plan`。规划时底盘保持，执行时车臂同时受同一驱动适配器控制。当前显式采用局部时间适配，在需要的位置减速，其他位置按速度上限运行；完整网格、四轮速度/加速度/转向速率都重新验证。报告保留原多项式和时间映射，几何不变。面板分别显示候选、验证、开始和完成；STOP 停车并作废待执行计划。更改目标需要重新规划。原始后端不启用旧 FSM。

C++ 服务为 `/coupled/plan`、`/coupled/execute`、`/simulation/stop`，状态为 `/coupled/status`；计划编号、模型指纹和地图版本必须匹配。候选/通过验证的底盘路径发布到 RViz。场景使用当前世界完整几何，并持续校验 Gazebo 模型列表与姿态。

验收会产生小幅运动，结果按会话保存在 `reports/gazebo_*/`，最新会话信息为 `reports/gazebo_latest.json`：

```bash
./scripts/in_env python3 scripts/validation/test_collision_config.py
ROS_DOMAIN_ID=182 GAZEBO_MASTER_URI=http://127.0.0.1:11482 ./scripts/in_env python3 scripts/validation/test_gazebo.py
./scripts/in_env python3 scripts/validation/test_coupled_planning.py
ROS_DOMAIN_ID=182 GAZEBO_MASTER_URI=http://127.0.0.1:11482 ./scripts/in_env python3 scripts/validation/test_coupled_execution.py
```

机器人尺寸、碰撞体、忽略对、限位、膨胀与接触参数逐项见 [碰撞参数与配置](docs/robot_collision_configuration.md)。当前 Gazebo 关闭机器人自碰撞，手动界面不验证任意目标的无碰撞性；耦合执行另经完整网格与四轮采样验证；未知/过期场景、反馈失效、版本错误或跟踪误差超限均进入保持。配置和具体限值见该文档。


自动验收可使用独立无界面实例，避免与当前 GUI 的手动操作争用。默认隔离为 domain 186 / master 11486，输入与报告另存，停止时只清理该记录的进程组：

```bash
python3 scripts/validation/start_gazebo.py --headless
ROS_DOMAIN_ID=186 GAZEBO_MASTER_URI=http://127.0.0.1:11486 ./scripts/in_env python3 scripts/validation/test_coupled_execution.py --session reports/gazebo_headless_latest.json
python3 scripts/validation/start_gazebo.py --headless --replace
ROS_DOMAIN_ID=186 GAZEBO_MASTER_URI=http://127.0.0.1:11486 ./scripts/in_env python3 scripts/validation/test_coupled_execution.py --session reports/gazebo_headless_latest.json --abort-on-first
python3 scripts/validation/start_gazebo.py --headless --stop
```

`test_gazebo.py` 同样接受 `--session`。手动回归与耦合回归各自从初态测试时，使用 `start_gazebo.py --headless --replace` 仅重置此无界面实例；可见 GUI 不受影响。


当前 0.6 m 耦合到位案例和运动中场景失效停车分别通过；一次后续反向候选被原始优化器轮加速度检查拒绝，未执行，失败报告单独保留。最新分项结果见 `reports/validation_summary.json`，不以一个案例代替任意目标可行或连续安全证明。


当前真实面板验收见 [走廊联动与速度配置](reports/gazebo_20261124_113219/gui_checks/20261124_113727/validation.json)：规划 1.12 s、执行轨迹 11.29 s，到位停车。方向约定及历史验收见 [架构](docs/architecture.md) 与 [当前进度](agent/current_work_progress.md)。


GUI 上方分别设置底盘线速度、角速度和关节速度上限，默认 0.40 m/s、0.60 rad/s、0.50 rad/s，范围上限分别为 0.80、1.00、0.80。设置同时用于手动控制与下一次规划；更改后旧执行入口失效，需要重新 Plan。曲率、加速度、四轮转向和碰撞约束仍会限制实际速度，不保证整段恒速。

默认 `display_arm` 已改为紧凑且相机水平的展示/仿真构型，镜头高度约 1.37 m；ARM_FIRST 的唯一 `stow` 保留。下拉选择任务后点击“载入目标”，再 Plan/Execute；预设文件为 `src/mm_bringup/config/goal_presets.yaml`，固定世界坐标，不自动运动或选择模式。

| 预设 | 世界 X/Y（m） | 车头朝向 | 主要约束 |
|---|---|---|---|
| 走廊错位联动 | -4.75 / 0.45 | 0° | 曲线错位、紧凑臂与相机侧向观察 |
| 北侧门洞进站 | -5.00 / 2.30 | +90° | 转弯穿过约 1.6 m 门洞 |
| 南侧门洞进站 | -5.00 / -2.05 | -90° | 穿门并在低净空检修区前停车 |

这些目标已从记录的参考起态做候选和公共校验；位置变化后仍必须从当前实测状态重新规划。
