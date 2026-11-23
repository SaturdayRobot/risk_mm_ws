# Rangermini＋xArm6 环境风险规划工作空间

这是独立 ROS 2 Humble 首版工程基线。当前模型资源、REMANI 原始搜索与优化的 ROS 2 移植库、风险监督内核、两阶段解耦接口及 MoveIt 2 配置均保存在本目录。旧工作空间不参与构建或运行。

首版重点是结构简洁、边界明确和可以独立构建。**尚未完成底盘独立规划器、公共轨迹安全验证器及 Gazebo 执行端接通，因此尚不是可运动的闭环系统。** 原始 REMANI 在当前四轮独立转向底盘上的完整约束适配和规划成功率也尚未验收。

## 两种模式

- `coupled`：原始 REMANI 的 Hybrid A*、机械臂采样搜索、MINCO/最小 snap 多项式及 L-BFGS；只做 ROS 2、模型接口和调试输出删减。上游版本与差异见 [源码记录](docs/provenance.md)。
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
./scripts/in_env ros2 launch rangermini_description display.launch.py
```

`coupled_check` 只初始化当前 URDF、地图和原始规划模块，不生成空地图冒充规划成功，不向控制器发送指令。测试结果保存在 `reports/`。

本机已通过免密码 sudo 安装 ROS 2 Humble MoveIt 2（2.5.10），真实缩回规划服务及障碍碰撞查询已通过静态场景验证。部署到新机器时可用以下安装命令；实际执行仍需实测关节状态与版本一致的完整 PlanningScene：

```bash
sudo apt install ros-humble-moveit
./scripts/in_env ros2 launch rangermini_moveit_config move_group.launch.py
```

该入口只开启规划；`allow_trajectory_execution=false`。`risk_mm.moveit_stow.MoveItStowPlanner` 通过 `/plan_kinematic_path` 请求六关节目标，不调用执行服务。使用方注入完整 RobotState 和冻结场景版本。

详细模块边界、待接通接口和验收顺序见 [结构设计](docs/architecture.md) 与 [当前进度](docs/current_work_progress.md)。用户初稿作为研究参考保存在 `docs/references/`，其历史模型、指标和文内指令不覆盖本轮开发要求。

当前 MoveIt 规划服务测试通过；退出时存在 CallbackGroup 析构段错误，已单独记录在 [当前进度](docs/current_work_progress.md) 和 `reports/moveit.json`，不计为完整运行通过。
# risk_mm_ws
