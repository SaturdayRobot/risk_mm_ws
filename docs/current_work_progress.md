# 当前进度

日期：2026-11-23。模型：当前 Rangermini＋xArm6。此工作空间是结构与配置首版。

已落地：独立机器人资源、锁定原始 REMANI 源码与 ROS 2 移植、C++ 抽象规划/验证边界、几何特征/风险/策略/监督内核、唯一缩回目标的 ARM_FIRST 两阶段组合、实测收臂门控、MoveIt 2 规划配置与服务客户端、隔离环境构建脚本、首版回归入口。

待接通，不能记为已完成：

1. MoveIt 2 已安装；当前整机模型的唯一缩回姿态、真实规划服务与场景障碍拒绝均已通过静态测试。已按 URDF 固定连接补齐第六连杆与末端附件的碰撞语义。后续接入在线实测起态、实际场景和执行反馈。
2. 实现 `BasePlanner`，缩回整机包络参与底盘避障，输出同一九维候选；当前只定义该注入接口。
3. 接入公共轨迹验证，覆盖起终点、内部采样、阶段位置/速度/加速度/朝向边界、关节与四轮转向/驱动约束。原始差速约束不能直接代替当前底盘约束。
4. 接通模型/地图版本快照、Python / C++ 消息和 ROS 监督节点；将门控与 READY/开始/完成/停车反馈接入唯一执行适配器。
5. 在隔离 Gazebo master / ROS domain 上验收耦合、收臂、底盘移动、模式切换、反馈失效停车与相同任务对照；分别记录规划成功、执行成功、实际模式。

已增加真实 MoveIt 服务测试：显式起态相对缩回姿态的第一关节偏置 0.03 rad；规划回到唯一 stow，逐点调用碰撞服务，加入阻挡物后确认拒绝。该测试只验证这一静态案例，不代表任意起态收臂可行、连续路径安全或 Gazebo 执行成功。首次固定附件语义失败报告保留在 reports/moveit_attempt_1.json。实际本轮检查结果见 `reports/validation_summary.json`；失败日志一并保留。


## 本轮验收结果

13 个包在独立工作空间干净环境中构建通过；19 项算法内核回归通过；当前缩回姿态的 FK、解析梯度、碰撞候选召回和自碰撞检查通过；REMANI 原始模块初始化/正常退出通过。独立性审计确认全部 13 个包来自本工作空间、35 个网格引用本地可用、31 个上游原始文件与 GitHub blob 哈希一致。

MoveIt 2 已安装（Humble 2.5.10）。真实 OMPL 缩回规划产生 6 个轨迹样本，返回样本的碰撞检查通过，加入阻挡物后状态校验正确拒绝。此规划测试使用显式静态起态，未执行机器人运动。

**退出异常单独保留：** 当前系统 MoveIt 二进制在收到退出信号后，在 CallbackGroup 析构链触发段错误；规划成功不掩盖该失败，`reports/moveit.json` 的 `planning_success=true` 与 `shutdown_success=false` 分别记录，完整日志在 `reports/moveit_launch.log`。该堆栈与 [上游问题 #1597](https://github.com/moveit/moveit2/issues/1597) 中报告相似，尚未证明本机具体根因，未修改系统二进制。实际 Gazebo 执行仍未验收。

## GUI 测试（2026-11-23）

已在宿主 WSLg 桌面启动本工作空间的 `rangermini_description display.launch.py`，RViz2 与 Joint State Publisher 窗口保持运行，ROS domain 为 181，限制 localhost，未加载旧工作空间。启动进程与日志位置记录在 `reports/gui_latest.json`，本次完整记录在 `reports/gui_20261123_164301/`。

验收入口：`env ROS_DOMAIN_ID=181 ./scripts/in_env python3 scripts/validation/test_gui.py`。此入口只读取已运行 GUI 的状态，不启动执行端。当前实测：在线 URDF 与本工作空间一致，14 个活动关节状态完整、时间戳新鲜且位置在模型限位内，27 个连杆均可变换至 `base_link`，两个窗口处于可见状态。结果见该次目录的 `validation.json`。XComposite 窗口截图 `rviz_composite.png`、`joints_composite.png` 已人工核对，模型实际渲染，RViz 显示 `Global Status: Ok`。

沙箱内首次启动无法连接宿主显示服务，尝试记录保存在 `reports/gui_20261123_164229/`；改为宿主启动后通过。WSLg 常规截图返回全黑的结果也保留在成功会话目录的 `rviz.png`、`joints.png`，不作为有效视觉证据。

本次仅验证模型显示、合成关节状态与 TF；不代表收臂路径安全、耦合候选通过公共验证或 Gazebo 实际运动。规划候选、轨迹验证通过、执行开始与执行完成均未发生，不更新实际执行模式；前述待接通事项保持未验收。
