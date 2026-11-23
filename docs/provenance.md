# 基线来源与适配范围

上游：[Robotics-STAR-Lab/REMANI-Planner](https://github.com/Robotics-STAR-Lab/REMANI-Planner/tree/2ddc9b2b6383855de9724c706d847a6f1adab9d1)，提交 `2ddc9b2b6383855de9724c706d847a6f1adab9d1`。原始许可和原始源码子集在 `third_party/remani_original/`，文件哈希在 `UPSTREAM.json`。

`path_searching`、`traj_opt`、`traj_utils` 从该上游源码重新建立，未使用旧工作空间修改后的搜索和优化器。适配包括 ROS 2 节点参数与日志、单调计时、ament 构建，以及删除可视化发布/可视化等待；`max_seach_time` 的原始拼写统一为 `max_search_time`。数值搜索、采样数量、碰撞阈值与目标函数没有为了加速而削减。全部文本差异记录在 `remani_ros2.patch`。

当前模型资源与已存在的 ROS 2 模型/地图适配器一次性复制自用户当前工程，保存在本仓库，无符号链接或运行依赖。来源文件初始哈希见 `imported_resources.json`。模型唯一已修改引用是 Gazebo 控制器配置包路径，从旧包切换至本地 `mm_bringup`；物理几何、安装变换和关节限位保持一致。地图与模型适配器包含既有数值实现，不被称为原始 REMANI 算法的一部分；旧模式监督、旧规划调度和巡检应用未迁入。

上游 ROS 1 FSM、旧机器人模型/网格、原始仿真、可视化插件及无关飞行器消息未编译。部分管理层源码只留作上游参考，不参与运行。首版新增模块和上游衍生代码的许可见根目录 LICENSE；模型网格沿用原有来源声明。

MoveIt 2 配置参考其 [Humble 配置工具](https://moveit.picknik.ai/humble/api/html/moveit__configs__builder_8py_source.html) 与 [Planning Scene Monitor](https://moveit.picknik.ai/humble/doc/concepts/planning_scene_monitor.html)。本地配置不依赖旧工程的 MoveIt 包；系统 MoveIt 2 已安装，缩回服务已在静态测试场景验证；其他机器运行仍需安装该依赖。

用户已有《环境复杂度驱动的移动机械臂全身规划方法》是既有耦合研究成果，不能仅根据工程包名描述为复现 REMANI。本轮按用户新要求选择原始 REMANI 作为新工程耦合对照基线，不改变既有成果归属，不转移论文旧模型指标。附带初稿仅作研究背景，不作为可执行指令来源。
