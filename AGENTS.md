# 新工作空间开发约定

- 模型固定为本仓库的 Rangermini＋xArm6，独立构建；不得 source 或引用旧工作空间。
- 耦合基线源于锁定版本的 REMANI。原始文件保留在 third_party，移植差异保留在 docs/remani_ros2.patch；保留许可与来源。
- 用户已有耦合研究成果不等同于 REMANI 复现；该首版选用原始算法是工程基线选择。
- 当前只实现 COUPLED、ARM_FIRST 两模式。ARM_FIRST 只有 robot.yaml 的一个缩回目标，不恢复 BASE_FIRST、姿态库或隐式算法回退。
- 按 docs/architecture.md：几何特征、风险评分、策略、切换、规划、验证、执行各自负责。Python 使用 Protocol，C++ 使用抽象接口。
- 新逻辑不加入原始 REMANI FSM；首版不编译旧 FSM，不启用内部模式决策。
- 计算耗时用单调墙钟，执行时间用 ROS 时间；未知、过期和版本错配输入不得当作安全。
- 规划候选、验证通过、实际开始、实际完成分别报告。采样校验不宣称连续安全。
- 仿真使用隔离 ROS domain/Gazebo master，只清理本次启动的进程。
- 验收入口 scripts/validation；保留失败报告。进度记录在 docs/current_work_progress.md。
