# xarm_description：当前 xArm6 网格资源

本包仅提供整机 `rangermini_xarm6.urdf` 实际引用的官方网格：

- `meshes/xarm6/visual/`：固定基座与六个连杆，共 7 个 STL。
- `meshes/end_tool/collision/end_tool.stl`：法兰末端几何。

网格来自项目原有的 UFactory `xarm_ros2` 资源，保持原文件内容。完整关节、惯量和装配关系统一保存在 `rangermini_description/urdf/rangermini_xarm6.urdf`。

其他机械臂型号、夹爪、官方相机、重复 DAE 和旧 xacro 配置已移至工作空间外备份，详见工作空间 `docs/slim_cleanup.json`。本包不含实物通信驱动。
