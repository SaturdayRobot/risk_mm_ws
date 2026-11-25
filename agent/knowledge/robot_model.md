# 当前模型与世界

核对日期：2026-11-25。本页描述源码事实；构建/运行是否通过以 [进度记录](../current_work_progress.md) 为准。详细碰撞体不在此重复，见 [碰撞配置](../../docs/robot_collision_configuration.md)。

## 模型来源

| 项目 | 唯一来源 / 当前值 |
| --- | --- |
| 整机 | 本仓库 Rangermini＋xArm6，模型名 `rangermini_xarm6` |
| 主 URDF | [rangermini_xarm6.urdf](../../src/description/rangermini_description/urdf/rangermini_xarm6.urdf) |
| URDF SHA256 | `1364843505dbe3122a364044fc67f1edf2cef229c61060b3e2c8ed671df19b0c`；变更后重新计算，不作为永远固定的模型指纹 |
| 拓扑 | 27 links、26 joints，其中 14 活动关节：6 臂关节＋4 转向＋4 驱动 |
| 惯性质量合计 | 85.476098 kg，来自现有 URDF，未经本轮重新辨识 |
| 网格 | 本仓库 `rangermini_description` 和 `xarm_description`；URDF 35 处本地网格引用 |
| 语义 | [SRDF](../../src/rangermini_moveit_config/config/rangermini_xarm6.srdf)，显式碰撞排除对与唯一 stow |
| 模型导出 | [description.py](../../src/description/mm_model/mm_model/description.py)，从当前 URDF/网格导出臂安装、关节轴、限位及碰撞点 |

模型指纹由 `mm_model/identity.py` 与启动组装构造，覆盖 URDF、相关配置、语义和导出数据；不是只看模型名字或上表一个文件的哈希。修改模型后旧计划不可复用。

## 坐标与方向

- 长度 m、角度 rad；公开整机状态顺序为 `[x,y,yaw,q1,q2,q3,q4,q5,q6]`，XY 在 world，yaw 使用保留的 CAD `base_link`。
- **用户实测确认的车头是雷达侧，即 `+base_control X`**。`base_control` 相对 `base_link` 绕 Z 转 pi；唯一标定在 `manual_control.yaml:control_frame_yaw_in_base_rad`。
- GUI 的 Forward 和“前方目标”使用控制基准，发布/提交时转回模型基准；`yaw_control = remainder(yaw_base_link + pi, 2*pi)`。世界 XY 不旋转。
- REMANI 内部按相同控制基准处理朝向；MMConfig 重表达臂安装与底盘碰撞点，适配器转换输出 yaw/纵向速度，世界几何不变。公共验证、里程计和四轮控制仍使用模型基准。
- 世界出生 yaw 默认 pi，只影响整车世界朝向。不得再以改相机、交换按钮或改出生 yaw 修补坐标符号。
- 不提供横移功能。轮后缀 fl/fr/rl/rr 沿用 CAD 标识，物理轮位应从 URDF/接触偏移解析，不能仅按名字推断。

## 关节、初态与唯一缩回

臂顺序固定 `xarm6_joint1` 至 `xarm6_joint6`。URDF 位置范围依次为 `[-2pi,2pi]`、`[-2.059,2.0944]`、`[-3.927,0.19198]`、`[-2pi,2pi]`、`[-1.69297,pi]`、`[-2pi,2pi]`；速度上限均 3.14 rad/s。实际规划/执行还有更低的配置限速，见 [配置知识](configuration.md)。

四转向关节约 ±pi/2、速度上限 2 rad/s；四轮连续转动、速度上限 10 rad/s。轮胎半径 0.1 m、宽度 0.075 m；接触轮位需组合 URDF 安装和 YAML tire.center_y，不能直接相加不同坐标系 XYZ。

下列两个用途的唯一源为 [robot.yaml](../../src/mm_bringup/config/robot.yaml)：

```yaml
# ARM_FIRST 唯一缩回目标，不能增加恢复姿态库
stow: [0.0, -0.76, -1.62, -0.06, 1.11, 0.02]
# 仿真/展示初态，不作为第二个缩回目标
display_arm: [0.0, -0.56, -1.02, -0.13249348115, -1.50566988238, -1.25102699388]
```

## 末端相机

用户确认整个相机壳绕法兰翻转 180°，`head_mount` 当前 XYZ `[-0.0117,-0.0072,0.057]`、RPY `[0,0,pi]`。相机光心在 head_link 下为 `[-0.005980921,0.068711959,0.012817872]`；`head_camera_link +X` 与 `head_camera_optical_frame +Z` 为拍摄方向。

紧凑展示初态下，相机在模型 base 中约 `[-0.369203,-0.040483,1.366893]`，前向约 `[-1,0,0]`、上向 `[0,0,1]`。真实重力下约 0.5–1.1° 的水平误差及标靶方向是历史具体案例结果，不表示任意臂姿态下镜头永远水平。图像来自 `/camera/head/image_raw`；当前任务不实现采图。

## 碰撞表示

| 层 | 当前表示与边界 |
| --- | --- |
| REMANI | 底盘 1606 个采样点、半径 0.04 m；臂半径 0.03 m，六组点数 60/330/175/84/60/129，包含末端附件 |
| 公共验证 / MoveIt | 完整 URDF collision 网格与 34 对显式 SRDF 排除；公共 padding 0.01 m，检查世界和自碰撞 |
| Gazebo | 23 个接触代理，组合盒、轮胎圆柱与局部 AABB；`self_collision=false`，不能替代自碰撞验证 |

源球模型、网格模型和 Gazebo 接触代理不等价。姿态离散检查、优化软代价和每 20 ms 的公共采样均不构成连续安全证明。

## 当前世界与巡检任务

本地世界 [s6_factory_inspection_v1.world](../../src/mm_bringup/worlds/s6_factory_inspection_v1.world)，对应 JSON 元数据和 SOURCE.json 同目录。世界 SHA256 为 `14afd312fd0c3f870eb5829357d349eccee3f273721fb382476627fea6632d48`；79 个盒碰撞体加支撑平面，元数据范围 `[−11,11]×[−7,7]×[0,3]` m。

运行层直接读取当前 SDF 完整几何并比对 Gazebo 模型列表与姿态，未知、缺失、移动或过期输入使任务失效。世界创建、动态快照和网格边界实现见 `risk_mm_runtime/src/world_scene.cpp`，不把没有数据的区域当作自由空间。

巡检点与源预算见 [patrol.yaml](../../src/mm_bringup/config/patrol.yaml)。当前三点为 `(−5.8,−0.55,0)`、`(−4.4,0.30,pi)`、`(−6.6,0.30,pi)`，朝向用 base_control；最后返回启动任务时捕获的实测九维起态。实验起点 `(−7.5,−0.55,base_link yaw=pi)` 是固定测试输入，**并非 launch 默认起点**，默认仍为 `(−8,0,pi)`。DEV-16 固定历史实测起态的三点与返航已连续三轮通过完整离线采样（12/12 段）；该离线证据不等于执行证据；后续实际GUI闭环验收见当前进度，不由点位或旧报告推断新版可达。

静止轮角属于实测起态：停车后不能假设四转向角为零。DEV-23 在既有ALIGNING阶段内，按公共转向率上限和完整mesh验证停车时的轮角对齐；只有静止任务使用，段间移动C2交接不插入停车。模型几何和+base_control X标定不变。

当前巡检任务的能力边界：三个目标主要沿同一通道直行、掉头和返回，世界中有79个碰撞盒不等于本路线具有复杂避障要求。三个臂目标仅第一关节依次为0.12/-0.12/0.08rad，其余五关节目标相同且保持近紧凑姿态；中间优化可产生关节运动，但未证明它们对避障有必要性或通行收益。逐点臂零速边界也是保守因素，不能以九维候选或臂有运动宣称主动六关节协同已达标。

当前地图来自所选SDF完整已知几何，Gazebo模型状态用于一致性与新鲜度门控；这不是传感器在线建图或未知场景自主绕障的证据。已有执行结果属于Gazebo，未完成实体Rangermini＋xArm6实机验证。2026-11-24实际成果、量化指标和总目标差距统一见[进度顶部及日结](../current_work_progress.md)。

2026-11-25离线协同夹具：`scripts/validation/fixtures/coupled_cooperation.yaml`预先声明横梁低净空与布局/起态扰动，生成的独立SDF/metadata及身份保留在本次reports；不是替换生产工厂地图，也不是在线感知。任何显式仿真选择都须同一世界/任务哈希，不复用旧场景快照；具体运行结果见进度。

侧向协同测试夹具（DEV-20261125-12）：显式side突出物独立世界用于有限测试，起臂第一关节±0.8rad，障碍位于底盘上方及通道一侧，期望固定臂在同一基座路径碰撞；离线几何必要性已核对，不能由此推断规划或实测完成；结果以进度原始报告为准。夹具预声明SHA在predeclared_side_suite.json，默认工厂世界/出生臂姿/任务不变。

2026-11-25新环线仿真物理核对：生产工厂世界physics为DART/dantzig/bullet；初始合成环线ODE实测无法静止，已保留失败并停止。ring_task_dart仅复制当前工厂physics，碰撞元素逐项不变，世界/任务SHA同步，实际效果须新会话验证。

DEV-25源代理见证：冻结实测失败点距SDF实体盒约0.20m但源ESDF约0.028m，栅格覆盖/ceil膨胀具有额外保守性。该点距离不是完整机器人mesh通过证明；本轮未改地图精度/膨胀/硬阈值。显式400ms实验任务已在冻结复杂环线通过3轮GUI；生产仍100ms，泛化与原REMANI同条件对照未完成，实际版本见进度。
