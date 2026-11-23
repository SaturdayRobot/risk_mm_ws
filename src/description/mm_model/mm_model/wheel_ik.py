# -*- coding: utf-8 -*-
"""4WIS-4WID 底盘运动学 (规划器/控制器共用参考实现).

几何与限幅全部取自 `matlab_modeling`:
  wheel_params.csv     : 轮心 x/y、轮半径、转向极限
  chassis_params.csv   : 轴距 0.494、轮距 0.3365、轮半径 0.1、
                         转向 ±90 deg、轮缘速度 2 m/s、转向速率 90 deg/s、
                         底盘设计速度 1.2 m/s、偏航率 1.2 rad/s
对应学位论文第 2 章 "四轮独立转向驱动建模":
    v_i = v + ω × r_i          (r_i 为轮心相对 base_link 原点的位置)
    δ_i = atan2(v_iy, v_ix)
    ω_i = |v_i| / r_w

规划器只需要 (x, y, yaw) 或 (vx, vy, ω); 轮级量只出现在控制器与执行器之间。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

# 轮心相对 base_link (m), 顺序固定 fl, fr, rl, rr
WHEEL_POSITIONS: Dict[str, Tuple[float, float]] = {
    "fl": (0.246999, 0.167833),
    "fr": (0.246999, -0.167452),
    "rl": (-0.247004, 0.167405),
    "rr": (-0.247001, -0.168634),
}
WHEEL_ORDER: List[str] = ["fl", "fr", "rl", "rr"]


@dataclass
class ChassisLimits:
    wheel_radius: float = 0.100
    steer_limit: float = math.pi / 2          # rad
    steer_rate_max: float = math.pi / 2       # rad/s
    wheel_speed_max: float = 2.0              # m/s 轮缘线速度
    base_speed_max: float = 1.2               # m/s
    base_yaw_max: float = 1.2                 # rad/s
    wheelbase: float = 0.494003
    track: float = 0.336467
    origin_radius: float = math.hypot(0.247, 0.1686)   # 轮心到原点最大距离


@dataclass
class WheelSolution:
    steer: List[float] = field(default_factory=list)          # rad
    wheel_speed: List[float] = field(default_factory=list)    # rad/s
    wheel_linear: List[float] = field(default_factory=list)   # m/s
    feasible: bool = True
    reasons: List[str] = field(default_factory=list)
    saturated_steer: List[bool] = field(default_factory=list)
    saturated_speed: List[bool] = field(default_factory=list)


def inverse(vx: float, vy: float, wz: float,
            limits: Optional[ChassisLimits] = None,
            prev_steer: Optional[Sequence[float]] = None,
            dt: float = 0.01) -> WheelSolution:
    """底盘速度 -> 四轮 (转向角, 轮速).

    prev_steer 用于低速/零速时的转角保持 (避免 atan2(0,0) 抖动);
    同时用于检查转向速率是否超限。
    """
    lim = limits or ChassisLimits()
    sol = WheelSolution()
    if math.hypot(vx, vy) > lim.base_speed_max + 1e-6:
        sol.feasible = False
        sol.reasons.append(f"base speed {math.hypot(vx, vy):.3f} > {lim.base_speed_max}")
    if abs(wz) > lim.base_yaw_max + 1e-6:
        sol.feasible = False
        sol.reasons.append(f"yaw rate {abs(wz):.3f} > {lim.base_yaw_max}")

    for k, name in enumerate(WHEEL_ORDER):
        x, y = WHEEL_POSITIONS[name]
        vix = vx - wz * y
        viy = vy + wz * x
        lin = math.hypot(vix, viy)
        if lin < 1e-6 and prev_steer is not None and len(prev_steer) == 4:
            delta = _wrap_steer(float(prev_steer[k]))
            signed_v = 0.0
        else:
            delta = math.atan2(viy, vix)
            signed_v = lin
            # 轮子可正反转: 把转角折叠到 [-90°, 90°], 用反向滚动作等价表达。
            # 否则原地回转/斜行时 atan2 会给出 >90° 的"不可实现"解。
            if delta > math.pi / 2:
                delta -= math.pi
                signed_v = -lin
            elif delta < -math.pi / 2:
                delta += math.pi
                signed_v = -lin
        sat_steer = abs(delta) > lim.steer_limit + 1e-9
        if sat_steer:
            delta = math.copysign(lim.steer_limit, delta)
            sol.feasible = False
            sol.reasons.append(f"{name}: steer {delta:.3f} exceeds limit")
        sat_speed = abs(signed_v) > lim.wheel_speed_max + 1e-9
        if sat_speed:
            sol.feasible = False
            sol.reasons.append(f"{name}: wheel linear {abs(signed_v):.3f} > {lim.wheel_speed_max}")
        if prev_steer is not None and len(prev_steer) == 4 and dt > 0:
            rate = abs(_shortest_steer_diff(delta, float(prev_steer[k]))) / dt
            if rate > lim.steer_rate_max + 1e-6:
                sol.feasible = False
                sol.reasons.append(f"{name}: steer rate {rate:.2f} > {lim.steer_rate_max}")
        sol.steer.append(delta)
        sol.wheel_linear.append(signed_v)
        sol.wheel_speed.append(signed_v / lim.wheel_radius)
        sol.saturated_steer.append(sat_steer)
        sol.saturated_speed.append(sat_speed)
    return sol


def _wrap_steer(delta: float) -> float:
    """把任意角折叠到 [-π/2, π/2] (轮子可反向滚动的等价转角)."""
    d = math.remainder(delta, math.pi)
    if d > math.pi / 2:
        d -= math.pi
    elif d < -math.pi / 2:
        d += math.pi
    return d


def _shortest_steer_diff(a: float, b: float) -> float:
    """轮子反向滚动等价: 转角差按 π 取最短."""
    d = math.remainder(a - b, math.pi)
    if d > math.pi / 2:
        d -= math.pi
    elif d < -math.pi / 2:
        d += math.pi
    return d


def forward(steer: Sequence[float], wheel_speed: Sequence[float],
            limits: Optional[ChassisLimits] = None) -> Tuple[float, float, float]:
    """四轮 (转角, 轮速) -> 底盘速度 (最小二乘). 用于控制器反馈/一致性校验."""
    lim = limits or ChassisLimits()
    # 每个轮心约束: v + ω × r_i = v_i  ->  [1 0 -y_i; 0 1 x_i] [vx vy wz]^T = v_i
    a = []
    b = []
    for k, name in enumerate(WHEEL_ORDER):
        x, y = WHEEL_POSITIONS[name]
        v = wheel_speed[k] * lim.wheel_radius
        a.append([1.0, 0.0, -y])
        b.append(v * math.cos(steer[k]))
        a.append([0.0, 1.0, x])
        b.append(v * math.sin(steer[k]))
    # 3x3 正规方程
    ata = [[sum(a[r][i] * a[r][j] for r in range(len(a))) for j in range(3)] for i in range(3)]
    atb = [sum(a[r][i] * b[r] for r in range(len(a))) for i in range(3)]
    sol = _solve3(ata, atb)
    return sol[0], sol[1], sol[2]


def _solve3(m: List[List[float]], rhs: List[float]) -> List[float]:
    """3x3 高斯消元 (带主元), 避免为一个 3x3 引入 numpy 依赖."""
    a = [row[:] + [rhs[i]] for i, row in enumerate(m)]
    for col in range(3):
        piv = max(range(col, 3), key=lambda r: abs(a[r][col]))
        a[col], a[piv] = a[piv], a[col]
        p = a[col][col]
        if abs(p) < 1e-12:
            continue
        for j in range(col, 4):
            a[col][j] /= p
        for r in range(3):
            if r == col:
                continue
            f = a[r][col]
            for j in range(col, 4):
                a[r][j] -= f * a[col][j]
    return [a[i][3] for i in range(3)]


def twist_domain(limits: Optional[ChassisLimits] = None,
                 n: int = 12) -> List[Tuple[float, float, float]]:
    """采样可行 twist 域边界 (论文图 2-4 用): 以 (v, ω) 网格标记可行性."""
    lim = limits or ChassisLimits()
    out = []
    for i in range(n + 1):
        v = lim.base_speed_max * i / n
        for j in range(-n, n + 1):
            w = lim.base_yaw_max * j / n
            sol = inverse(v, 0.0, w, lim)
            out.append((v, w, 1.0 if sol.feasible else 0.0))
    return out
