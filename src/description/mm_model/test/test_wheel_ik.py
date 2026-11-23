# -*- coding: utf-8 -*-
"""4WIS-4WID 运动学单测: 直行 / 斜行 / 原地回转 / 往返一致 / 限幅."""
from __future__ import annotations

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from mm_model.wheel_ik import (ChassisLimits, WHEEL_POSITIONS, forward, inverse,
                               twist_domain)


class WheelIKTest(unittest.TestCase):
    def setUp(self):
        self.lim = ChassisLimits()

    def test_straight_line(self):
        sol = inverse(0.5, 0.0, 0.0, self.lim)
        self.assertTrue(sol.feasible)
        for d in sol.steer:
            self.assertAlmostEqual(d, 0.0, places=9)
        for w in sol.wheel_speed:
            self.assertAlmostEqual(w, 0.5 / self.lim.wheel_radius, places=6)

    def test_crab_motion(self):
        # 纯横移: 四轮都应打到 ±90° (论文图 2-3(b) 的极限工况)
        sol = inverse(0.0, 0.3, 0.0, self.lim)
        for d in sol.steer:
            self.assertAlmostEqual(abs(d), math.pi / 2, places=6)

    def test_spin_in_place(self):
        sol = inverse(0.0, 0.0, 0.5, self.lim)
        self.assertTrue(sol.feasible, sol.reasons)
        for k, name in enumerate(["fl", "fr", "rl", "rr"]):
            x, y = WHEEL_POSITIONS[name]
            # 轮缘速度 = |ω| * 轮心到原点距离; 转角折叠到 ±90° 内
            self.assertAlmostEqual(abs(sol.wheel_linear[k]), 0.5 * math.hypot(x, y), places=6)
            self.assertLessEqual(abs(sol.steer[k]), math.pi / 2 + 1e-9)
        fx, fy, fw = forward(sol.steer, sol.wheel_speed, self.lim)
        self.assertAlmostEqual(fx, 0.0, places=6)
        self.assertAlmostEqual(fy, 0.0, places=6)
        self.assertAlmostEqual(fw, 0.5, places=6)

    def test_forward_roundtrip(self):
        for vx, vy, wz in [(0.4, 0.0, 0.0), (0.3, 0.2, 0.3), (0.0, 0.0, 0.6),
                           (0.2, -0.15, -0.4)]:
            sol = inverse(vx, vy, wz, self.lim)
            self.assertTrue(sol.feasible, f"{vx},{vy},{wz}: {sol.reasons}")
            fx, fy, fw = forward(sol.steer, sol.wheel_speed, self.lim)
            self.assertAlmostEqual(fx, vx, places=6)
            self.assertAlmostEqual(fy, vy, places=6)
            self.assertAlmostEqual(fw, wz, places=6)

    def test_speed_limit_flags_infeasible(self):
        strict = ChassisLimits(wheel_speed_max=0.3)
        sol = inverse(0.5, 0.0, 0.0, strict)            # 轮缘 0.5 m/s > 0.3
        self.assertFalse(sol.feasible)
        self.assertTrue(any("wheel linear" in r for r in sol.reasons))

    def test_base_speed_limit_flags_infeasible(self):
        sol = inverse(1.5, 0.0, 0.0, self.lim)          # 超底盘设计速度 1.2 m/s
        self.assertFalse(sol.feasible)
        self.assertTrue(any("base speed" in r for r in sol.reasons))

    def test_steer_hold_at_zero_speed(self):
        prev = [0.3, 0.3, 0.3, 0.3]
        sol = inverse(0.0, 0.0, 0.0, self.lim, prev_steer=prev, dt=0.01)
        self.assertEqual([round(d, 6) for d in sol.steer], prev)

    def test_steer_rate_limit(self):
        prev = [0.0, 0.0, 0.0, 0.0]
        sol = inverse(0.0, 0.3, 0.0, self.lim, prev_steer=prev, dt=0.01)  # 90° 阶跃
        self.assertFalse(sol.feasible)
        self.assertTrue(any("steer rate" in r for r in sol.reasons))

    def test_twist_domain_is_full_box_within_limits(self):
        # 结论 (写入 05 文档): 轮缘速度上限 2.0 m/s 相对底盘 1.2 m/s + 1.2 rad/s
        # 仍有余量, 因此可行 twist 域就是 (v, ω) 矩形本身; 真正的瓶颈是转向速率。
        dom = twist_domain(self.lim, n=6)
        self.assertTrue(all(f == 1.0 for _, _, f in dom))
        self.assertFalse(inverse(1.3, 0.0, 0.0, self.lim).feasible)
        self.assertFalse(inverse(0.0, 0.0, 1.3, self.lim).feasible)


if __name__ == "__main__":
    unittest.main(verbosity=2)
