"""Measured stow gate to be called by the execution adapter before base motion."""
from math import isfinite

class StowGate:
    def __init__(self, q_stow, tolerance_rad=0.03, timeout_s=0.5, velocity_tolerance=0.01):
        self.q_stow = tuple(q_stow)
        self.tolerance, self.timeout, self.velocity_tolerance = tolerance_rad, timeout_s, velocity_tolerance
        if len(self.q_stow) != 6 or not all(map(isfinite, self.q_stow)) or any(not isfinite(x) or x <= 0 for x in (tolerance_rad, timeout_s, velocity_tolerance)):
            raise ValueError("invalid stow gate configuration")

    def require(self, q, joint_velocities, base_twist, stamp_ros_s, now_ros_s):
        # base_twist = vx, vy, yaw_rate; rotation also counts as base motion.
        values = tuple(q) + tuple(joint_velocities) + tuple(base_twist) + (stamp_ros_s, now_ros_s)
        if len(q) != 6 or len(joint_velocities) != 6 or len(base_twist) != 3 or not all(map(isfinite, values)):
            raise ValueError("invalid measured feedback")
        if (not 0 <= now_ros_s-stamp_ros_s <= self.timeout or
            any(abs(v) > self.velocity_tolerance for v in base_twist) or
            any(abs(v) > self.velocity_tolerance for v in joint_velocities) or
            any(abs(a-b) > self.tolerance for a,b in zip(q, self.q_stow))):
            raise ValueError("base transit requires fresh, stopped, measured stow")
