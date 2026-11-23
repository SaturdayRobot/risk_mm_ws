"""Geometry measurements, risk scoring and mode selection are separate ports."""
from dataclasses import dataclass
from math import isfinite
from typing import Protocol
from .contracts import Mode

@dataclass(frozen=True)
class Features:
    clearance_m: float  # whole-robot clearance, including the arm
    closing_speed_mps: float
    stamp_ros_s: float
    map_revision: int
    valid: bool = True

class FeatureExtractor(Protocol):
    def extract(self, snapshot, state) -> Features: ...

class RiskAssessor(Protocol):
    def score(self, features: Features, now_ros_s: float, map_revision: int) -> float: ...

class ModePolicy(Protocol):
    def select(self, score: float, current: Mode) -> Mode: ...

@dataclass(frozen=True)
class ClearanceRisk:
    clear_distance_m: float = 0.6
    critical_ttc_s: float = 3.0
    max_age_s: float = 0.5

    def __post_init__(self):
        if any(not isfinite(v) or v <= 0 for v in (self.clear_distance_m, self.critical_ttc_s, self.max_age_s)):
            raise ValueError("risk scales must be finite and positive")

    def score(self, f: Features, now_ros_s: float, map_revision: int) -> float:
        age = now_ros_s - f.stamp_ros_s
        if (not f.valid or f.map_revision != map_revision or
            not all(map(isfinite, (f.clearance_m, f.closing_speed_mps, age))) or
            not 0 <= age <= self.max_age_s or f.closing_speed_mps < 0):
            raise ValueError("invalid, stale, future or mismatched environment")
        clearance = max(0.0, f.clearance_m)
        distance_risk = max(0.0, 1.0 - clearance / self.clear_distance_m)
        ttc_risk = max(0.0, 1.0 - clearance / (max(f.closing_speed_mps, 1e-12) * self.critical_ttc_s))
        return max(distance_risk, ttc_risk)

@dataclass(frozen=True)
class HysteresisPolicy:
    low: float = 0.3
    high: float = 0.65
    high_risk_mode: Mode = Mode.ARM_FIRST

    def __post_init__(self):
        if not 0 <= self.low < self.high <= 1 or self.high_risk_mode not in Mode:
            raise ValueError("invalid policy configuration")

    def select(self, score: float, current: Mode) -> Mode:
        if not isfinite(score) or not 0 <= score <= 1:
            raise ValueError("risk must be in [0,1]")
        low_mode = Mode.COUPLED if self.high_risk_mode == Mode.ARM_FIRST else Mode.ARM_FIRST
        return self.high_risk_mode if score >= self.high else low_mode if score <= self.low else current
