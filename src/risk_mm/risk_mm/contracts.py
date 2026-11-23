"""ROS independent contracts; state = x, y, yaw, six arm joints (SI)."""
from dataclasses import dataclass
from enum import Enum
from math import isfinite
from typing import Protocol

class Mode(str, Enum):
    COUPLED = "coupled"
    ARM_FIRST = "arm_first"

@dataclass(frozen=True)
class Task:
    task_id: str
    map_revision: int
    start: tuple[float, ...]
    goal: tuple[float, ...]
    model_id: str = "rangermini_xarm6"

    def __post_init__(self):
        if not self.task_id or self.map_revision < 0 or self.model_id != "rangermini_xarm6":
            raise ValueError("invalid task identity")
        if any(len(q) != 9 or not all(map(isfinite, q)) for q in (self.start, self.goal)):
            raise ValueError("state requires nine finite values")

@dataclass(frozen=True)
class Point:
    time_s: float  # relative trajectory time; execution origin must be ROS time
    position: tuple[float, ...]

@dataclass(frozen=True)
class Plan:
    task: Task
    mode: Mode
    points: tuple[Point, ...]
    planning_wall_s: float
    # A candidate is never permission to execute. Validator and executor are ports.

class Planner(Protocol):
    async def plan(self, task: Task) -> Plan: ...

class Validator(Protocol):
    def validate(self, plan: Plan) -> None: ...

class ArmPlanner(Protocol):
    async def stow(self, task: Task, q_stow: tuple[float, ...]) -> tuple[Point, ...]: ...

class BasePlanner(Protocol):
    async def transit(self, task: Task) -> tuple[Point, ...]: ...
