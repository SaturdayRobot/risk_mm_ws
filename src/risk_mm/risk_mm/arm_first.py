"""Only STOW -> TRANSIT. No posture library, restoration or mode fallback."""
from dataclasses import replace
from math import isfinite
from time import monotonic
from .contracts import ArmPlanner, BasePlanner, Validator, Task, Plan, Point, Mode

def check_points(points, start, goal):
    if len(points) < 2 or abs(points[0].time_s) > 1e-9:
        raise ValueError("stage requires start, end and zero origin")
    previous = -1.0
    for point in points:
        if (not isfinite(point.time_s) or point.time_s <= previous or
            len(point.position) != 9 or not all(map(isfinite, point.position))):
            raise ValueError("invalid stage sample or time")
        previous = point.time_s
    if any(abs(a-b) > 1e-6 for a,b in zip(points[0].position, start)) or any(abs(a-b) > 1e-6 for a,b in zip(points[-1].position, goal)):
        raise ValueError("stage boundary mismatch")

class ArmFirstPlanner:
    def __init__(self, arm: ArmPlanner, base: BasePlanner, validator: Validator, q_stow):
        self.arm, self.base, self.validator = arm, base, validator
        self.q_stow = tuple(q_stow)
        if len(self.q_stow) != 6 or not all(map(isfinite, self.q_stow)):
            raise ValueError("one finite six-joint stow target required")

    async def plan(self, task: Task) -> Plan:
        started = monotonic()
        if any(abs(a-b) > 1e-6 for a,b in zip(task.goal[3:], self.q_stow)):
            raise ValueError("arm_first only supports a stowed arm goal")
        intermediate = task.start[:3] + self.q_stow
        arm = await self.arm.stow(task, self.q_stow)
        check_points(arm, task.start, intermediate)
        if any(p.position[:3] != task.start[:3] for p in arm):
            raise ValueError("base moved during stow")
        self.validator.validate(Plan(task, Mode.ARM_FIRST, arm, monotonic()-started))
        base_task = replace(task, start=intermediate)
        base = await self.base.transit(base_task)
        check_points(base, intermediate, task.goal)
        if any(p.position[3:] != self.q_stow for p in base):
            raise ValueError("arm moved during transit")
        points = tuple(arm) + tuple(Point(p.time_s+arm[-1].time_s, p.position) for p in base[1:])
        result = Plan(task, Mode.ARM_FIRST, points, monotonic()-started)
        self.validator.validate(result)
        return result
