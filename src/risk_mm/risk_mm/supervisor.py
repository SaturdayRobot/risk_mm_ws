"""Single mode authority. Execution feedback, never a plan, confirms actual mode."""
from math import isfinite
from .contracts import Mode, Plan
from .risk import RiskAssessor, ModePolicy

class Supervisor:
    def __init__(self, risk: RiskAssessor, policy: ModePolicy, dwell_s=1.0):
        if not isfinite(dwell_s) or dwell_s < 0:
            raise ValueError("invalid dwell")
        self.risk, self.policy, self.dwell_s = risk, policy, dwell_s
        self.actual = Mode.COUPLED
        self.desired = self.actual
        self.phase = "HOLD"  # a stopped, fresh snapshot must arrive first
        self.generation = 0
        self.last_switch = float('-inf')
        self.pending = None
        self.observation = None

    def observe(self, features, now_ros_s, now_wall_s, map_revision):
        if not isfinite(now_wall_s):
            self.hold()
            raise ValueError("invalid monotonic time")
        try:
            score = self.risk.score(features, now_ros_s, map_revision)
            desired = self.policy.select(score, self.actual)
        except ValueError:
            self.hold()
            raise
        if now_wall_s - self.last_switch >= self.dwell_s:
            self.desired = desired
        if self.pending and (self.pending.task.map_revision != map_revision or self.pending.mode != self.desired):
            self.hold()
        self.observation = features
        return score, self.desired

    def prepare(self, plan: Plan, *, stopped: bool, validated: bool, task_id: str, map_revision: int, now_ros_s: float):
        if self.observation is None:
            raise ValueError("fresh environment observation required")
        try:
            self.risk.score(self.observation, now_ros_s, map_revision)
        except ValueError:
            self.hold()
            raise
        if (self.phase == "EXECUTING" or not stopped or not validated or plan.mode != self.desired or
            plan.task.task_id != task_id or plan.task.map_revision != map_revision):
            raise ValueError("switch requires stopped state, matching identity and validated plan")
        self.generation += 1
        self.pending = plan
        self.phase = "PREPARED"
        return self.generation

    def started(self, generation, now_wall_s):
        if self.phase != "PREPARED" or generation != self.generation or not isfinite(now_wall_s):
            raise ValueError("unexpected execution acknowledgement")
        self.actual = self.pending.mode
        self.phase = "EXECUTING"
        self.last_switch = now_wall_s

    def finished(self, generation, *, reached: bool, stopped: bool):
        if self.phase != "EXECUTING" or generation != self.generation or not reached or not stopped:
            self.hold()
            raise ValueError("execution did not finish safely")
        self.pending = None
        self.phase = "IDLE"

    def hold(self):
        self.generation += 1
        self.pending = None
        self.observation = None
        self.phase = "HOLD"
