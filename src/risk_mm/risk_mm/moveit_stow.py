"""MoveIt 2 GetMotionPlan adapter. Plans one joint target; never executes."""
from .contracts import Point
from math import isfinite

class MoveItStowPlanner:
    def __init__(self, node, scene_revision, robot_state, service='/plan_kinematic_path'):
        from moveit_msgs.srv import GetMotionPlan
        self.request_type = GetMotionPlan.Request
        self.client = node.create_client(GetMotionPlan, service)
        self.scene_revision = scene_revision  # frozen planning-scene revision provider
        self.robot_state = robot_state      # full measured RobotState provider

    async def stow(self, task, q_stow):
        from moveit_msgs.msg import Constraints, JointConstraint
        if len(q_stow) != 6 or not all(map(isfinite, q_stow)):
            raise ValueError('six finite stow angles required')
        names = [f'xarm6_joint{i}' for i in range(1,7)]
        if self.scene_revision() != task.map_revision or not self.client.service_is_ready():
            raise RuntimeError('MoveIt scene/service unavailable')
        request = self.request_type()
        req = request.motion_plan_request
        req.group_name = 'arm'
        req.planner_id = 'RRTConnect'
        req.num_planning_attempts = 1
        req.allowed_planning_time = 2.0
        req.max_velocity_scaling_factor = 1.0
        req.max_acceleration_scaling_factor = 1.0
        req.start_state = self.robot_state()
        js = req.start_state.joint_state
        measured = dict(zip(js.name, js.position))
        if any(name not in measured or abs(measured[name]-task.start[i+3]) > 1e-6 for i,name in enumerate(names)):
            raise ValueError('MoveIt start state differs from task snapshot')
        constraints = Constraints()
        for name,value in zip(names,q_stow):
            constraints.joint_constraints.append(JointConstraint(joint_name=name, position=value,
                tolerance_above=1e-6,tolerance_below=1e-6,weight=1.0))
        req.goal_constraints = [constraints]
        response = (await self.client.call_async(request)).motion_plan_response
        if response.error_code.val != 1 or self.scene_revision() != task.map_revision:
            raise RuntimeError('MoveIt failed or scene changed')
        trajectory = response.trajectory.joint_trajectory
        if set(trajectory.joint_names) != set(names):
            raise ValueError('MoveIt joint names mismatch')
        indices = [trajectory.joint_names.index(name) for name in names]
        points = tuple(Point(p.time_from_start.sec+p.time_from_start.nanosec*1e-9,
            task.start[:3]+tuple(p.positions[i] for i in indices)) for p in trajectory.points)
        if not points:
            raise RuntimeError('MoveIt returned empty trajectory')
        if points[0].time_s > 0:
            points = (Point(0.0, task.start),)+points
        return points
