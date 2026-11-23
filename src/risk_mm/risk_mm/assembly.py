"""Configuration and concrete implementation selection belong to this layer."""
from pathlib import Path
import yaml
from .contracts import Mode
from .risk import ClearanceRisk, HysteresisPolicy
from .supervisor import Supervisor
from .execution_gate import StowGate
from .arm_first import ArmFirstPlanner

def load_supervisor(path):
    config = yaml.safe_load(Path(path).read_text())
    policy = dict(config['policy'])
    policy['high_risk_mode'] = Mode(policy['high_risk_mode'])
    return Supervisor(ClearanceRisk(**config['risk']), HysteresisPolicy(**policy),
                      dwell_s=config['switch']['dwell_s'])

def load_arm_first(robot_path, supervisor_path, arm, base, validator):
    robot = yaml.safe_load(Path(robot_path).read_text())
    switch = yaml.safe_load(Path(supervisor_path).read_text())['switch']
    if robot['model_id'] != 'rangermini_xarm6' or not switch['require_measured_stop'] or not switch['require_stow_confirmation_before_base']:
        raise ValueError('current robot and stopped-stow gates are required')
    q = tuple(robot['stow'])
    return (ArmFirstPlanner(arm, base, validator, q),
            StowGate(q, robot['stow_tolerance_rad'], switch['feedback_timeout_s']))
