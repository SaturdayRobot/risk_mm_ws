import asyncio
from dataclasses import replace
from pathlib import Path
import sys
import pytest

WS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WS/'src/risk_mm'))
from risk_mm.contracts import Task, Plan, Point, Mode
from risk_mm.risk import ClearanceRisk, HysteresisPolicy, Features
from risk_mm.features import GeometrySnapshot, BoxClearance
from risk_mm.supervisor import Supervisor
from risk_mm.arm_first import ArmFirstPlanner
from risk_mm.execution_gate import StowGate

STOW = (0., -.76, -1.62, -.06, 1.11, .02)
START = (0., 0., 0.)+(0.,)*6
GOAL = (1., 0., 0.)+STOW
TASK = Task('test', 4, START, GOAL)

def test_geometry_distance_and_missing_map():
    f = BoxClearance().extract(GeometrySnapshot(2.,4,((1.,-1.,-1.,2.,1.,1.),),True),((0.,0.,0.,.2),))
    assert f.valid and f.clearance_m == pytest.approx(.8)
    assert not BoxClearance().extract(GeometrySnapshot(2.,4,(),True),((0.,0.,0.,.2),)).valid

@pytest.mark.parametrize('f,now', [(Features(.2,0,0,4),2.), (Features(.2,0,3,4),2.),
    (Features(float('nan'),0,2,4),2.), (Features(.2,0,2,3),2.), (Features(.2,-1,2,4),2.)])
def test_risk_rejects_unusable_input(f,now):
    with pytest.raises(ValueError):ClearanceRisk().score(f,now,4)

def test_policy_hysteresis_and_ttc():
    risk=ClearanceRisk()
    assert risk.score(Features(.6,.6,2.,4),2.,4) > .6
    policy=HysteresisPolicy()
    assert policy.select(.5,Mode.COUPLED)==Mode.COUPLED
    assert policy.select(.5,Mode.ARM_FIRST)==Mode.ARM_FIRST
    assert policy.select(.9,Mode.COUPLED)==Mode.ARM_FIRST
    assert policy.select(.1,Mode.ARM_FIRST)==Mode.COUPLED

def prepare(supervisor,plan,**changes):
    args=dict(stopped=True,validated=True,task_id='test',map_revision=4,now_ros_s=2.)
    args.update(changes)
    return supervisor.prepare(plan,**args)

def test_switch_actual_mode_and_invalidated_acknowledgement():
    s=Supervisor(ClearanceRisk(),HysteresisPolicy())
    plan=Plan(TASK,Mode.ARM_FIRST,(),.1)
    with pytest.raises(ValueError):prepare(s,plan)
    s.observe(Features(.1,0,2.,4),2.,10.,4)
    with pytest.raises(ValueError):prepare(s,plan,stopped=False)
    token=prepare(s,plan)
    assert s.actual==Mode.COUPLED
    s.observe(Features(.1,0,2.,5),2.,10.,5)
    with pytest.raises(ValueError):s.started(token,10.)
    assert s.actual==Mode.COUPLED and s.phase=='HOLD'

def test_switch_execution_failure_and_no_implicit_success():
    s=Supervisor(ClearanceRisk(),HysteresisPolicy())
    s.observe(Features(.1,0,2.,4),2.,10.,4)
    plan=Plan(TASK,Mode.ARM_FIRST,(),.1)
    token=prepare(s,plan)
    s.started(token,10.)
    assert s.actual==Mode.ARM_FIRST
    with pytest.raises(ValueError):s.finished(token,reached=False,stopped=True)
    assert s.phase=='HOLD' and s.actual==Mode.ARM_FIRST

def test_prepare_rechecks_freshness():
    s=Supervisor(ClearanceRisk(),HysteresisPolicy())
    s.observe(Features(.1,0,2.,4),2.,10.,4)
    with pytest.raises(ValueError):prepare(s,Plan(TASK,Mode.ARM_FIRST,(),.1),now_ros_s=3.)

class Arm:
    async def stow(self,task,q):return (Point(0.,task.start),Point(1.,task.start[:3]+q))
class Base:
    async def transit(self,task):return (Point(0.,task.start),Point(2.,task.goal))
class Validator:
    def validate(self,plan):pass

def test_arm_first_two_stages_and_no_other_pose():
    planner=ArmFirstPlanner(Arm(),Base(),Validator(),STOW)
    result=asyncio.run(planner.plan(TASK))
    assert [p.time_s for p in result.points]==[0.,1.,3.]
    assert result.points[-1].position==GOAL
    with pytest.raises(ValueError):asyncio.run(planner.plan(replace(TASK,goal=(1.,0.,0.)+(0.,)*6)))

def test_arm_first_stow_failure_prevents_base_planning():
    class BadArm:
        async def stow(self,task,q):raise RuntimeError('collision')
    class ForbiddenBase:
        async def transit(self,task):raise AssertionError('base must not be called')
    with pytest.raises(RuntimeError,match='collision'):
        asyncio.run(ArmFirstPlanner(BadArm(),ForbiddenBase(),Validator(),STOW).plan(TASK))

@pytest.mark.parametrize('q,vel,stamp', [(STOW,(0.,)*6,0.), ((0.,)*6,(0.,)*6,2.), (STOW,(.2,)*6,2.)])
def test_base_requires_actual_fresh_stopped_stow(q,vel,stamp):
    with pytest.raises(ValueError):StowGate(STOW).require(q,vel,(0.,0.,0.),stamp,2.)

def test_base_stow_gate_and_config_consistency():
    import yaml
    import xml.etree.ElementTree as ET
    StowGate(STOW).require(STOW,(0.,)*6,(0.,0.,0.),2.,2.)
    config=yaml.safe_load((WS/'src/mm_bringup/config/robot.yaml').read_text())
    srdf=ET.parse(WS/'src/rangermini_moveit_config/config/rangermini_xarm6.srdf')
    assert tuple(config['stow'])==STOW
    assert tuple(float(j.get('value')) for j in srdf.findall('.//group_state/joint'))==STOW
    assert len(srdf.findall('.//group_state'))==1

def test_stow_gate_rejects_rotating_base():
    with pytest.raises(ValueError):StowGate(STOW).require(STOW,(0.,)*6,(0.,0.,.2),2.,2.)

def test_composition_reads_workspace_config():
    from risk_mm.assembly import load_supervisor, load_arm_first
    config=WS/'src/mm_bringup/config'
    supervisor=load_supervisor(config/'supervisor.yaml')
    assert supervisor.observe(Features(.1,0.,2.,4),2.,10.,4)[1]==Mode.ARM_FIRST
    planner,gate=load_arm_first(config/'robot.yaml',config/'supervisor.yaml',Arm(),Base(),Validator())
    assert planner.q_stow==gate.q_stow==STOW


def test_fixed_attachment_exemption_is_a_rigid_connection():
    import xml.etree.ElementTree as ET
    urdf=ET.parse(WS/'src/description/rangermini_description/urdf/rangermini_xarm6.urdf')
    edges={}
    for joint in urdf.findall('joint'):
        if joint.get('type')=='fixed':
            a=joint.find('parent').get('link');b=joint.find('child').get('link')
            edges.setdefault(a,set()).add(b);edges.setdefault(b,set()).add(a)
    seen={'xarm6_link6'};pending=list(seen)
    while pending:
        for child in edges.get(pending.pop(),()):
            if child not in seen:seen.add(child);pending.append(child)
    assert 'head_link' in seen
    srdf=ET.parse(WS/'src/rangermini_moveit_config/config/rangermini_xarm6.srdf')
    assert any({pair.get('link1'),pair.get('link2')}=={'xarm6_link6','head_link'} for pair in srdf.findall('disable_collisions'))
