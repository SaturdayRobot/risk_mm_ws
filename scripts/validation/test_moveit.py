#!/usr/bin/env python3
"""Real MoveIt service smoke test with explicit static fixtures, never execution."""
from pathlib import Path
import json,os,signal,subprocess,time,xml.etree.ElementTree as ET
import yaml
import rclpy
from rclpy.executors import SingleThreadedExecutor
from moveit_msgs.srv import GetStateValidity,ApplyPlanningScene
from moveit_msgs.msg import RobotState,CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory as share
from risk_mm.contracts import Task
from risk_mm.moveit_stow import MoveItStowPlanner

def main():
    ws=Path(__file__).resolve().parents[2]
    report={'status':'failed','execution_success':None,'scope':'synthetic static start and scene; real MoveIt services; no motion execution'}
    process=None;node=None;executor=None
    env=dict(os.environ,ROS_DOMAIN_ID='174',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(ws/'reports/moveit_ros'))
    os.environ.update({k:env[k] for k in ['ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','ROS_LOG_DIR']})
    with (ws/'reports/moveit_launch.log').open('w') as log:
        try:
            process=subprocess.Popen(['ros2','launch','rangermini_moveit_config','move_group.launch.py'],env=env,
                                     stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            rclpy.init();node=rclpy.create_node('moveit_smoke');executor=SingleThreadedExecutor();executor.add_node(node)
            def service(kind,name):
                client=node.create_client(kind,name)
                if not client.wait_for_service(timeout_sec=20.):raise RuntimeError('service unavailable: '+name)
                return client
            validity=service(GetStateValidity,'/check_state_validity')
            apply=service(ApplyPlanningScene,'/apply_planning_scene')
            def call(client,request):
                future=client.call_async(request);executor.spin_until_future_complete(future,timeout_sec=8.)
                if not future.done():raise TimeoutError('MoveIt service timed out')
                return future.result()
            config=yaml.safe_load((ws/'src/mm_bringup/config/robot.yaml').read_text())
            names=config['arm_joint_names'];stow=tuple(config['stow'])
            urdf=ET.parse(Path(share('rangermini_description'))/'urdf/rangermini_xarm6.urdf')
            joints=[j.get('name') for j in urdf.findall('joint') if j.get('type')!='fixed']
            def state(q):
                result=RobotState();positions=dict(zip(names,q));result.joint_state.name=joints
                result.joint_state.position=[positions.get(n,0.) for n in joints]
                result.joint_state.velocity=[0.]*len(joints);result.is_diff=False
                return result
            def check(q):
                request=GetStateValidity.Request();request.group_name='arm';request.robot_state=state(q)
                return call(validity,request)
            def obstacle(blocked):
                request=ApplyPlanningScene.Request();request.scene.is_diff=True
                box=CollisionObject();box.header.frame_id='base_link';box.id='validation_box';box.operation=CollisionObject.ADD
                box.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[2.,2.,2.] if blocked else [.2,.2,.2])]
                pose=Pose();pose.orientation.w=1.;pose.position.x=.2 if blocked else 3.;pose.position.z=1.
                box.primitive_poses=[pose];request.scene.world.collision_objects=[box]
                if not call(apply,request).success:raise RuntimeError('scene rejected')
            obstacle(False)
            valid=check(stow)
            report['stow_valid']=valid.valid
            report['contacts']=[(c.contact_body_1,c.contact_body_2) for c in valid.contacts]
            if not valid.valid:raise RuntimeError('configured stow is in mesh self collision')
            start=list(stow);start[0]+=.03;start=tuple(start)
            if not check(start).valid:raise RuntimeError('smoke start state invalid')
            adapter=MoveItStowPlanner(node,lambda:1,lambda:state(start))
            if not adapter.client.wait_for_service(timeout_sec=5.):raise RuntimeError('planning service unavailable')
            task=Task('moveit-smoke',1,(0.,0.,0.)+start,(0.,0.,0.)+stow)
            future=executor.create_task(adapter.stow(task,stow));executor.spin_until_future_complete(future,timeout_sec=10.)
            if not future.done():raise TimeoutError('stow planning timed out')
            points=future.result()
            if len(points)<2:raise RuntimeError('insufficient trajectory points')
            if any(abs(a-b)>1e-6 for a,b in zip(points[-1].position[3:],stow)):raise RuntimeError('stow goal mismatch')
            for p in points:
                if not check(p.position[3:]).valid:raise RuntimeError('invalid returned trajectory sample')
            obstacle(True)
            if check(stow).valid:raise RuntimeError('MoveIt failed to detect blocking scene object')
            report.update(status='passed',planning_success=True,trajectory_samples=len(points),
                          duration_s=points[-1].time_s,blocked_scene_rejected=True)
        except Exception as error:
            report['reason']=str(error)
        finally:
            if executor:executor.shutdown(timeout_sec=1.)
            if node:node.destroy_node()
            if rclpy.ok():rclpy.shutdown()
            if process and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=8.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5.)
    launch_log=(ws/'reports/moveit_launch.log').read_text()
    report['planning_status']=report['status']
    report['shutdown_success']=not any(token in launch_log for token in ('Segmentation fault','exit code -11','exit code -6'))
    if not report['shutdown_success']:
        report['status']='failed'
        report['reason']='MoveIt process failed during teardown; planning result is recorded separately'
    (ws/'reports/moveit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['status']=='passed' else 1

if __name__=='__main__':raise SystemExit(main())
