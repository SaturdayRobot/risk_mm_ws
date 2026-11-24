#!/usr/bin/env python3
"""Exercise typed coupled services against the recorded isolated Gazebo plant."""
import argparse
import json
import math
import yaml
import os
from pathlib import Path
import time

import rclpy
from rclpy.parameter import Parameter
from risk_mm_interfaces.msg import CoupledStatus
from risk_mm_interfaces.srv import PlanCoupled, ExecutePlan
from std_srvs.srv import Trigger
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--distance', type=float, default=.6)
    parser.add_argument('--session', type=Path)
    parser.add_argument('--abort-on-first', action='store_true', help='test scene invalidation during the first motion only')
    args = parser.parse_args()
    if not 0. < args.distance <= 1.:
        raise ValueError('acceptance distance must be in (0, 1] m')
    ws = Path(__file__).resolve().parents[2]
    session = json.loads((args.session or ws / 'reports/gazebo_latest.json').read_text())
    folder = Path(session['report_dir'])
    number = 1
    while (folder / f'coupled_validation_{number}.json').exists():
        number += 1
    output = folder / f'coupled_validation_{number}.json'
    report = {'distance_m': args.distance, 'status': 'failed', 'candidate_generated': False, 'validation_passed': False,
              'execution_started': False, 'execution_completed': False, 'actual_mode': None, 'phases': []}
    node = None
    latest = None
    started = time.monotonic()
    clients = []
    probe_spawned = False
    measured = {}
    try:
        assert int(os.environ['ROS_DOMAIN_ID']) == session['domain_id']
        rclpy.init()
        node = rclpy.create_node('coupled_execution_validation', parameter_overrides=[Parameter('use_sim_time', value=True)])

        def receive(message):
            nonlocal latest
            latest = message
            if not report['phases'] or report['phases'][-1]['phase'] != message.phase:
                report['phases'].append({'phase': message.phase, 'wall_s': time.monotonic()-started,
                    'reason': message.reason, 'candidate': message.candidate_generated,
                    'validated': message.validation_passed, 'started': message.execution_started,
                    'completed': message.execution_completed})

        subscription = node.create_subscription(CoupledStatus, '/coupled/status', receive, 10)
        joints = node.create_subscription(JointState, '/joint_states', lambda m: measured.update(joints=m), qos_profile_sensor_data)
        odom = node.create_subscription(Odometry, '/ground_truth/odom', lambda m: measured.update(odom=m), qos_profile_sensor_data)
        def measured_stopped():
            if 'joints' not in measured or 'odom' not in measured:
                return False
            joint = measured['joints']
            arm = [abs(v) for name, v in zip(joint.name, joint.velocity) if name.startswith('xarm6_joint')]
            twist = measured['odom'].twist.twist
            return len(arm) == 6 and max(arm) < .01 and math.hypot(twist.linear.x, twist.linear.y) < .005 and abs(twist.angular.z) < .01
        def stable_stop():
            stable = None
            def condition():
                nonlocal stable
                now = node.get_clock().now().nanoseconds*1e-9
                if not measured_stopped():
                    stable = None
                    return False
                if stable is None:
                    stable = now
                return now-stable > .3
            wait(condition, 15.)

        def wait(condition, timeout=30.):
            deadline = time.monotonic()+timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.03)
                if condition():
                    return
            raise TimeoutError(f'condition timed out; last phase={latest.phase if latest else None}, reason={latest.reason if latest else None}')

        def call(kind, topic, request):
            client = node.create_client(kind, topic)
            clients.append(client)
            if not client.wait_for_service(timeout_sec=5.):
                raise RuntimeError('service missing: '+topic)
            future = client.call_async(request)
            wait(future.done, 15.)
            return future.result()

        wait(lambda: latest and latest.map_ready and latest.feedback_ready)
        stable_stop()
        initial = list(latest.state)
        report['initial_state'] = initial
        report['model_signature'] = latest.model_signature
        report['map_revision'] = latest.map_revision
        report['initial_yaw_error_rad'] = abs(math.remainder(initial[2]-math.pi, 2*math.pi))
        assert report['initial_yaw_error_rad'] < .02, 'whole robot initial world heading differs from pi'
        request = PlanCoupled.Request(task_id='coupled_acceptance', model_signature=latest.model_signature,
                                     map_revision=latest.map_revision, goal=initial)
        manual = folder/'inputs/mm_bringup/config/manual_control.yaml'
        if not manual.exists():
            manual = ws/'src/mm_bringup/config/manual_control.yaml'
        drive_yaw = yaml.safe_load(manual.read_text())['control_frame_yaw_in_base_rad']
        report['drive_yaw_in_base_rad'] = drive_yaw
        request.goal[0] += args.distance*math.cos(initial[2]+drive_yaw)
        request.goal[1] += args.distance*math.sin(initial[2]+drive_yaw)
        request.goal[3] += .04
        request.model_signature = 'wrong-model'
        rejected = call(PlanCoupled, '/coupled/plan', request)
        assert not rejected.accepted
        report['wrong_model_rejected'] = True
        request.model_signature = latest.model_signature
        request.map_revision = 0
        assert not call(PlanCoupled, '/coupled/plan', request).accepted
        report['wrong_map_rejected'] = True
        request.map_revision = latest.map_revision
        accepted = call(PlanCoupled, '/coupled/plan', request)
        assert accepted.accepted, accepted.reason
        plan_id = accepted.plan_id
        wait(lambda: latest.plan_id == plan_id and latest.phase != 'PLANNING', 150.)
        report.update(candidate_generated=latest.candidate_generated, validation_passed=latest.validation_passed,
                      planning_wall_s=latest.planning_wall_s, validation_wall_s=latest.validation_wall_s,
                      duration_s=latest.duration_s, plan_id=plan_id, candidate_samples=latest.candidate_samples)
        assert latest.ready_to_execute, latest.reason
        assert not latest.execution_started and not latest.execution_completed
        stable_stop()
        request_execute = ExecutePlan.Request(plan_id=plan_id, model_signature=latest.model_signature,
                                             map_revision=latest.plan_map_revision)
        request_execute.plan_id = 'wrong-plan'
        assert not call(ExecutePlan, '/coupled/execute', request_execute).accepted
        report['wrong_plan_rejected'] = True
        request_execute.plan_id = plan_id
        executed = call(ExecutePlan, '/coupled/execute', request_execute)
        assert executed.accepted, executed.reason
        if not args.abort_on_first:
            wait(lambda: latest.plan_id == plan_id and latest.phase in ('COMPLETED', 'HOLD'), latest.duration_s+35.)
            report.update(execution_started=latest.execution_started, execution_completed=latest.execution_completed,
                          execution_success=latest.execution_success, actual_mode=latest.actual_mode or None,
                          final_state=list(latest.state), execution_reason=latest.reason)
            assert latest.execution_completed and latest.execution_success, latest.reason
            final = list(latest.state)
            report['endpoint_error_m'] = math.hypot(final[0]-request.goal[0], final[1]-request.goal[1])
            report['arm_endpoint_error_rad'] = max(abs(final[i]-request.goal[i]) for i in range(3, 9))
            assert report['endpoint_error_m'] < .04 and report['arm_endpoint_error_rad'] < .03
            report['scope'] = 'coupled goal completion and identity gates'
            report['status'] = 'passed'
            return 0
        else:
            report['scope'] = 'intentional scene invalidation during validated motion'
            report['expected_abort'] = True
            wait(lambda: latest.execution_started or latest.phase == 'HOLD', 15.)
            assert latest.execution_started and latest.phase != 'HOLD', latest.reason
            report.update(execution_started=True, execution_completed=False, execution_success=False,
                          actual_mode=latest.actual_mode or None)
        # Fresh scene input must invalidate on an unmodelled spawned object.
        model = '<sdf version="1.6"><model name="scene_lease_probe"><static>true</static><pose>10 6 0 0 0 0</pose><link name="body"><visual name="v"><geometry><box><size>.01 .01 .01</size></box></geometry></visual></link></model></sdf>'
        spawned = call(SpawnEntity, '/spawn_entity', SpawnEntity.Request(name='scene_lease_probe', xml=model))
        assert spawned.success, spawned.status_message
        probe_spawned = True
        wait(lambda: not latest.map_ready and latest.phase == 'HOLD')
        assert latest.execution_started and not latest.execution_completed and not latest.execution_success
        report['scene_abort'] = {'plan_id': latest.plan_id, 'execution_started': latest.execution_started,
                                 'execution_completed': latest.execution_completed, 'reason': latest.reason}
        wait(measured_stopped, 8.)
        held = list(latest.state)
        hold_until = node.get_clock().now().nanoseconds*1e-9+1.
        wait(lambda: node.get_clock().now().nanoseconds*1e-9 > hold_until)
        drift = math.hypot(latest.state[0]-held[0], latest.state[1]-held[1])
        assert drift < .01, f'base drift after scene abort: {drift}'
        report['scene_abort']['stopped_drift_m'] = drift
        report['unknown_scene_rejected'] = True
        old_revision = request.map_revision
        assert latest.map_revision != old_revision
        assert call(DeleteEntity, '/delete_entity', DeleteEntity.Request(name='scene_lease_probe')).success
        probe_spawned = False
        wait(lambda: latest.map_ready)
        report['scene_revision_after_restore'] = latest.map_revision
        assert latest.map_revision != old_revision
        stopped = call(Trigger, '/simulation/stop', Trigger.Request())
        assert stopped.success
        assert not call(ExecutePlan, '/coupled/execute', request_execute).accepted
        report['stopped_token_rejected'] = True
        report['status'] = 'passed'
    except Exception as error:
        report['reason'] = str(error)
    finally:
        if node:
            try:
                if probe_spawned:
                    call(DeleteEntity, '/delete_entity', DeleteEntity.Request(name='scene_lease_probe'))
                call(Trigger, '/simulation/stop', Trigger.Request())
            except Exception:
                pass
            for client in clients:
                node.destroy_client(client)
            node.destroy_node()
            rclpy.shutdown()
        report['wall_s'] = time.monotonic()-started
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        (folder / 'coupled_latest.json').write_text(json.dumps({'report': str(output), 'status': report['status']}, indent=2)+'\n')
        print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
