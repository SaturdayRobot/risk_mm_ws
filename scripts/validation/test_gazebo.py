#!/usr/bin/env python3
"""Exercise the running isolated Gazebo plant; record measured manual motion."""
import argparse
import json
import math
import os
from pathlib import Path
import time

import numpy as np
from PIL import Image as PILImage
from controller_manager_msgs.srv import ListControllers
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--session', type=Path)
    args = parser.parse_args()
    ws = Path(__file__).resolve().parents[2]
    session = json.loads((args.session or ws / 'reports/gazebo_latest.json').read_text())
    folder = Path(session['report_dir'])
    previous = folder / 'validation.json'
    if previous.exists():
        index = 1
        while (folder / f'validation_attempt_{index}.json').exists():
            index += 1
        previous.rename(folder / f'validation_attempt_{index}.json')
    report = {'status': 'failed', 'scope': session['scope'], 'stages': [],
              'planning_candidate': None, 'common_trajectory_validation': None,
              'actual_executed_mode': None, 'manual_motion_started': False,
              'manual_motion_completed': False}
    latest = {}
    node = None
    board = False
    started = time.monotonic()

    def save():
        (folder / 'validation.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')

    try:
        if int(os.environ['ROS_DOMAIN_ID']) != session['domain_id']:
            raise RuntimeError('domain differs from the recorded simulator session')
        rclpy.init()
        node = rclpy.create_node('gazebo_validation', parameter_overrides=[Parameter('use_sim_time', value=True)])
        subscriptions = []
        for kind, topic, key in [(JointState, '/joint_states', 'joints'),
                                  (Odometry, '/ground_truth/odom', 'odom'),
                                  (Image, '/camera/head/image_raw', 'image')]:
            subscriptions.append(node.create_subscription(kind, topic,
                lambda m, key=key: latest.update({key: m}), qos_profile_sensor_data))
        subscriptions.append(node.create_subscription(String, '/simulation/control_status',
            lambda m: latest.update(status=json.loads(m.data)), 10))
        base_pub = node.create_publisher(TwistStamped, '/simulation/cmd_vel', 10)
        arm_pub = node.create_publisher(JointState, '/simulation/arm_command', 10)
        buffer = Buffer(node=node)
        listener = TransformListener(buffer, node)
        names = [f'xarm6_joint{i}' for i in range(1, 7)]
        config = yaml.safe_load((ws / 'src/mm_bringup/config/robot.yaml').read_text())
        manual = folder/'inputs/mm_bringup/config/manual_control.yaml'
        if not manual.exists():
            manual = ws/'src/mm_bringup/config/manual_control.yaml'
        drive_yaw = yaml.safe_load(manual.read_text())['control_frame_yaw_in_base_rad']
        front_axis = [math.cos(drive_yaw), math.sin(drive_yaw), 0.]
        report['drive_yaw_in_base_rad'] = drive_yaw

        def wait(predicate, timeout=25.):
            until = time.monotonic() + timeout
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.02)
                if predicate():
                    return
            raise TimeoutError('simulator condition timed out')

        def call(kind, topic, request):
            client = node.create_client(kind, topic)
            try:
                if not client.wait_for_service(timeout_sec=15.):
                    raise RuntimeError('service unavailable: ' + topic)
                future = client.call_async(request)
                wait(future.done, 20.)
                return future.result()
            finally:
                node.destroy_client(client)

        def now():
            return node.get_clock().now().nanoseconds / 1e9

        def snapshot():
            odom = latest['odom']
            p, q, twist = odom.pose.pose.position, odom.pose.pose.orientation, odom.twist.twist
            joints = latest['joints']
            values = dict(zip(joints.name, joints.position))
            rpy = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')
            return {'ros_s': now(), 'position': [p.x, p.y, p.z], 'rpy': rpy.tolist(),
                    'speed_m_s': math.hypot(twist.linear.x, twist.linear.y),
                    'yaw_rate_rad_s': twist.angular.z,
                    'arm': [values[n] for n in names],
                    'steer': [values['joint_steer_' + w] for w in ['fl', 'fr', 'rl', 'rr']],
                    'joint_velocity': dict(zip(joints.name, joints.velocity))}

        def command_base(velocity):
            message = TwistStamped()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = 'base_link'
            message.twist.linear.x, message.twist.linear.y, message.twist.angular.z = velocity
            base_pub.publish(message)

        def stage(name, duration, velocity=None, arm=None):
            if name in ('base_longitudinal', 'base_reverse'):
                align_wall, align_ros = time.monotonic(), now()
                while max(abs(v) for v in snapshot()['steer']) > .04:
                    if time.monotonic() - align_wall > 20. or now() - align_ros > 8.:
                        raise TimeoutError('steering did not align for ' + name)
                    command_base(velocity)
                    rclpy.spin_once(node, timeout_sec=.01)
            first = snapshot()
            wall, ros_start = time.monotonic(), now()
            history = []
            while now() - ros_start < duration:
                if time.monotonic() - wall > 60.:
                    raise TimeoutError('simulation stalled during ' + name)
                if velocity is not None:
                    command_base(velocity)
                if arm is not None:
                    message = JointState()
                    message.header.stamp = node.get_clock().now().to_msg()
                    message.name, message.position = names, list(arm)
                    arm_pub.publish(message)
                rclpy.spin_once(node, timeout_sec=.01)
                if not latest['status']['ready']:
                    raise RuntimeError('feedback became unhealthy during ' + name)
                if not history or now() - history[-1]['ros_s'] >= .1:
                    history.append(snapshot())
            final = snapshot()
            record = {'name': name, 'start': first, 'finish': final, 'samples': history,
                      'wall_s': time.monotonic() - wall}
            report['stages'].append(record)
            save()
            print('completed:', name, flush=True)
            return first, final

        wait(lambda: all(k in latest for k in ['joints', 'odom', 'image', 'status']) and
             latest['status']['ready'] and now() > 0.)
        controllers = call(ListControllers, '/controller_manager/list_controllers', ListControllers.Request())
        report['controllers'] = {c.name: c.state for c in controllers.controller}
        expected = ['joint_state_broadcaster', 'arm_position_controller', 'steer_position_controller', 'wheel_velocity_controller']
        if any(report['controllers'].get(name) != 'active' for name in expected):
            raise RuntimeError('not all four controllers are active')
        _, idle = stage('initial_hold', 3., velocity=(0., 0., 0.))
        error = max(abs(a-b) for a, b in zip(idle['arm'], config['display_arm']))
        report['initial_arm_error_rad'] = error
        if error > .03:
            raise RuntimeError('initial arm failed to hold the configured pose')
        arm = list(idle['arm'])
        target = list(arm)
        target[0] += .08
        first, final = stage('arm_step', 3., velocity=(0., 0., 0.), arm=target)
        report['manual_motion_started'] = abs(final['arm'][0] - first['arm'][0]) > .04
        if not report['manual_motion_started'] or max(abs(a-b) for a, b in zip(final['arm'], target)) > .03:
            raise RuntimeError('arm step did not reach the commanded position')
        first, final = stage('arm_hold_without_commands', 4.)
        if max(abs(a-b) for a, b in zip(first['arm'], final['arm'])) > .01:
            raise RuntimeError('arm drifted while holding without commands')
        stage('arm_return', 3., arm=config['display_arm'])
        first, final = stage('base_longitudinal', 2., velocity=(.12*front_axis[0], .12*front_axis[1], 0.))
        report['base_translation_m'] = math.dist(first['position'][:2], final['position'][:2])
        body_forward = Rotation.from_euler('xyz', first['rpy']).apply(front_axis)
        report['base_forward_projected_m'] = float((np.array(final['position']) - first['position']) @ body_forward)
        if not .08 < report['base_translation_m'] < .5:
            raise RuntimeError('base did not translate as commanded')
        if report['base_forward_projected_m'] < .08:
            raise RuntimeError('positive control X did not move toward the user-confirmed front')
        _, stopped = stage('base_command_timeout', 1.5)
        if stopped['speed_m_s'] > .02 or abs(stopped['yaw_rate_rad_s']) > .03:
            raise RuntimeError('base did not stop after command timeout')
        first, final = stage('base_reverse', 2., velocity=(-.12*front_axis[0], -.12*front_axis[1], 0.))
        body_forward = Rotation.from_euler('xyz', first['rpy']).apply(front_axis)
        report['base_reverse_projected_m'] = float((np.array(final['position']) - first['position']) @ body_forward)
        if report['base_reverse_projected_m'] > -.08:
            raise RuntimeError('negative control X did not move away from the user-confirmed front')
        stage('reverse_stop', 1., velocity=(0., 0., 0.))
        first, final = stage('lateral_command_rejected', .7, velocity=(0., .08, 0.))
        report['unsupported_lateral_drift_m'] = math.dist(first['position'][:2], final['position'][:2])
        if report['unsupported_lateral_drift_m'] > .02:
            raise RuntimeError('unsupported lateral command moved the robot')
        stage('base_stop', 1., velocity=(0., 0., 0.))
        first, final = stage('base_rotation', 4., velocity=(0., 0., .15))
        report['base_rotation_rad'] = math.remainder(final['rpy'][2] - first['rpy'][2], 2 * math.pi)
        if report['base_rotation_rad'] < .08:
            raise RuntimeError('four-wheel steering rotation failed')
        stage('final_stop', 1.5, velocity=(0., 0., 0.))
        wait(lambda: buffer.can_transform('world', 'head_camera_optical_frame', rclpy.time.Time()))
        head = buffer.lookup_transform('head_link', 'head_camera_optical_frame', rclpy.time.Time()).transform.rotation
        forward = Rotation.from_quat([head.x, head.y, head.z, head.w]).apply([0., 0., 1.])
        expected_axis = np.array([.31270015, .94820117, .05597460])
        report['camera_axis_error_deg'] = math.degrees(math.acos(np.clip(forward @ (expected_axis / np.linalg.norm(expected_axis)), -1., 1.)))
        if report['camera_axis_error_deg'] > .01:
            raise RuntimeError('camera optical axis differs from the confirmed CAD lens face')
        optical = buffer.lookup_transform('world', 'head_camera_optical_frame', rclpy.time.Time()).transform
        camera = buffer.lookup_transform('world', 'head_camera_link', rclpy.time.Time()).transform
        q = optical.rotation
        direction = Rotation.from_quat([q.x, q.y, q.z, q.w]).apply([0., 0., 1.])
        request = SpawnEntity.Request()
        request.name = 'risk_mm_camera_probe'
        request.reference_frame = 'world'
        # Gazebo camera +X forward, +Y left, +Z up. An asymmetric target
        # verifies both the optical axis and the actual image orientation.
        visuals = []
        for name, y, z, rgb in [('center', 0., 0., '0 0 1'),
                                ('top', 0., .09, '1 0 0'),
                                ('right', -.09, 0., '0 1 0'),
                                ('bottom', 0., -.09, '1 1 0'),
                                ('left', .09, 0., '1 0 1')]:
            visuals.append(f'<visual name="{name}"><pose>0 {y} {z} 0 0 0</pose><geometry><box><size>.01 .065 .065</size></box></geometry><material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse><emissive>{rgb} 1</emissive></material></visual>')
        request.xml = '<sdf version="1.6"><model name="probe"><static>true</static><link name="panel">' + ''.join(visuals) + '</link></model></sdf>'
        request.initial_pose.position.x = optical.translation.x + .5 * direction[0]
        request.initial_pose.position.y = optical.translation.y + .5 * direction[1]
        request.initial_pose.position.z = optical.translation.z + .5 * direction[2]
        request.initial_pose.orientation = camera.rotation
        if not call(SpawnEntity, '/spawn_entity', request).success:
            raise RuntimeError('camera probe spawn failed')
        board = True
        image_stamp = latest['image'].header.stamp
        wait(lambda: latest['image'].header.stamp.sec > image_stamp.sec + 1, 20.)
        image = latest['image']
        rgb = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.step)[:, :image.width * 3].reshape(image.height, image.width, 3).copy()
        if image.encoding == 'bgr8':
            rgb = rgb[:, :, ::-1]
        elif image.encoding != 'rgb8':
            raise RuntimeError('unexpected camera encoding')
        PILImage.fromarray(rgb).save(folder / 'camera_axis_probe.png')
        center = rgb[image.height//2-20:image.height//2+20, image.width//2-20:image.width//2+20].mean((0, 1))
        report['camera_probe_center_rgb'] = center.tolist()
        if center[2] < 150 or center[0] > 100 or center[1] > 100:
            raise RuntimeError('physical camera view does not see the target on the declared optical axis')
        offset = round(image.width / (2 * math.tan(1.047 / 2)) * .09 / .5)
        colors = {}
        for name, dx, dy, expected in [('top', 0, -offset, [255, 0, 0]),
                                       ('right', offset, 0, [0, 255, 0]),
                                       ('bottom', 0, offset, [255, 255, 0]),
                                       ('left', -offset, 0, [255, 0, 255])]:
            x, y = image.width // 2 + dx, image.height // 2 + dy
            color = rgb[y-10:y+10, x-10:x+10].mean((0, 1))
            colors[name] = color.tolist()
            if np.max(np.abs(color - expected)) > 70:
                raise RuntimeError('camera image is flipped/misaligned at the ' + name + ' target')
        report['camera_image_orientation_rgb'] = colors
        if any(abs(s['rpy'][0]) > .15 or abs(s['rpy'][1]) > .15 or not -.02 < s['position'][2] < .05
               for stage_record in report['stages'] for s in stage_record['samples']):
            raise RuntimeError('robot ground contact / attitude became unstable')
        report.update(status='passed', manual_motion_completed=True)
    except Exception as error:
        report['reason'] = str(error)
    finally:
        if node is not None:
            try:
                command_base((0., 0., 0.))
                if board:
                    request = DeleteEntity.Request()
                    request.name = 'risk_mm_camera_probe'
                    report['camera_probe_removed'] = call(DeleteEntity, '/delete_entity', request).success
                    if not report['camera_probe_removed']:
                        raise RuntimeError('temporary camera probe was not removed')
            except Exception as error:
                report['cleanup_error'] = str(error)
                report['status'] = 'failed'
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        report['wall_s'] = time.monotonic() - started
        save()
    print(json.dumps({k: v for k, v in report.items() if k != 'stages'}, ensure_ascii=False, indent=2))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
