#!/usr/bin/env python3
"""Drive actual Qt controls with native X11 events; observe ROS, never publish commands."""
import argparse
import ctypes as C
from datetime import datetime
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time
from zoneinfo import ZoneInfo
import yaml
from ament_index_python.packages import get_package_share_directory as share

import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from risk_mm_interfaces.msg import CoupledStatus
from tf2_ros import Buffer, TransformListener


class ButtonEvent(C.Structure):
    _fields_ = [('type', C.c_int), ('serial', C.c_ulong), ('send_event', C.c_int),
                ('display', C.c_void_p), ('window', C.c_ulong), ('root', C.c_ulong),
                ('subwindow', C.c_ulong), ('time', C.c_ulong), ('x', C.c_int), ('y', C.c_int),
                ('x_root', C.c_int), ('y_root', C.c_int), ('state', C.c_uint),
                ('button', C.c_uint), ('same_screen', C.c_int)]


class KeyEvent(C.Structure):
    _fields_ = ButtonEvent._fields_[:-2] + [('keycode', C.c_uint), ('same_screen', C.c_int)]


class Event(C.Union):
    _fields_ = [('button', ButtonEvent), ('key', KeyEvent), ('pad', C.c_long*24)]


class NativePanel:
    def __init__(self, session):
        self.x = C.CDLL('libX11.so.6')
        self.x.XOpenDisplay.argtypes, self.x.XOpenDisplay.restype = [C.c_char_p], C.c_void_p
        self.x.XDefaultRootWindow.argtypes, self.x.XDefaultRootWindow.restype = [C.c_void_p], C.c_ulong
        self.x.XSendEvent.argtypes = [C.c_void_p, C.c_ulong, C.c_int, C.c_long, C.c_void_p]
        self.x.XTranslateCoordinates.argtypes = [C.c_void_p, C.c_ulong, C.c_ulong, C.c_int, C.c_int, C.POINTER(C.c_int), C.POINTER(C.c_int), C.POINTER(C.c_ulong)]
        self.x.XFlush.argtypes = [C.c_void_p]
        self.x.XKeysymToKeycode.argtypes, self.x.XKeysymToKeycode.restype = [C.c_void_p, C.c_ulong], C.c_uint
        self.x.XStringToKeysym.argtypes, self.x.XStringToKeysym.restype = [C.c_char_p], C.c_ulong
        self.display = self.x.XOpenDisplay(None)
        if not self.display:
            raise RuntimeError('host X display unavailable')
        tree = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
        windows = [s for s in tree.splitlines() if '"Rangermini + xArm6 | C++ Gazebo control":' in s]
        owned = {p['pid'] for p in session.get('additional_processes', [])}
        self.window = None
        for row in windows:
            window = int(re.search(r'0x[0-9a-fA-F]+', row).group(), 16)
            prop = subprocess.check_output(['xprop', '-id', hex(window), '_NET_WM_PID'], text=True)
            pid = int(prop.rsplit('=', 1)[1])
            if pid in owned or os.getpgid(pid) == session['pgid']:
                self.window = window
                break
        if self.window is None:
            raise RuntimeError('recorded session control panel not found')
        self.root = self.x.XDefaultRootWindow(self.display)
        self.held = None

    def mouse(self, geometry, press):
        xr, yr, child = C.c_int(), C.c_int(), C.c_ulong()
        self.x.XTranslateCoordinates(self.display, self.window, self.root, 0, 0, C.byref(xr), C.byref(yr), C.byref(child))
        event = Event(); button = event.button
        button.type = 4 if press else 5
        button.send_event, button.display, button.window, button.root = 1, self.display, self.window, self.root
        button.x = geometry['x'] + geometry['width']//2
        button.y = geometry['y'] + geometry['height']//2
        button.x_root, button.y_root = xr.value+button.x, yr.value+button.y
        button.state, button.button, button.same_screen = 0 if press else 256, 1, 1
        if not self.x.XSendEvent(self.display, self.window, 0, 4 if press else 8, C.byref(event)):
            raise RuntimeError('X11 mouse event rejected')
        self.x.XFlush(self.display)
        self.held = geometry if press else None

    def click(self, geometry):
        if not geometry.get('enabled', True):
            raise RuntimeError('GUI control is disabled')
        self.mouse(geometry, True); self.mouse(geometry, False)

    def key(self, name, state=0):
        code = self.x.XKeysymToKeycode(self.display, self.x.XStringToKeysym(name.encode()))
        for kind, mask in [(2, 1), (3, 2)]:
            event = Event(); key = event.key
            key.type, key.send_event, key.display, key.window, key.root = kind, 1, self.display, self.window, self.root
            key.state, key.keycode, key.same_screen = state, code, 1
            self.x.XSendEvent(self.display, self.window, 0, mask, C.byref(event))
        self.x.XFlush(self.display)

    def edit(self, geometry, value):
        self.click(geometry); self.key('a', 4)
        for character in f'{value:.4f}':
            self.key({'-': 'minus', '.': 'period'}.get(character, character))
        self.key('Return')

    def select(self, geometry, row):
        self.click(geometry)
        time.sleep(.15)
        tree = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
        popup = next(line for line in tree.splitlines() if '"sim_control_gui": ("sim_control_gui" "sim_control_gui")' in line)
        window = int(re.search(r'0x[0-9a-fA-F]+', popup).group(),16)
        original = self.window
        try:
            # Qt's popup is a separate native window. Route keys to its grab,
            # rather than assuming they are delivered through the main panel.
            self.window = window
            self.key('Home')
            for _ in range(row):
                self.key('Down')
            self.key('Return')
        finally:
            self.window = original


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--session', type=Path)
    parser.add_argument('--scenario', choices=['drive', 'rejection', 'plan', 'execute', 'presets', 'speed_invalidation', 'patrol'], default='drive')
    parser.add_argument('--hold-s', type=float, default=1.8)
    parser.add_argument('--preset', help='plan the selected configured preset instead of the 0.6 m case')
    parser.add_argument('--speeds', type=float, nargs=3)
    args = parser.parse_args()
    ws = Path(__file__).resolve().parents[2]
    session = json.loads((args.session or ws/'reports/gazebo_latest.json').read_text())
    assert int(os.environ['ROS_DOMAIN_ID']) == session['domain_id']
    folder = Path(session['report_dir'])/'gui_checks'/datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S')
    folder.mkdir(parents=True)
    report = {'status': 'failed', 'scenario': args.scenario, 'input_method': 'native Qt mouse/key event path; no ROS command/service publisher', 'requests': [], 'phases': []}
    started = time.monotonic(); data = {}; commands = []; odom_samples = []; panel = None
    rclpy.init(); node = rclpy.create_node('real_gui_acceptance', parameter_overrides=[Parameter('use_sim_time', value=True)])
    subscriptions = []

    def gui(message):
        value = json.loads(message.data); data['gui'] = value
        if not report['requests'] or report['requests'][-1]['text'] != value['request']:
            report['requests'].append({'wall_s': time.monotonic()-started, 'text': value['request']})

    def coupled(message):
        data['coupled'] = message
        entry = {'phase': message.phase, 'plan_id': message.plan_id, 'reason': message.reason}
        if not report['phases'] or any(report['phases'][-1][k] != entry[k] for k in entry):
            report['phases'].append(dict(entry, wall_s=time.monotonic()-started))

    subscriptions.append(node.create_subscription(String, '/simulation/gui_status', gui, 10))
    subscriptions.append(node.create_subscription(CoupledStatus, '/coupled/status', coupled, 10))
    for kind, topic, key in [(Odometry, '/ground_truth/odom', 'odom'), (JointState, '/joint_states', 'joints'), (String, '/simulation/control_status', 'control')]:
        subscriptions.append(node.create_subscription(kind, topic, lambda m, key=key: data.update({key: m}), 10))
    subscriptions.append(node.create_subscription(Odometry, '/ground_truth/odom', lambda m: odom_samples.append(
        math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y)), 10))
    subscriptions.append(node.create_subscription(TwistStamped, '/simulation/cmd_vel', lambda m: commands.append({'x': m.twist.linear.x, 'y': m.twist.linear.y, 'wz': m.twist.angular.z}), 10))
    buffer = Buffer(node=node); listener = TransformListener(buffer, node)

    def wait(condition, timeout=15.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
            if condition():
                return
        raise TimeoutError('GUI condition timed out: '+str(data.get('gui', {}).get('request')))

    def spin(duration):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)

    def click(name):
        panel.click(data['gui']['buttons'][name]); spin(.15)

    def pose():
        p = data['odom'].pose.pose.position; q = data['odom'].pose.pose.orientation
        return [p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]

    try:
        wait(lambda: all(k in data for k in ('gui', 'odom', 'coupled', 'control')))
        panel = NativePanel(session)
        report['window'] = hex(panel.window)
        assert data['coupled'].phase not in ('EXECUTING', 'ALIGNING', 'SETTLING'), 'existing execution active; do not interfere'
        if args.speeds:
            for key,value in zip(('base_speed','yaw_speed','arm_speed'),args.speeds):
                panel.edit(data['gui']['inputs'][key],value);spin(.25)
            wait(lambda: all(abs(a-b)<.005 for a,b in zip(data['gui']['speed_limits'],args.speeds)))
            report['requested_speed_limits']=args.speeds
        if args.scenario != 'presets':
            click('stop')
        wait(lambda: json.loads(data['control'].data)['owner_plan'] == '' and data['gui']['buttons']['plan']['enabled'])
        if args.scenario == 'drive':
            report['stages'] = []
            for button, sign in [('forward', 1), ('back', -1)]:
                before = pose(); offset = data['gui']['drive_yaw_in_base_rad']; heading = before[2]+offset
                transform = buffer.lookup_transform('world', 'head_camera_optical_frame', rclpy.time.Time()).transform.rotation
                optical = [2*(transform.x*transform.z+transform.w*transform.y), 2*(transform.y*transform.z-transform.w*transform.x)]
                index = len(commands); feedback_index = len(odom_samples); geometry = data['gui']['buttons'][button]
                assert geometry['enabled']
                panel.mouse(geometry, True); spin(args.hold_s)
                joints = data['joints']; wheels = {n: v for n, v in zip(joints.name, joints.velocity) if 'wheel_' in n}
                panel.mouse(geometry, False); spin(.6); after = pose()
                dx, dy = after[0]-before[0], after[1]-before[1]
                projection = dx*math.cos(heading)+dy*math.sin(heading)
                camera_projection = dx*optical[0]+dy*optical[1]
                stage = {'button': button, 'before': before, 'after': after, 'front_projection_m': projection,
                         'camera_forward_projection_m': camera_projection, 'commands': commands[index:], 'wheel_velocities': wheels,
                         'measured_max_speed_m_s':max(odom_samples[feedback_index:],default=0.)}
                report['stages'].append(stage)
                assert sign*projection > 0., f'{button} moved opposite to calibrated front'
                assert sign*projection > .1, f'{button} displacement too small: {projection:.4f} m; inspect competing inputs/duration'
                assert sign*camera_projection > .09, f'{button} is opposite to the user-confirmed camera/front'
        elif args.scenario == 'presets':
            config = yaml.safe_load((Path(share('mm_bringup'))/'config/goal_presets.yaml').read_text())
            report['loaded_goals'] = []
            initial_pose = pose(); phase = data['coupled'].phase; command_index = len(commands)
            def select_preset(index, preset):
                panel.select(data['gui']['inputs']['preset_selector'],index+1)
                wait(lambda: data['gui']['preset_selection'] == preset['id'], 5.)
                click('load_preset')
                wait(lambda: data['gui']['loaded_preset'] == preset['id'])
                expected = preset['base'] + preset['arm']
                expected[2] = math.remainder(expected[2]-data['gui']['drive_yaw_in_base_rad'], 2*math.pi)
                actual = data['gui']['backend_goal']
                for j in range(9):
                    error = math.remainder(actual[j]-expected[j], 2*math.pi) if j == 2 else actual[j]-expected[j]
                    assert abs(error) < 5.1e-5, f'{preset["id"]}: wrong goal component {j}'
                assert data['gui']['goal_dirty'] and not data['gui']['accepted_plan_id']
                assert not data['gui']['buttons']['execute']['enabled']
                return {'id': preset['id'], 'backend_goal': actual}
            for index, preset in enumerate(config['goals']):
                report['loaded_goals'].append(select_preset(index, preset))
            select_preset(0, config['goals'][0]); spin(.5)
            report['selection_drift_m'] = math.dist(initial_pose[:2], pose()[:2])
            assert report['selection_drift_m'] < .005, 'loading a preset moved the robot'
            assert data['coupled'].phase == phase, 'loading a preset changed backend execution state'
            assert all(max(abs(command[k]) for k in ('x', 'y', 'wz')) < 1e-9 for command in commands[command_index:])
        elif args.scenario == 'patrol':
            previous = data['coupled'].plan_id
            click('patrol')
            wait(lambda: data['gui']['accepted_plan_id'] not in ('', previous) or
                 data['gui']['request'].startswith('Plan rejected:'), 20.)
            mission = data['gui']['accepted_plan_id']
            assert mission, data['gui']['request']
            report['gui_service_accepted_plan_id'] = mission
            wait(lambda: data['coupled'].plan_id == mission and
                 (data['coupled'].execution_started or data['coupled'].phase == 'HOLD'), 20.)
            assert data['coupled'].execution_started, data['coupled'].reason
            wait(lambda: data['coupled'].phase in ('COMPLETED', 'HOLD'), 120.)
            result_file = Path(session['report_dir'])/'patrol'/mission/'mission.yaml'
            result = yaml.safe_load(result_file.read_text())
            report.update(mission=result, mission_report=str(result_file))
            state = data['coupled']
            report.update(execution_started=state.execution_started, execution_completed=state.execution_completed,
                          execution_success=state.execution_success)
            assert result['success'] and result['visited'] == result['inspection_points'], result['reason']
            assert state.execution_completed and state.execution_success
            assert result['return_error_m'] < .04 and result['planning_stops'] == 0
            assert all(v['measured_speed_m_s'] > .02 and v['arm_error_rad'] < .04 for v in result['visits'])
            assert all(v['requested_while_moving'] for v in result['segments'][1:])
            assert max(v['planning_ms'] for v in result['segments']) < 500.
            wait(lambda: data['gui']['buttons']['patrol']['enabled'])
        elif args.scenario == 'rejection':
            original = data['gui']['backend_goal'][0]
            panel.edit(data['gui']['inputs']['world_x'], 11.)
            wait(lambda: abs(data['gui']['backend_goal'][0]-11.) < 1e-6)
            click('plan')
            wait(lambda: data['gui']['request'].startswith('Plan rejected:'), 30.)
            reason = data['gui']['request']; spin(2.)
            assert data['gui']['request'] == reason, 'request rejection was overwritten by periodic backend status'
            assert not data['gui']['buttons']['execute']['enabled']
            report['persistent_rejection'] = reason
            panel.edit(data['gui']['inputs']['world_x'], original)
            wait(lambda: abs(data['gui']['backend_goal'][0]-original) < 1e-5)
        else:
            initial_pose = pose()
            front_heading = initial_pose[2]+data['gui']['drive_yaw_in_base_rad']
            if args.preset:
                config=yaml.safe_load((Path(share('mm_bringup'))/'config/goal_presets.yaml').read_text())
                index=next(i for i,g in enumerate(config['goals']) if g['id']==args.preset)
                panel.select(data['gui']['inputs']['preset_selector'],index+1)
                wait(lambda:data['gui']['preset_selection']==args.preset,5.)
                click('load_preset');wait(lambda:data['gui']['loaded_preset']==args.preset)
                report['preset_id']=args.preset
            else:
                click('ahead'); spin(.5)
                arm_target = data['gui']['backend_goal'][3]+.04
                panel.edit(data['gui']['inputs']['joint1'], arm_target)
                wait(lambda: abs(data['gui']['backend_goal'][3]-arm_target) < 1e-4)
            requested_goal = list(data['gui']['backend_goal']); previous = data['coupled'].plan_id
            click('plan')
            wait(lambda: data['gui']['accepted_plan_id'] not in ('', previous) or data['gui']['request'].startswith('Plan rejected:'), 30.)
            assert data['gui']['accepted_plan_id'], data['gui']['request']
            plan_id = data['gui']['accepted_plan_id']; report['gui_service_accepted_plan_id'] = plan_id
            planning_started=time.monotonic()
            wait(lambda: data['coupled'].plan_id == plan_id and data['coupled'].phase in ('READY', 'REJECTED', 'HOLD'), 30.)
            state = data['coupled']; report.update(candidate_generated=state.candidate_generated, validation_passed=state.validation_passed, planner_reason=state.reason)
            report['planning_wait_wall_s']=time.monotonic()-planning_started
            report['planning_wall_s']=state.planning_wall_s;report['validation_wall_s']=state.validation_wall_s;report['duration_s']=state.duration_s
            assert state.ready_to_execute, state.reason
            wait(lambda: data['gui']['buttons']['execute']['enabled'])
            report['validated_goal'] = requested_goal
            if args.scenario == 'speed_invalidation':
                previous_speed=data['gui']['speed_limits'][0]
                changed=previous_speed-.05 if previous_speed>.4 else previous_speed+.05
                panel.edit(data['gui']['inputs']['base_speed'],changed)
                wait(lambda: data['gui']['goal_dirty'] and not data['gui']['accepted_plan_id'] and
                     not data['gui']['buttons']['execute']['enabled'])
                report['speed_edit_invalidated_gui_execution']=True
                assert not data['coupled'].execution_started
                click('stop')
                wait(lambda: json.loads(data['control'].data)['owner_plan']=='')
                panel.edit(data['gui']['inputs']['base_speed'],previous_speed)
            if args.scenario == 'execute':
                click('execute')
                wait(lambda: data['coupled'].execution_started or data['coupled'].phase == 'HOLD', 15.)
                assert data['coupled'].execution_started, data['coupled'].reason
                wait(lambda: data['coupled'].phase in ('COMPLETED', 'HOLD'), state.duration_s+35.)
                state = data['coupled']; report.update(execution_started=state.execution_started, execution_completed=state.execution_completed, execution_success=state.execution_success, execution_reason=state.reason)
                assert state.execution_completed and state.execution_success, state.reason
                report['endpoint_error_m'] = math.hypot(state.state[0]-requested_goal[0], state.state[1]-requested_goal[1])
                report['arm_endpoint_error_rad'] = max(abs(state.state[j]-requested_goal[j]) for j in range(3, 9))
                report['front_projection_m'] = ((state.state[0]-initial_pose[0])*math.cos(front_heading)+
                                                (state.state[1]-initial_pose[1])*math.sin(front_heading))
                assert report['endpoint_error_m'] < .04 and report['arm_endpoint_error_rad'] < .03
                if not args.preset:
                    assert report['front_projection_m'] > .55, 'execution did not advance toward the user-confirmed front'
        report['status'] = 'passed'
    except Exception as error:
        report['reason'] = str(error)
    finally:
        if panel and panel.held is not None:
            panel.mouse(panel.held, False)
        report['wall_s'] = time.monotonic()-started
        (folder/'validation.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps({k: v for k, v in report.items() if k != 'stages'}, ensure_ascii=False, indent=2))
        node.destroy_node(); rclpy.shutdown()
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
