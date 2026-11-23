#!/usr/bin/env python3
"""Read-only checks of an already running RViz/model GUI; never execute motion."""
import argparse
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def main():
    ws = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report-dir', type=Path)
    args = parser.parse_args()
    folder = args.report_dir or Path(json.loads((ws / 'reports/gui_latest.json').read_text())['report_dir'])
    session = json.loads((folder / 'session.json').read_text())
    report = {'status': 'failed', 'scope': session['scope'],
              'planning_candidate': None, 'validation_passed': None,
              'execution_started': None, 'execution_completed': None}
    node = None
    started = time.monotonic()
    try:
        if int(os.environ['ROS_DOMAIN_ID']) != session['domain_id']:
            raise RuntimeError('ROS domain differs from the GUI session')
        rclpy.init()
        node = rclpy.create_node('gui_validation')
        observed = {'samples': 0}

        def state(message):
            observed.update(state=message, received=time.monotonic(), samples=observed['samples'] + 1)

        node.create_subscription(JointState, '/joint_states', state, 10)
        node.create_subscription(String, '/robot_description',
                                 lambda message: observed.update(description=message.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        buffer = Buffer()
        listener = TransformListener(buffer, node)
        description = (Path(get_package_share_directory('rangermini_description')) /
                       'urdf/rangermini_xarm6.urdf').read_text()
        root = ET.fromstring(description)
        links = [link.get('name') for link in root.findall('link')]
        joints = {joint.get('name'): joint for joint in root.findall('joint')
                  if joint.get('type') != 'fixed'}
        while time.monotonic() - started < 12.:
            rclpy.spin_once(node, timeout_sec=.1)
            if (observed['samples'] >= 3 and 'description' in observed and
                    all(buffer.can_transform('base_link', link, rclpy.time.Time()) for link in links)):
                break
        if observed.get('description') != description:
            raise RuntimeError('live robot description missing or differs from this workspace')
        message = observed.get('state')
        if message is None or observed['samples'] < 3:
            raise RuntimeError('insufficient joint-state samples')
        if len(message.name) != len(set(message.name)) or set(message.name) != set(joints):
            raise RuntimeError('joint-state names differ from the current model')
        if len(message.position) != len(message.name):
            raise RuntimeError('incomplete joint positions')
        age = (node.get_clock().now().nanoseconds -
               (message.header.stamp.sec * 10**9 + message.header.stamp.nanosec)) / 1e9
        if not 0. <= age <= 2. or time.monotonic() - observed['received'] > 2.:
            raise RuntimeError('joint-state timestamp or receipt is stale/invalid')
        for name, value in zip(message.name, message.position):
            limit = joints[name].find('limit')
            if not math.isfinite(value):
                raise RuntimeError('nonfinite joint position: ' + name)
            if joints[name].get('type') != 'continuous' and not (
                    float(limit.get('lower')) <= value <= float(limit.get('upper'))):
                raise RuntimeError('joint position outside model limits: ' + name)
        missing = [link for link in links if not buffer.can_transform('base_link', link, rclpy.time.Time())]
        if missing:
            raise RuntimeError('missing transforms: ' + ', '.join(missing))
        tree = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True, timeout=5.)
        (folder / 'windows.txt').write_text(tree)
        windows = {}
        for name in ['rviz2', 'joint_state_publisher_gui']:
            row = next(row for row in tree.splitlines() if f'("{name}" "{name}")' in row)
            window = re.search(r'0x[0-9a-fA-F]+', row).group()
            details = subprocess.check_output(['xwininfo', '-id', window], text=True, timeout=5.)
            if 'Map State: IsViewable' not in details:
                raise RuntimeError('window not viewable: ' + name)
            windows[name] = window
        report.update(status='passed', domain_id=session['domain_id'], model_id=root.get('name'),
                      joint_count=len(joints), link_count=len(links), joint_samples=observed['samples'],
                      joint_state_age_ros_s=age, windows=windows,
                      joint_positions=dict(zip(message.name, message.position)),
                      note='Synthetic GUI state and TF only; no collision or trajectory safety validation.')
    except Exception as error:
        report['reason'] = str(error)
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    report['check_wall_s'] = time.monotonic() - started
    (folder / 'validation.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
