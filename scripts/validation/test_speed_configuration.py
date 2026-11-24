#!/usr/bin/env python3
"""Reject invalid speed requests and measure camera level; no motion request."""
import argparse
import json
import math
import os
from pathlib import Path
import time

import rclpy
from risk_mm_interfaces.msg import CoupledStatus
from risk_mm_interfaces.srv import PlanCoupled
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--session', type=Path)
    args = parser.parse_args()
    ws = Path(__file__).resolve().parents[2]
    session = json.loads((args.session or ws/'reports/gazebo_latest.json').read_text())
    assert int(os.environ['ROS_DOMAIN_ID']) == session['domain_id']
    folder = Path(session['report_dir'])
    number = 1
    while (folder/f'speed_camera_validation_{number}.json').exists():
        number += 1
    report = {'status': 'failed', 'scope': 'rejected invalid requests and measured TF; no motion'}
    rclpy.init()
    node = rclpy.create_node('speed_camera_validation')
    data = {}
    subscription = node.create_subscription(CoupledStatus, '/coupled/status', lambda m: data.update(state=m), 10)
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)

    def wait(condition):
        deadline = time.monotonic()+10.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if condition():
                return
        raise TimeoutError('fresh state, camera TF or service response unavailable')

    try:
        wait(lambda: 'state' in data and buffer.can_transform('base_control', 'head_camera_optical_frame', rclpy.time.Time()))
        state = data['state']
        assert state.map_ready and state.feedback_ready
        assert state.phase not in ('PLANNING', 'ALIGNING', 'EXECUTING', 'SETTLING'), 'existing work active'
        client = node.create_client(PlanCoupled, '/coupled/plan')
        assert client.wait_for_service(timeout_sec=5.)
        report['invalid_requests'] = []
        for limits in [[1.5,.6,.5], [.4,-.1,.5], [.4,.6,1.0], [float('nan'),.6,.5]]:
            request = PlanCoupled.Request(task_id='reject_invalid_speed', model_signature=state.model_signature,
                map_revision=state.map_revision, goal=list(state.state), speed_limits=limits)
            future = client.call_async(request)
            wait(future.done)
            result = future.result()
            assert not result.accepted and 'speed' in result.reason
            report['invalid_requests'].append({'limits': [v if math.isfinite(v) else 'nan' for v in limits],
                                              'accepted': result.accepted, 'reason': result.reason})
        transform = buffer.lookup_transform('base_control', 'head_camera_optical_frame', rclpy.time.Time()).transform
        q = transform.rotation
        up = [-2*(q.x*q.y-q.w*q.z), -(1-2*(q.x*q.x+q.z*q.z)), -2*(q.y*q.z+q.w*q.x)]
        report['camera_level_error_deg'] = math.degrees(math.acos(max(-1., min(1., up[2]))))
        report['camera_up'] = up
        report['camera_forward'] = [2*(q.x*q.z+q.w*q.y), 2*(q.y*q.z-q.w*q.x), 1-2*(q.x*q.x+q.y*q.y)]
        report['camera_position'] = [transform.translation.x, transform.translation.y, transform.translation.z]
        assert report['camera_level_error_deg'] < 2., 'camera is not level in the compact posture'
        report['status'] = 'passed'
    except Exception as error:
        report['reason'] = str(error)
    finally:
        (folder/f'speed_camera_validation_{number}.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(report, ensure_ascii=False, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
