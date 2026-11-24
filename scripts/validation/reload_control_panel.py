#!/usr/bin/env python3
"""Reload only the recorded Qt panel; preserve the simulator and staged input."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from zoneinfo import ZoneInfo

import rclpy
from std_msgs.msg import String
from risk_mm_interfaces.msg import CoupledStatus
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--session', type=Path)
    args = parser.parse_args()
    ws = Path(__file__).resolve().parents[2]
    record = args.session or ws/'reports/gazebo_latest.json'
    session = json.loads(record.read_text())
    if int(os.environ['ROS_DOMAIN_ID']) != session['domain_id']:
        raise RuntimeError('domain differs from the recorded simulation')
    rclpy.init()
    node = rclpy.create_node('operator_panel_reload')
    data = {}
    subscriptions = [node.create_subscription(kind, topic, lambda m, key=key: data.update({key: m}), 10)
        for kind, topic, key in [(String, '/simulation/gui_status', 'gui'), (CoupledStatus, '/coupled/status', 'state')]]
    try:
        deadline = time.monotonic()+10.
        while len(data) < 2 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        if len(data) != 2:
            raise RuntimeError('current panel and backend state required')
        if data['state'].phase in ('PLANNING', 'ALIGNING', 'EXECUTING', 'SETTLING'):
            raise RuntimeError('planning/execution is active; leave the current panel running')
        gui = json.loads(data['gui'].data)
        target = gui['backend_goal']
        executable = str(ws/'install/mm_bringup/lib/mm_bringup/sim_control_gui')
        groups = {session['pgid']} | {p['pgid'] for p in session.get('additional_processes', []) if p['executable'] == executable}
        candidates = []
        for path in Path('/proc').glob('[0-9]*/cmdline'):
            try:
                if path.read_bytes().split(b'\0')[0].decode() == executable and os.getpgid(int(path.parent.name)) in groups:
                    candidates.append(int(path.parent.name))
            except (ProcessLookupError, FileNotFoundError, PermissionError):
                continue
        if len(candidates) != 1:
            raise RuntimeError('exactly one identity-matching panel in the recorded process groups required')
        old_pid = candidates[0]
        folder = Path(session['report_dir'])
        stamp = datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S')
        params = folder/('gui_reload_'+stamp+'.yaml')
        world = next((s[7:] for s in session['command'] if s.startswith('world:=')), 's6_factory_inspection_v1')
        parameters = {'use_sim_time': True, 'initial_goal': target, 'world_name': world}
        if 'speed_limits' in gui:
            parameters['initial_speed_limits'] = gui['speed_limits']
        params.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}))
        os.kill(old_pid, signal.SIGTERM)
        proc = Path('/proc')/str(old_pid)/'cmdline'
        deadline = time.monotonic()+8.
        while proc.exists() and proc.read_bytes() and time.monotonic() < deadline:
            time.sleep(.1)
        if proc.exists() and proc.read_bytes():
            raise RuntimeError('recorded old panel has not exited')
        command = [str(ws/'scripts/in_env'), executable, '--ros-args', '--params-file', str(params)]
        with (folder/('gui_reload_'+stamp+'.log')).open('w') as log:
            process = subprocess.Popen(command, env=dict(os.environ, ROS_DOMAIN_ID=str(session['domain_id']),
                GAZEBO_MASTER_URI=session['gazebo_master_uri']), stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        session['additional_processes'] = [{'pid': process.pid, 'pgid': process.pid, 'executable': executable, 'command': command}]
        session.setdefault('panel_reloads', []).append({'previous_pid': old_pid, 'new_pid': process.pid, 'params': str(params)})
        session['restored_gui_goal'] = target
        record.write_text(json.dumps(session, indent=2)+'\n')
        (folder/'session.json').write_text(json.dumps(session, indent=2)+'\n')
        print(json.dumps({'gui_pid': process.pid, 'restored_goal': target, 'backend_unchanged': True}, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
