#!/usr/bin/env python3
"""Start one recorded isolated simulation; replace only its own prior launch group."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import shutil
import math
import subprocess
import time
from zoneinfo import ZoneInfo


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--replace', action='store_true')
    parser.add_argument('--stop', action='store_true')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--domain', type=int)
    parser.add_argument('--port', type=int)
    parser.add_argument('--restore-state', type=Path, help='restore measured base pose and staged GUI target when reloading the backend')
    args = parser.parse_args()
    ws = Path(__file__).resolve().parents[2]
    latest = ws / 'reports' / ('gazebo_headless_latest.json' if args.headless else 'gazebo_latest.json')
    domain = args.domain if args.domain is not None else (186 if args.headless else 182)
    port = args.port if args.port is not None else (11486 if args.headless else 11482)
    restored = json.loads(args.restore_state.read_text()) if args.restore_state else None
    if restored:
        if len(restored['base_pose']) != 3 or len(restored['backend_goal']) != 9 or not all(math.isfinite(v) for v in restored['base_pose']+restored['backend_goal']):
            raise ValueError('invalid saved base pose / GUI goal')
    if latest.exists():
        old = json.loads(latest.read_text())
        for extra in old.get('additional_processes', []):
            command_file = Path('/proc') / str(extra['pid']) / 'cmdline'
            if not command_file.exists() or not command_file.read_bytes():
                continue
            if not args.replace and not args.stop:
                raise RuntimeError('recorded control panel already running; use --replace')
            executable = command_file.read_bytes().split(b'\0')[0].decode()
            if executable != extra['executable'] or os.getpgid(extra['pid']) != extra['pgid']:
                raise RuntimeError('recorded control panel PID changed; refusing cleanup')
            os.killpg(extra['pgid'], signal.SIGINT)
        process = Path('/proc') / str(old['pid']) / 'cmdline'
        if process.exists() and process.read_bytes():
            if not args.replace and not args.stop:
                raise RuntimeError('recorded simulation already running; use --replace')
            command = process.read_bytes().replace(b'\0', b' ').decode()
            if 'mm_bringup' not in command or 'gazebo.launch.py' not in command or os.getpgid(old['pid']) != old['pgid']:
                raise RuntimeError('recorded PID no longer identifies our Gazebo launch; refusing cleanup')
            os.killpg(old['pgid'], signal.SIGINT)
            deadline = time.monotonic()+25.
            while process.exists() and process.read_bytes() and time.monotonic() < deadline:
                time.sleep(.2)
            if process.exists() and process.read_bytes():
                raise RuntimeError('previous own launch has not exited; inspect its log before retrying')
            old['status'] = 'stopped' if args.stop else 'replaced'
            (Path(old['report_dir']) / 'session.json').write_text(json.dumps(old, indent=2)+'\n')
        if args.stop:
            old['status'] = 'stopped'
            latest.write_text(json.dumps(old, indent=2)+'\n')
    if args.stop:
        return
    folder = ws / 'reports' / (('gazebo_headless_' if args.headless else 'gazebo_') + datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S'))
    folder.mkdir(parents=True)
    # Freeze the exact installed inputs alongside this run's success/failures.
    for package, relative in [('mm_bringup', 'config'),
                              ('rangermini_description', 'urdf/rangermini_xarm6.urdf'),
                              ('rangermini_moveit_config', 'config/rangermini_xarm6.srdf')]:
        source = ws / 'install' / package / 'share' / package / relative
        target = folder / 'inputs' / package / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        if source.is_dir():
            shutil.copytree(source, target)
        else:
            shutil.copy2(source, target)
    for name in ('s6_factory_inspection_v1.world', 's6_factory_inspection_v1.json', 'SOURCE.json'):
        source = ws / 'install/mm_bringup/share/mm_bringup/worlds' / name
        target = folder / 'inputs/mm_bringup/worlds' / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    command = [str(ws / 'scripts/in_env'), 'ros2', 'launch', 'mm_bringup', 'gazebo.launch.py', 'report_directory:='+str(folder)]
    if restored:
        command += [name+':='+str(value) for name, value in zip(('x', 'y', 'yaw'), restored['base_pose'])]
        command += ['gui_initial_goal:='+json.dumps(restored['backend_goal'])]
        shutil.copy2(args.restore_state, folder/'inputs/restart_state.json')
    if args.headless:
        command += ['gui:=false', 'rviz:=false', 'controls:=false']
    environment = dict(os.environ, ROS_DOMAIN_ID=str(domain), GAZEBO_MASTER_URI=f'http://127.0.0.1:{port}', ROS_LOG_DIR=str(folder / 'ros'))
    with (folder / 'launch.log').open('w') as log:
        process = subprocess.Popen(command, cwd=ws, env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    record = {'pid': process.pid, 'pgid': process.pid, 'domain_id': domain, 'headless': args.headless,
              'gazebo_master_uri': environment['GAZEBO_MASTER_URI'], 'command': command,
              'report_dir': str(folder), 'status': 'running', 'returncode': None,
              'initial_world_yaw_rad': restored['base_pose'][2] if restored else 3.141592653589793,
              'scope': 'C++ coupled planning, sampled mesh validation and Gazebo execution; manual longitudinal/turn controls',
              'frame_convention': 'user-confirmed front is base_control +X; base_control is rotated pi about retained CAD base_link Z; manual and planner share this calibration',
              'camera_mount': 'entire head rotated 180 degrees about flange Z; image follows physical sensor'}
    if restored:
        record['restored_base_pose']=restored['base_pose'];record['restored_gui_goal']=restored['backend_goal']
    latest.write_text(json.dumps(record, indent=2)+'\n')
    (folder / 'session.json').write_text(json.dumps(record, indent=2)+'\n')
    print(json.dumps(record, indent=2))


if __name__ == '__main__':
    main()
