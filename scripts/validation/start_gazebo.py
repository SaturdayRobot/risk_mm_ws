#!/usr/bin/env python3
"""Start one recorded isolated simulation; replace only its own prior launch group."""
import argparse
import hashlib
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import shutil
import math
import subprocess
import time
import re
import shlex
from zoneinfo import ZoneInfo


def check_static_consumers(ws):
    """Check the actual CMake link inputs; snapshots alone cannot prove relinking."""
    result = {}
    for package, names in [('mm_bringup', ['simulation_runtime', 'sim_control_gui', 'inspection_visualization', 'prepare_sim']),
                           ('risk_mm_runtime', ['coupled_probe', 'scene_state_probe'])]:
        directory = ws/'build'/package
        for name in names:
            built = directory/name
            installed = ws/'install'/package/'lib'/package/name
            recipe = directory/'CMakeFiles'/(name+'.dir')/'link.txt'
            if not all(p.is_file() for p in (built, installed, recipe)):
                raise RuntimeError('missing executable/link evidence; rebuild consumers: '+name)
            ids = []
            for executable in (built, installed):
                note = subprocess.check_output(['readelf', '-n', str(executable)], text=True)
                match = re.search(r'Build ID: ([0-9a-f]+)', note)
                if not match:
                    raise RuntimeError('missing ELF build identity: '+str(executable))
                ids.append(match[1])
            # Installation may rewrite RPATH, so whole-file hashes can differ.
            if ids[0] != ids[1]:
                raise RuntimeError('build/install ELF mismatch; reinstall consumer: '+name)
            archives = {}
            for token in shlex.split(recipe.read_text()):
                if not token.endswith('.a'):
                    continue
                archive = (directory/token).resolve()
                if not archive.is_relative_to(ws):
                    if not any(archive.is_relative_to(root) for root in (Path('/usr'), Path('/opt/ros'))):
                        raise RuntimeError('static link input outside current workspace/system: '+str(archive))
                    continue
                if not archive.is_file() or archive.stat().st_mtime_ns > built.stat().st_mtime_ns:
                    raise RuntimeError('stale static consumer '+name+'; rebuild after '+str(archive.relative_to(ws)))
                archives[str(archive.relative_to(ws))] = hashlib.sha256(archive.read_bytes()).hexdigest()
            if not archives:
                raise RuntimeError('no workspace static link evidence: '+name)
            result[package+'/'+name] = {'elf_build_id': ids[0], 'static_archives_sha256': archives}
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--replace', action='store_true')
    parser.add_argument('--stop', action='store_true')
    parser.add_argument('--check-build', action='store_true', help='check static consumers without starting or stopping services')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--domain', type=int)
    parser.add_argument('--port', type=int)
    parser.add_argument('--restore-state', type=Path, help='restore measured base pose and staged GUI target when reloading the backend')
    parser.add_argument('--pose', type=float, nargs=3, metavar=('X','Y','MODEL_YAW'), help='explicit simulation spawn pose')
    parser.add_argument('--world-file', type=Path)
    parser.add_argument('--world-metadata-file', type=Path)
    parser.add_argument('--patrol-file', type=Path)
    parser.add_argument('--arm-sampling', choices=['global','reachable'])
    parser.add_argument('--arm-extension-fraction', type=float)
    parser.add_argument('--arm-soft-margin-m', type=float)
    parser.add_argument('--arm-search-budget-ms', type=float)
    args = parser.parse_args()
    ws = Path(__file__).resolve().parents[2]
    if args.check_build:
        print(json.dumps({'status': 'passed', 'static_consumers': check_static_consumers(ws)}, indent=2))
        return
    linked_inputs = check_static_consumers(ws) if not args.stop else {}
    latest = ws / 'reports' / ('gazebo_headless_latest.json' if args.headless else 'gazebo_latest.json')
    domain = args.domain if args.domain is not None else (186 if args.headless else 182)
    port = args.port if args.port is not None else (11486 if args.headless else 11482)
    restored = json.loads(args.restore_state.read_text()) if args.restore_state else None
    if args.pose and (restored or not all(math.isfinite(v) for v in args.pose)):
        raise ValueError('pose must be finite and cannot be combined with restored state')
    if bool(args.world_file) != bool(args.world_metadata_file):
        raise ValueError('explicit world and metadata must be supplied together')
    if args.arm_extension_fraction is not None and (not math.isfinite(args.arm_extension_fraction) or not 0. < args.arm_extension_fraction <= 1.):
        raise ValueError('arm extension fraction must be in (0,1]')
    if args.arm_soft_margin_m is not None and (not math.isfinite(args.arm_soft_margin_m) or args.arm_soft_margin_m<0.):
        raise ValueError('arm soft margin must be finite and nonnegative')
    if args.arm_search_budget_ms is not None and (not math.isfinite(args.arm_search_budget_ms) or args.arm_search_budget_ms<=0.):
        raise ValueError('arm search budget must be finite positive milliseconds')
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
            if old.get('process_start_ticks') is not None:
                start_ticks = (process.parent/'stat').read_text().rsplit(') ',1)[1].split()[19]
                if start_ticks != old['process_start_ticks']:
                    raise RuntimeError('recorded launch PID has been reused; refusing cleanup')
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
    for source in (ws/'src/mm_bringup/config').glob('*.yaml'):
        installed=ws/'install/mm_bringup/share/mm_bringup/config'/source.name
        if not installed.is_file() or source.read_bytes()!=installed.read_bytes():
            raise RuntimeError('source/install configuration mismatch; rebuild before starting: '+source.name)
    source_launch=ws/'src/mm_bringup/launch/gazebo.launch.py'
    if source_launch.read_bytes() != (ws/'install/mm_bringup/share/mm_bringup/launch/gazebo.launch.py').read_bytes():
        raise RuntimeError('source/install launch mismatch; rebuild before starting')
    for package,relative,source in [
            ('rangermini_description','urdf/rangermini_xarm6.urdf',ws/'src/description/rangermini_description/urdf/rangermini_xarm6.urdf'),
            ('rangermini_moveit_config','config/rangermini_xarm6.srdf',ws/'src/rangermini_moveit_config/config/rangermini_xarm6.srdf')]:
        if source.read_bytes()!=(ws/'install'/package/'share'/package/relative).read_bytes():
            raise RuntimeError('source/install model mismatch: '+str(source))
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
    for name in (() if args.world_file else ('s6_factory_inspection_v1.world', 's6_factory_inspection_v1.json', 'SOURCE.json')):
        source = ws / 'install/mm_bringup/share/mm_bringup/worlds' / name
        target = folder / 'inputs/mm_bringup/worlds' / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    command = [str(ws / 'scripts/in_env'), 'ros2', 'launch', 'mm_bringup', 'gazebo.launch.py', 'report_directory:='+str(folder)]
    selected={}
    if args.world_file:
        world=args.world_file.resolve();metadata=args.world_metadata_file.resolve()
        if hashlib.sha256(world.read_bytes()).hexdigest() != json.loads(metadata.read_text())['world_sha256']:
            raise ValueError('explicit world/metadata SHA256 mismatch')
        for key,source in [('world_file',world),('world_metadata_file',metadata)]:
            target=folder/'inputs/selected_world'/source.name;target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(source,target);command.append(key+':='+str(target));selected[key]=str(target)
    if args.patrol_file:
        target=folder/'inputs/selected_patrol.yaml';shutil.copy2(args.patrol_file,target)
        command.append('patrol_file:='+str(target));selected['patrol_file']=str(target)
    if args.pose:
        command += [name+':='+str(value) for name,value in zip(('x','y','yaw'),args.pose)]
    if args.arm_sampling: command.append('arm_sampling:='+args.arm_sampling)
    if args.arm_extension_fraction is not None: command.append('arm_extension_fraction:='+str(args.arm_extension_fraction))
    if args.arm_soft_margin_m is not None: command.append('arm_soft_margin_m:='+str(args.arm_soft_margin_m))
    if args.arm_search_budget_ms is not None: command.append('arm_search_budget_ms:='+str(args.arm_search_budget_ms))
    if restored:
        command += [name+':='+str(value) for name, value in zip(('x', 'y', 'yaw'), restored['base_pose'])]
        command += ['gui_initial_goal:='+json.dumps(restored['backend_goal'])]
        shutil.copy2(args.restore_state, folder/'inputs/restart_state.json')
    if args.headless:
        command += ['gui:=false', 'rviz:=false', 'controls:=false']
    identities={'installed_executables':{},'sources':{},'static_consumers':linked_inputs}
    for package,names in [('mm_bringup',['simulation_runtime','sim_control_gui','inspection_visualization','prepare_sim']),
                          ('risk_mm_runtime',['coupled_probe','scene_state_probe'])]:
        for name in names:
            binary=ws/'install'/package/'lib'/package/name
            identities['installed_executables'][str(binary.relative_to(ws))]=hashlib.sha256(binary.read_bytes()).hexdigest()
    for relative in ['src/description/mm_config','src/planning/path_searching','src/planning/traj_opt',
                     'src/planning/remani_adapter','src/runtime/risk_mm_runtime','src/mm_bringup']:
        for source in sorted((ws/relative).rglob('*')):
            if source.is_file() and source.suffix in ('.cpp','.hpp','.h','.py','.yaml'):
                identities['sources'][str(source.relative_to(ws))]=hashlib.sha256(source.read_bytes()).hexdigest()
    identity_file=folder/'inputs/code_identity.json';identity_file.write_text(json.dumps(identities,indent=2)+'\n')
    environment = dict(os.environ, ROS_DOMAIN_ID=str(domain), GAZEBO_MASTER_URI=f'http://127.0.0.1:{port}', ROS_LOG_DIR=str(folder / 'ros'))
    with (folder / 'launch.log').open('w') as log:
        process = subprocess.Popen(command, cwd=ws, env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    record = {'pid': process.pid, 'pgid': process.pid, 'domain_id': domain, 'headless': args.headless,
              'gazebo_master_uri': environment['GAZEBO_MASTER_URI'], 'command': command,
              'report_dir': str(folder), 'status': 'running', 'returncode': None,
              'initial_world_yaw_rad': restored['base_pose'][2] if restored else (args.pose[2] if args.pose else 3.141592653589793),
              'explicit_spawn_pose':args.pose,'selected_inputs':selected,
              'arm_sampling':args.arm_sampling or 'configured_default','arm_extension_fraction':args.arm_extension_fraction,
              'arm_soft_margin_m':args.arm_soft_margin_m,'arm_search_budget_ms':args.arm_search_budget_ms,
              'code_identity_file':str(identity_file),
              'scope': 'C++ coupled planning, sampled mesh validation and Gazebo execution; manual longitudinal/turn controls',
              'frame_convention': 'user-confirmed front is base_control +X; base_control is rotated pi about retained CAD base_link Z; manual and planner share this calibration',
              'camera_mount': 'entire head rotated 180 degrees about flange Z; image follows physical sensor'}
    record['process_start_ticks']=(Path('/proc')/str(process.pid)/'stat').read_text().rsplit(') ',1)[1].split()[19]
    if restored:
        record['restored_base_pose']=restored['base_pose'];record['restored_gui_goal']=restored['backend_goal']
    latest.write_text(json.dumps(record, indent=2)+'\n')
    (folder / 'session.json').write_text(json.dumps(record, indent=2)+'\n')
    print(json.dumps(record, indent=2))


if __name__ == '__main__':
    main()
