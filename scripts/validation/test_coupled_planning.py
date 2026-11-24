#!/usr/bin/env python3
"""Run the original C++ coupled backend against the actual selected world."""
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
from zoneinfo import ZoneInfo

from ament_index_python.packages import get_package_prefix, get_package_share_directory as share
import yaml
from mm_model.description import load_model
from mm_model.identity import robot_signature


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--direction', choices=['control_forward', 'model_forward'], default='control_forward')
    parser.add_argument('--preset', help='explicit task ID in goal_presets.yaml')
    parser.add_argument('--presets-file', type=Path)
    parser.add_argument('--start-file', type=Path, help='JSON measured reference: state[9], steer[4], wheel_position[4]')
    parser.add_argument('--goal-file', type=Path, help='JSON exact nine-dimensional goal; bypasses GUI presets')
    parser.add_argument('--report-dir', type=Path)
    parser.add_argument('--no-update-latest', action='store_true', help='preserve reports/coupled_probe_latest.json')
    parser.add_argument('--domain', type=int, default=185)
    parser.add_argument('--timeout-s', type=float, default=180.)
    parser.add_argument('--speeds', type=float, nargs=3, metavar=('BASE','YAW','ARM'))
    parser.add_argument('--optimizer-budget-ms', type=float)
    parser.add_argument('--search-seed', type=int, help='override the configured REMANI random seed')
    parser.add_argument('--through-speed', type=float)
    parser.add_argument('--moving-start', action='store_true', help='require start_velocity/acceleration/jerk in the start JSON')
    parser.add_argument('--stop-at-goal', action='store_true', help='plan a terminal stop instead of a through boundary')
    parser.add_argument('--time-profile', choices=['local','through','uniform'])
    parser.add_argument('--max-time-scale', type=float)
    parser.add_argument('--capture-diagnostics', action='store_true', help='save search path, MINCO seed and optimized candidate')
    parser.add_argument('--check-initial-gradient', action='store_true', help='offline finite difference check of every initial optimizer variable; see planning.log')
    parser.add_argument('--forward-only', action='store_true', help='use the patrol forward-only search configuration')
    parser.add_argument('--minimum-turning-radius', type=float, default=.55)
    parser.add_argument('--external-four-wheel-validation', action='store_true',
                         help='match patrol mode: source geometry gate plus public four-wheel hard checks')
    parser.add_argument('--warm-replans', type=int, default=0)
    args = parser.parse_args()
    if args.search_seed is not None and args.search_seed < 0:
        raise ValueError('--search-seed must be nonnegative')
    ws = Path(__file__).resolve().parents[2]
    folder = args.report_dir or ws / 'reports' / ('coupled_probe_' + datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True)
    model = Path(share('rangermini_description')) / 'urdf/rangermini_xarm6.urdf'
    bringup = Path(share('mm_bringup'))
    config = yaml.safe_load((bringup / 'config/robot.yaml').read_text())
    params = yaml.safe_load((bringup / 'config/remani.yaml').read_text())['/**']['ros__parameters']
    params.update(yaml.safe_load((bringup / 'config/coupled_execution.yaml').read_text())['/**']['ros__parameters'])
    derived = load_model(model)
    derived['mm.planning_frame_yaw_in_base_rad'] = yaml.safe_load(
        (bringup / 'config/manual_control.yaml').read_text())['control_frame_yaw_in_base_rad']
    params.update(derived)
    if args.speeds:
        params['task_speed_limits'] = args.speeds
    if args.optimizer_budget_ms is not None:
        params['optimization.solve_budget_ms'] = args.optimizer_budget_ms
    if args.search_seed is not None:
        params['search.random_seed'] = args.search_seed
    if args.through_speed is not None:
        params['probe.through_speed'] = args.through_speed
    if args.time_profile:
        params['coupled_time_scaling.enabled'] = True
        params['coupled_time_scaling.profile'] = args.time_profile
    if args.max_time_scale is not None:
        params['coupled_time_scaling.max_scale'] = args.max_time_scale
    if args.capture_diagnostics:
        params['probe.capture_diagnostics'] = True
    if args.check_initial_gradient:
        params['optimization.check_initial_gradient'] = True
    params['probe.forward_only'] = args.forward_only
    params['probe.minimum_turning_radius'] = args.minimum_turning_radius
    params['probe.external_kinematics'] = args.external_four_wheel_validation
    params['probe.warm_replans'] = args.warm_replans
    start = [-8., 0., math.pi] + config['display_arm']
    reference = json.loads(args.start_file.read_text()) if args.start_file else None
    if reference:
        start = reference['state']
        params['task_start_steer'] = reference['steer']
        params['task_start_wheel_position'] = reference['wheel_position']
        if args.moving_start:
            for name in ('velocity','acceleration','jerk'):
                values=reference.get('start_'+name,reference.get(name))
                if values is None or len(values)!=9 or not all(math.isfinite(float(value)) for value in values):
                    raise ValueError(f'--moving-start requires finite start_{name}[9] in --start-file')
                params['probe.start_'+name]=[float(value) for value in values]
    params['probe.moving_start']=bool(args.moving_start)
    params['probe.stop_at_goal']=bool(args.stop_at_goal or args.through_speed is None)
    goal = list(start)
    drive_yaw = yaml.safe_load((bringup / 'config/manual_control.yaml').read_text())['control_frame_yaw_in_base_rad'] if args.direction == 'control_forward' else 0.
    goal[0] += .6*math.cos(start[2]+drive_yaw)
    goal[1] += .6*math.sin(start[2]+drive_yaw)
    goal[3] += .04
    if args.preset and args.goal_file:
        raise ValueError('--preset and --goal-file are mutually exclusive')
    if args.preset:
        presets = yaml.safe_load((args.presets_file or bringup/'config/goal_presets.yaml').read_text())
        assert presets['position_frame'] == 'world' and presets['heading_frame'] == 'base_control' and presets['angle_unit'] == 'rad'
        assert presets['world_sha256'] == json.loads((bringup/'worlds/s6_factory_inspection_v1.json').read_text())['world_sha256']
        preset = next(item for item in presets['goals'] if item['id'] == args.preset)
        goal = list(preset['base']) + list(preset['arm'])
        goal[2] = math.remainder(goal[2]-derived['mm.planning_frame_yaw_in_base_rad'], 2*math.pi)
        # The standalone timing unit fixture is independent of task acceptance.
        # Re-scaling an already slow doorway task can exceed the duration cap.
        params['probe.run_time_scaling_regression'] = False
    if args.goal_file:
        goal_data = json.loads(args.goal_file.read_text())
        goal = goal_data['goal'] if isinstance(goal_data, dict) else goal_data
        if len(goal) != 9 or not all(math.isfinite(float(value)) for value in goal):
            raise ValueError('--goal-file must contain a finite nine-dimensional goal')
        goal = [float(value) for value in goal]
    params.update({'grid_map.external_snapshots': True,
        'world_file': str(bringup / 'worlds/s6_factory_inspection_v1.world'),
        'world_metadata_file': str(bringup / 'worlds/s6_factory_inspection_v1.json'),
        'robot_urdf_file': str(model),
        'robot_srdf_file': str(Path(share('rangermini_moveit_config')) / 'config/rangermini_xarm6.srdf'),
        'task_start': start, 'task_goal': goal,
        'model_signature': robot_signature(model, [bringup / ('config/' + name) for name in ('robot.yaml', 'remani.yaml', 'simulation_collision.yaml', 'coupled_execution.yaml', 'manual_control.yaml')] + [Path(share('rangermini_moveit_config')) / 'config/rangermini_xarm6.srdf'], model_data=derived),
        'report_file': str(folder / 'candidate.yaml')})
    executable = Path(get_package_prefix('risk_mm_runtime')) / 'lib/risk_mm_runtime/coupled_probe'
    report = {'status': 'failed', 'scope': 'original backend, complete static world fixture; no execution',
              'preset_id': args.preset, 'reference_start_file': str(args.start_file) if args.start_file else None,
              'fixed_goal_file': str(args.goal_file) if args.goal_file else None,
              'task_start': start, 'task_goal': goal,
              'optimizer_budget_ms': params.get('optimization.solve_budget_ms', 0.),
              'search_random_seed': params.get('search.random_seed'),
              'requested_speed_limits': params.get('task_speed_limits'),
              'through_speed_m_s': params.get('probe.through_speed', 0.),
              'moving_start': bool(args.moving_start),
              'stop_at_goal': bool(args.stop_at_goal or args.through_speed is None),
              'time_profile': params.get('coupled_time_scaling.profile', 'none'),
              'max_time_scale': params.get('coupled_time_scaling.max_scale', 1.),
              'capture_diagnostics': bool(args.capture_diagnostics),
              'check_initial_gradient': bool(args.check_initial_gradient),
              'forward_only': bool(args.forward_only),
              'minimum_turning_radius_m': args.minimum_turning_radius,
              'external_four_wheel_validation': bool(args.external_four_wheel_validation),
              'model_signature': params['model_signature'],
              'world_sha256': json.loads((bringup/'worlds/s6_factory_inspection_v1.json').read_text())['world_sha256']}
    try:
        with tempfile.TemporaryDirectory(prefix='risk_mm_coupled_') as temporary:
            path = Path(temporary) / 'params.yaml'
            path.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
            (folder/'params.yaml').write_bytes(path.read_bytes())
            with (folder / 'planning.log').open('w') as log:
                result = subprocess.run([str(executable), '--ros-args', '--params-file', str(path)],
                    env=dict(os.environ, ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1'),
                    stdout=log, stderr=subprocess.STDOUT, timeout=args.timeout_s)
        report['returncode'] = result.returncode
        if (folder / 'candidate.yaml').exists():
            report.update(yaml.safe_load((folder / 'candidate.yaml').read_text()))
        else:
            report['reason'] = 'C++ backend exited before producing its report; see planning.log'
        if result.returncode:
            report['status'] = 'failed'
        if report['status'] == 'passed':
            final = report['points'][-1]
            report['terminal_speed_m_s'] = math.hypot(*final['velocity'][:2])
            report['terminal_yaw_error_rad'] = abs(math.remainder(final['position'][2]-goal[2], 2*math.pi))
            expected_terminal_speed=.02 if report['stop_at_goal'] else (args.through_speed or .02)
            assert abs(report['terminal_speed_m_s']-expected_terminal_speed/report.get('endpoint_time_scale',report.get('time_scale', 1.))) < 1e-6, 'upstream terminal tangent contract lost'
            assert report['terminal_yaw_error_rad'] < 1e-6, 'terminal yaw constraint lost'
            assert abs(final['velocity'][2]) < 1e-5, 'nonzero terminal yaw rate'
            points = report['points']
            report['base_path_length_m'] = sum(math.hypot(b['position'][0]-a['position'][0], b['position'][1]-a['position'][1])
                for a, b in zip(points, points[1:]))
            report['base_heading_travel_rad'] = sum(abs(math.remainder(b['position'][2]-a['position'][2], 2*math.pi))
                for a, b in zip(points, points[1:]))
            report['max_arm_goal_change_rad'] = max(abs(goal[j]-start[j]) for j in range(3, 9))
    except Exception as error:
        report['status'] = 'failed'
        report['reason'] = str(error)
    (folder / 'validation.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    if not args.no_update_latest:
        (ws / 'reports/coupled_probe_latest.json').write_text(json.dumps({'report_dir': str(folder), 'status': report['status']}, indent=2) + '\n')
    large_fields={'points','raw_points','search_path','minco_seed_points','optimized_points','optimized_candidate_points'}
    print(json.dumps({k: v for k, v in report.items() if k not in large_fields and not k.endswith('_knots')}, ensure_ascii=False, indent=2))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
