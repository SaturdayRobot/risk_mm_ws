#!/usr/bin/env python3
"""Plan each configured patrol segment offline and pass its validated boundary forward."""
import argparse
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
from zoneinfo import ZoneInfo

from ament_index_python.packages import get_package_share_directory as share
import yaml


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def finite_vector(values, size, name):
    if len(values) != size or not all(math.isfinite(float(value)) for value in values):
        raise ValueError(f'{name} must contain {size} finite values')
    return [float(value) for value in values]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start-file', type=Path,
        default=Path('reports/patrol_offline_repro_20261124_01/start.json'))
    parser.add_argument('--first-goal-file', type=Path)
    parser.add_argument('--patrol-file', type=Path, help='explicit route; default is the installed factory route')
    parser.add_argument('--world-file', type=Path)
    parser.add_argument('--world-metadata-file', type=Path)
    parser.add_argument('--search-seed', type=int)
    parser.add_argument('--arm-sampling', choices=['global','reachable'])
    parser.add_argument('--arm-extension-fraction', type=float)
    parser.add_argument('--arm-soft-margin-m', type=float)
    parser.add_argument('--arm-search-budget-ms', type=float)
    parser.add_argument('--mesh-workers', type=int, choices=range(1,5))
    parser.add_argument('--optimizer-budget-ms', type=float)
    parser.add_argument('--report-dir', type=Path)
    parser.add_argument('--speeds', type=float, nargs=3, default=[.4,.6,.5],
        metavar=('BASE','YAW','ARM'))
    parser.add_argument('--domain-base', type=int, default=188)
    parser.add_argument('--timeout-s', type=float, default=180.)
    parser.add_argument('--max-time-scale', type=float, default=3.)
    args = parser.parse_args()

    ws = Path(__file__).resolve().parents[2]
    source_patrol = ws/'src/mm_bringup/config/patrol.yaml'
    source_remani = ws/'src/mm_bringup/config/remani.yaml'
    source_world = ws/'src/mm_bringup/worlds/s6_factory_inspection_v1.world'
    installed_share = Path(share('mm_bringup'))
    installed_patrol = installed_share/'config/patrol.yaml'
    installed_remani = installed_share/'config/remani.yaml'
    installed_world = installed_share/'worlds/s6_factory_inspection_v1.world'
    if sha256(source_patrol) != sha256(installed_patrol):
        raise RuntimeError('source/install patrol.yaml mismatch; rebuild before offline route validation')
    if sha256(source_remani) != sha256(installed_remani):
        raise RuntimeError('source/install remani.yaml mismatch; rebuild before offline route validation')
    selected_patrol = (args.patrol_file or installed_patrol).resolve()
    patrol = yaml.safe_load(selected_patrol.read_text())
    remani = yaml.safe_load(source_remani.read_text())['/**']['ros__parameters']
    optimizer_budget=float(patrol['optimizer_budget_ms']) if args.optimizer_budget_ms is None else args.optimizer_budget_ms
    if not math.isfinite(optimizer_budget) or optimizer_budget<=0.:
        raise ValueError('optimizer budget must be finite positive milliseconds')
    if sha256(source_world) != sha256(installed_world):
        raise RuntimeError('source/install world mismatch; rebuild before offline route validation')
    if bool(args.world_file) != bool(args.world_metadata_file):
        raise ValueError('world and metadata must be explicitly supplied together')
    selected_world = (args.world_file or installed_world).resolve()
    selected_metadata = (args.world_metadata_file or installed_world.with_suffix('.json')).resolve()
    world_hash = sha256(selected_world)
    if json.loads(selected_metadata.read_text())['world_sha256'] != world_hash:
        raise RuntimeError('world metadata SHA256 mismatch')
    if patrol['world_sha256'] != world_hash:
        raise RuntimeError('patrol world identity does not match the selected world')
    if patrol['world'] != selected_world.stem or patrol['position_frame'] != 'world' or patrol['heading_frame'] != 'base_control' or patrol['angle_unit'] != 'rad':
        raise RuntimeError('route world/frame convention mismatch')
    if not args.speeds or len(args.speeds) != 3 or any(not math.isfinite(x) or x <= 0. for x in args.speeds):
        raise ValueError('three finite positive speed limits are required')
    if not math.isfinite(args.max_time_scale) or args.max_time_scale < 1.:
        raise ValueError('max time scale must be finite and at least one')

    start = json.loads(args.start_file.read_text())
    start['state'] = finite_vector(start['state'],9,'measured origin')
    start['steer'] = finite_vector(start['steer'],4,'origin steering')
    start['wheel_position'] = finite_vector(start['wheel_position'],4,'origin wheel position')
    first_file = args.first_goal_file
    if first_file is None and args.patrol_file is None:
        first_file = ws/'reports/patrol_offline_repro_20261124_01/goal.json'
    goals = []
    manual = yaml.safe_load((installed_share/'config/manual_control.yaml').read_text())
    yaw_offset = float(manual['control_frame_yaw_in_base_rad'])
    for item in patrol['goals']:
        base = [float(value) for value in item['base']]
        arm = finite_vector(item['arm'],6,item['id']+' arm')
        base[2] = math.remainder(base[2]-yaw_offset,2.*math.pi)
        goals.append(base+arm)
    if first_file:
        first = json.loads(first_file.read_text())
        goals[0] = finite_vector(first['goal'] if isinstance(first,dict) else first,9,'first goal')
    goals.append(list(start['state']))
    names = [item['id'] for item in patrol['goals'][1:]]
    names.insert(0,patrol['goals'][0]['id'])
    names.append('return_origin')

    output = args.report_dir or ws/'reports'/('patrol_offline_route_'+
        datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f'))
    output = output if output.is_absolute() else ws/output
    output.mkdir(parents=True)
    run_script = Path(__file__).with_name('test_coupled_planning.py')
    report = {
        'status':'failed',
        'scope':'offline current REMANI port with robot-model/numerical adaptations and full sampled mesh/four-wheel validation; no actuator execution',
        'source_patrol_sha256':sha256(source_patrol),
        'installed_patrol_sha256':sha256(installed_patrol),
        'source_remani_sha256':sha256(source_remani),
        'installed_remani_sha256':sha256(installed_remani),
        'search_random_seed':args.search_seed if args.search_seed is not None else remani.get('search.random_seed'),
        'arm_sampling':args.arm_sampling or 'configured_default',
        'arm_extension_fraction':args.arm_extension_fraction if args.arm_extension_fraction is not None else remani.get('search.manipulator_extension_fraction',.3),
        'arm_soft_margin_m':args.arm_soft_margin_m if args.arm_soft_margin_m is not None else remani['optimization.safe_margin_mani'],
        'arm_search_budget_ms':args.arm_search_budget_ms if args.arm_search_budget_ms is not None else 1000.*remani['search.max_mani_search_time'],
        'world_sha256':world_hash,
        'selected_world_file':str(selected_world),
        'selected_world_metadata_file':str(selected_metadata),
        'selected_patrol_file':str(selected_patrol),
        'selected_patrol_sha256':sha256(selected_patrol),
        'optimizer_budget_ms':optimizer_budget,
        'optimizer_budget_override_ms':args.optimizer_budget_ms,
        'through_speed_m_s':float(patrol['through_speed_m_s']),
        'forward_only':bool(patrol['forward_only']),
        'minimum_turning_radius_m':float(patrol['min_turning_radius_m']),
        'external_four_wheel_validation':True,
        'speed_limits':list(args.speeds),
        'max_through_scale':args.max_time_scale,
        'origin':start['state'],
        'segments':[],
        'execution_started':False,
        'execution_completed':False,
        'continuous_safety_claim':False,
    }
    (output/'frozen_route_input.yaml').write_text(yaml.safe_dump({
        'reference_start_file':str(args.start_file),
        'reference_first_goal_file':str(first_file) if first_file else None,
        'goals_in_base_link_heading':goals,
        'goal_order':names,
        **{key:value for key,value in report.items() if key not in ('segments','origin')},
    },allow_unicode=True,sort_keys=False)+'\n')

    previous = None
    route_begin = time.monotonic()
    for index,(name,goal) in enumerate(zip(names,goals)):
        moving = index > 0
        if previous is None:
            boundary = dict(start)
        else:
            boundary = {
                'state':finite_vector(previous['terminal_position'],9,'previous terminal position'),
                'steer':finite_vector(previous['end_steer'],4,'previous terminal steering'),
                'wheel_position':finite_vector(previous['end_wheel_position'],4,'previous terminal wheel position'),
                'start_velocity':finite_vector(previous['terminal_velocity'],9,'previous terminal velocity'),
                'start_acceleration':finite_vector(previous['terminal_acceleration'],9,'previous terminal acceleration'),
                # Patrol.schedule explicitly starts each new MINCO horizon at zero jerk.
                'start_jerk':[0.]*9,
            }
        segment_dir = output/f'{index+1:02d}_{name}'
        segment_dir.mkdir()
        start_path=segment_dir/'start.json';goal_path=segment_dir/'goal.json'
        start_path.write_text(json.dumps(boundary,indent=2)+'\n')
        goal_path.write_text(json.dumps({'goal':goal,'source':name},indent=2)+'\n')
        command=[sys.executable,str(run_script),
            '--world-file',str(selected_world),'--world-metadata-file',str(selected_metadata),
            '--start-file',str(start_path),'--goal-file',str(goal_path),
            '--report-dir',str(segment_dir/'run'),'--no-update-latest',
            '--domain',str(args.domain_base+index),'--timeout-s',str(args.timeout_s),
            '--speeds',*[str(value) for value in args.speeds],
            '--optimizer-budget-ms',str(optimizer_budget),
            '--through-speed',str(patrol['through_speed_m_s']),
            '--time-profile','through','--max-time-scale',str(args.max_time_scale),
            '--capture-diagnostics','--forward-only',
            '--minimum-turning-radius',str(patrol['min_turning_radius_m']),
            '--external-four-wheel-validation']
        if args.search_seed is not None: command += ['--search-seed',str(args.search_seed)]
        if args.arm_sampling: command += ['--arm-sampling',args.arm_sampling]
        if args.arm_extension_fraction is not None: command += ['--arm-extension-fraction',str(args.arm_extension_fraction)]
        if args.arm_soft_margin_m is not None: command += ['--arm-soft-margin-m',str(args.arm_soft_margin_m)]
        if args.arm_search_budget_ms is not None: command += ['--arm-search-budget-ms',str(args.arm_search_budget_ms)]
        if args.mesh_workers is not None: command += ['--mesh-workers',str(args.mesh_workers)]
        if moving: command.append('--moving-start')
        if index==len(goals)-1: command.append('--stop-at-goal')
        begin=time.monotonic()
        completed=subprocess.run(command,env=dict(os.environ,ROS_LOCALHOST_ONLY='1'),
            stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=args.timeout_s+15.)
        elapsed=time.monotonic()-begin
        (segment_dir/'runner.log').write_text(completed.stdout)
        validation_path=segment_dir/'run'/'validation.json'
        stage={'index':index,'name':name,'moving_start':moving,'stop_at_goal':index==len(goals)-1,
               'wall_s':elapsed,'returncode':completed.returncode,
               'start':boundary,'goal':goal,'report':str(validation_path.relative_to(ws))}
        if validation_path.exists():
            result=json.loads(validation_path.read_text())
            stage.update({key:result.get(key) for key in (
                'status','reason','planning_wall_s','search_wall_s','optimization_wall_s',
                'source_check_wall_s','raw_validation_passed','raw_validation_reason',
                'planning_validation_wall_s','planning_validation_timing_scope',
                'startup_ramp_duration_s','startup_ramp_distance_m',
                'search_random_seed',
                'time_scale','maximum_local_time_scale','time_adaptation_wall_s',
                'first_violation_constraint','first_violation_time_s','first_violation_ratio',
                'best_scale_max_constraint','best_scale_max_time_s','best_scale_max_ratio',
                'validation_passed','validation_reason','validation_wall_s','validation_samples',
                'max_wheel_acceleration','max_steer_rate','initial_alignment_s','initial_steer',
                'terminal_position','terminal_velocity',
                'terminal_acceleration','end_steer','end_wheel_position')})
            if result.get('validation_passed') and result.get('terminal_position'):
                actual=result['points'][0]
                stage['start_position_error_max']=max(abs(actual['position'][j]-boundary['state'][j]) for j in range(9))
                stage['start_velocity_error_max']=max(abs(actual['velocity'][j]-boundary.get('start_velocity',[0.]*9)[j]) for j in range(9))
                stage['start_acceleration_error_max']=max(abs(actual['acceleration'][j]-boundary.get('start_acceleration',[0.]*9)[j]) for j in range(9))
                if stage['start_position_error_max']>1e-6 or stage['start_velocity_error_max']>1e-6 or stage['start_acceleration_error_max']>1e-5:
                    stage['status']='failed'
                    stage['reason']='candidate start p/v/a does not match the queued boundary'
        report['segments'].append(stage)
        if completed.returncode!=0 or stage.get('status')!='passed' or not stage.get('validation_passed'):
            report['reason']=stage.get('reason') or 'segment candidate or validation failed'
            break
        previous=result
    else:
        report['status']='all_segments_offline_validated'
        report['reason']='all configured patrol segments generated and passed sampled full-mesh/four-wheel validation'
        final=report['segments'][-1]['terminal_position']
        report['return_error_m']=math.hypot(final[0]-start['state'][0],final[1]-start['state'][1])
        report['return_yaw_error_rad']=abs(math.remainder(final[2]-start['state'][2],2.*math.pi))
        report['return_arm_error_rad']=max(abs(final[j]-start['state'][j]) for j in range(3,9))
    report['route_wall_s']=time.monotonic()-route_begin
    (output/'route_validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({key:value for key,value in report.items() if key!='segments'},ensure_ascii=False,indent=2))
    print(json.dumps(report['segments'],ensure_ascii=False,indent=2))
    return 0 if report['status']=='all_segments_offline_validated' else 1


if __name__=='__main__':
    raise SystemExit(main())
