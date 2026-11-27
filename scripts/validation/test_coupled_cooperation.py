#!/usr/bin/env python3
"""Frozen cooperation fixtures using the existing C++ planner and full mesh checker."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import statistics
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory as share, get_package_prefix
from mm_model.description import load_model
import yaml


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def scene_fixture(case, bounds, folder):
    cx,cy=case['center'];width=case['passage_width_m']
    boxes=[('side_south',[cx,cy-width/2.-.1,1.25],[8.,.2,2.5]),
           ('side_north',[cx,cy+width/2.+.1,1.25],[8.,.2,2.5])]
    if 'obstacle_box' in case:
        box=case['obstacle_box'];offset=box['center_offset']
        boxes.append(('arm_obstacle',[cx+offset[0],cy+offset[1],offset[2]],box['size']))
    else:
        clearance=case['clearance_m']
        boxes.append(('overhead_beam',[cx,cy,(clearance+2.)/2.],[case['beam_length_m'],width+.2,2.-clearance]))
    root=ET.Element('sdf',version='1.6');world=ET.SubElement(root,'world',name=case['id'])
    ET.SubElement(world,'gravity').text='0 0 -9.81'
    physics=ET.SubElement(world,'physics',name='ode',type='ode')
    ET.SubElement(physics,'max_step_size').text='0.001';ET.SubElement(physics,'real_time_update_rate').text='1000'
    floor=ET.SubElement(world,'model',name='ground_plane');ET.SubElement(floor,'static').text='true'
    link=ET.SubElement(floor,'link',name='link')
    for tag in ('collision','visual'):
        shape=ET.SubElement(link,tag,name=tag);plane=ET.SubElement(ET.SubElement(shape,'geometry'),'plane')
        ET.SubElement(plane,'normal').text='0 0 1';ET.SubElement(plane,'size').text='100 100'
    for name,center,size in boxes:
        model=ET.SubElement(world,'model',name=name);ET.SubElement(model,'static').text='true'
        ET.SubElement(model,'pose').text=' '.join(map(str,center))+' 0 0 0'
        link=ET.SubElement(model,'link',name='body')
        for tag in ('collision','visual'):
            shape=ET.SubElement(link,tag,name=tag);box=ET.SubElement(ET.SubElement(shape,'geometry'),'box')
            ET.SubElement(box,'size').text=' '.join(map(str,size))
    ET.indent(root)
    path=folder/(case['id']+'.world');ET.ElementTree(root).write(path,encoding='unicode',xml_declaration=True)
    metadata=folder/(case['id']+'.json')
    metadata.write_text(json.dumps({'version':case['id']+'_v1','world_sha256':sha(path),'map_bounds':bounds,
        'scope':case['split'],'collision_boxes':[{'name':name,'center':center,'size':size} for name,center,size in boxes]},indent=2)+'\n')
    return path,metadata


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report-dir',type=Path,required=True)
    parser.add_argument('--fixtures-file',type=Path,default=Path(__file__).with_name('fixtures')/'coupled_cooperation.yaml')
    parser.add_argument('--case',action='append',dest='cases')
    parser.add_argument('--seed',type=int,nargs='+',default=[42])
    parser.add_argument('--arm-sampling',choices=['global','reachable'],required=True)
    parser.add_argument('--arm-extension-fraction',type=float,required=True)
    parser.add_argument('--arm-search-budget-ms',type=float,default=80.)
    parser.add_argument('--arm-soft-margin-m',type=float,default=.02)
    parser.add_argument('--optimizer-budget-ms',type=float,default=100.)
    parser.add_argument('--geometry-only',action='store_true')
    parser.add_argument('--domain',type=int,default=220)
    args=parser.parse_args()
    ws=Path(__file__).resolve().parents[2];output=args.report_dir.resolve();output.mkdir(parents=True)
    spec_file=args.fixtures_file.resolve();spec=yaml.safe_load(spec_file.read_text())
    cases=[c for c in spec['cases'] if not args.cases or c['id'] in args.cases]
    if not cases or args.cases and set(args.cases)-{c['id'] for c in cases}:raise ValueError('unknown case')
    bringup=Path(share('mm_bringup'));robot=yaml.safe_load((bringup/'config/robot.yaml').read_text())
    hard=yaml.safe_load((bringup/'config/coupled_execution.yaml').read_text())['/**']['ros__parameters']
    geometry={'robot_urdf_file':str(Path(share('rangermini_description'))/'urdf/rangermini_xarm6.urdf'),
        'robot_srdf_file':str(Path(share('rangermini_moveit_config'))/'config/rangermini_xarm6.srdf'),
        'collision_padding':hard['validation.collision_padding']}
    model=load_model(Path(geometry['robot_urdf_file']))
    geometry.update(wheel_positions=model['controller.wheel_positions'],wheel_radius=.1)
    probe=Path(get_package_prefix('risk_mm_runtime'))/'lib/risk_mm_runtime/scene_state_probe'
    environment=dict(os.environ,ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='1')
    report={'status':'failed','scope':spec['scope'],'fixture_sha256':sha(spec_file),'arm_sampling':args.arm_sampling,
        'arm_extension_fraction':args.arm_extension_fraction,'arm_search_budget_ms':args.arm_search_budget_ms,
        'arm_soft_margin_m':args.arm_soft_margin_m,'optimizer_budget_ms':args.optimizer_budget_ms,'cases':[],
        'execution_started':False,'execution_completed':False,'continuous_safety_claim':False}
    report['measurement_host']={'machine':os.uname().machine,'available_cpus':len(os.sched_getaffinity(0)),
        'cpu_quota':Path('/sys/fs/cgroup/cpu.max').read_text().strip(),'initial_load_average':list(os.getloadavg())}
    (output/'frozen_suite.yaml').write_text(yaml.safe_dump({'selected_cases':cases,'seeds':args.seed,**{k:v for k,v in report.items() if k!='cases'}},sort_keys=False))
    def check(folder,name,world,metadata,states):
        fixture={**geometry,'world_file':str(world),'world_metadata_file':str(metadata),'states':states}
        source=folder/(name+'_input.yaml');target=folder/(name+'_report.yaml');source.write_text(yaml.safe_dump(fixture,sort_keys=False))
        with (folder/(name+'.log')).open('w') as log:
            subprocess.run([str(probe),str(source),str(target)],env=environment,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=60.)
        return yaml.safe_load(target.read_text())
    for case in cases:
        folder=output/case['id'];folder.mkdir();world,metadata=scene_fixture(case,spec['map_bounds'],folder)
        arm=[float(v) for v in robot['display_arm']];arm[0]=float(case['initial_joint1_rad'])
        cx,cy=case['center']
        start=[cx+case['start_offset'][0],cy+case['start_offset'][1],math.remainder(case['start_control_yaw']-math.pi,2*math.pi)]+arm
        goal=[cx+case['goal_offset'][0],cy+case['goal_offset'][1],math.remainder(case['goal_control_yaw']-math.pi,2*math.pi)]+arm
        states=[{'id':'start','position':start},{'id':'goal','position':goal},
                {'id':'fixed_arm_at_obstacle','position':[cx,cy,math.pi]+arm}]
        preflight=check(folder,'preflight',world,metadata,states)
        row={'id':case['id'],'split':case['split'],'world_sha256':sha(world),'preflight_report':str(folder/'preflight_report.yaml'),'attempts':[]}
        flags={s['id']:s['valid'] for s in preflight['states']}
        row['geometric_need_confirmed']=flags['start'] and flags['goal'] and not flags['fixed_arm_at_obstacle']
        if not row['geometric_need_confirmed']:
            row['reason']='fixture start/goal invalid or fixed-arm collision not established';report['cases'].append(row);continue
        (folder/'start.json').write_text(json.dumps({'state':start,'steer':[0.]*4,'wheel_position':[0.]*4,'source':'explicit synthetic fixture, not measured robot state'},indent=2)+'\n')
        (folder/'goal.json').write_text(json.dumps({'goal':goal},indent=2)+'\n')
        if not args.geometry_only:
            for index,seed in enumerate(args.seed):
                run=folder/f'run_{index+1}_seed{seed}'
                command=[sys.executable,str(Path(__file__).with_name('test_coupled_planning.py')),
                    '--world-file',str(world),'--world-metadata-file',str(metadata),'--start-file',str(folder/'start.json'),
                    '--goal-file',str(folder/'goal.json'),'--report-dir',str(run),'--no-update-latest','--domain',str(args.domain),
                    '--speeds','.4','.6','.5','--optimizer-budget-ms',str(args.optimizer_budget_ms),'--search-seed',str(seed),'--time-profile','through',
                    '--max-time-scale','3','--forward-only','--minimum-turning-radius','.4','--external-four-wheel-validation',
                    '--arm-sampling',args.arm_sampling,'--arm-extension-fraction',str(args.arm_extension_fraction),
                    '--arm-search-budget-ms',str(args.arm_search_budget_ms),'--arm-soft-margin-m',str(args.arm_soft_margin_m)]
                with (folder/(run.name+'.log')).open('w') as log:completed=subprocess.run(command,env=environment,stdout=log,stderr=subprocess.STDOUT,timeout=200.)
                data=json.loads((run/'validation.json').read_text());attempt={'seed':seed,'report':str(run/'validation.json'),
                    'status':data['status'],'reason':data.get('reason'),'returncode':completed.returncode,
                    'candidate_generated':bool(data.get('candidate_generated')),
                    'validation_passed':bool(data.get('validation_passed')),
                    'planning_validation_wall_s':data.get('planning_validation_wall_s'),'planning_wall_s':data.get('planning_wall_s')}
                for field in ('search_wall_s','optimization_wall_s','source_check_wall_s',
                              'time_adaptation_wall_s','validation_wall_s','cost_evaluations',
                              'max_wheel_acceleration','max_steer_rate','mesh_workers','validation_samples',
                              'source_validation'):
                    attempt[field]=data.get(field)
                if data.get('validation_passed'):
                    points=data['points'];movement=[max(abs(p['position'][j]-start[j]) for p in points) for j in range(3,9)]
                    counterfactual=[{'id':str(i),'position':p['position'][:3]+arm,
                        'velocity':p['velocity'][:3]+[0.]*6,'time_s':p['time_s']} for i,p in enumerate(points)]
                    geometry['initial_steer']=data.get('initial_steer',[0.]*4)
                    fixed=check(folder,run.name+'_fixed_arm',world,metadata,counterfactual)
                    collisions=[s for s in fixed['states'] if not s['valid']]
                    simultaneous=any(math.hypot(*p['velocity'][:2])>.02 and max(abs(v) for v in p['velocity'][3:])>.01 for p in points)
                    attempt.update(joint_excursions_rad=movement,fixed_arm_collision_states=len(collisions),
                        fixed_arm_first_collision=collisions[0] if collisions else None,
                        fixed_arm_comparison_scope='same validated base poses; not a proof of infeasibility of all fixed-arm base routes',
                        arm_and_base_moved_together=simultaneous)
                    if data['status']=='passed' and (not collisions or max(movement[1:])<.15 or not simultaneous):
                        attempt.update(status='failed',reason='no demonstrated necessary non-q1 arm participation during base motion')
                    if attempt['status']=='passed' and (data.get('planning_validation_wall_s') is None or data['planning_validation_wall_s']>=1.):
                        attempt.update(status='failed',reason='complete planning and validation response did not meet the 1000 ms target')
                row['attempts'].append(attempt);print(case['id'],seed,attempt['status'],attempt.get('reason'),flush=True)
        report['cases'].append(row)
        (output/'cooperation_report.json').write_text(json.dumps(report,indent=2)+'\n')
    attempts=[a for row in report['cases'] for a in row['attempts']]
    report['attempts']=len(attempts);report['successes']=sum(a['status']=='passed' for a in attempts)
    report['candidates_generated']=sum(a['candidate_generated'] for a in attempts)
    report['publicly_validated']=sum(a['validation_passed'] for a in attempts)
    successful_times=sorted(1000.*a['planning_validation_wall_s'] for a in attempts if a['status']=='passed')
    if successful_times:
        report['successful_planning_validation_ms']={'n':len(successful_times),
            'min':successful_times[0],'p50':statistics.median(successful_times),
            'p95_nearest_rank':successful_times[math.ceil(.95*len(successful_times))-1],
            'max':successful_times[-1]}
    good=all(row['geometric_need_confirmed'] for row in report['cases'])
    report['status']='geometry_preflight_passed' if args.geometry_only and good else ('all_offline_validated' if good and attempts and report['successes']==len(attempts) else 'failed')
    (output/'cooperation_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='cases'},indent=2))
    return 0 if report['status']!='failed' else 1


if __name__=='__main__':raise SystemExit(main())
