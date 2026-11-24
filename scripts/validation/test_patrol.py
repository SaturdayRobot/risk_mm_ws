#!/usr/bin/env python3
"""Exercise the real autonomous coupled patrol; preserve every attempt."""
import argparse
import json
import math
import os
from pathlib import Path
import time

import rclpy
from risk_mm_interfaces.msg import CoupledStatus
from risk_mm_interfaces.srv import StartPatrol
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--session',type=Path)
    parser.add_argument('--timeout-s',type=float,default=120.)
    parser.add_argument('--max-planning-ms',type=float,default=500.)
    args=parser.parse_args()
    ws=Path(__file__).resolve().parents[2]
    session=json.loads((args.session or ws/'reports/gazebo_headless_latest.json').read_text())
    assert int(os.environ['ROS_DOMAIN_ID'])==session['domain_id']
    folder=Path(session['report_dir']);number=1
    while (folder/f'patrol_validation_{number}.json').exists():number+=1
    report={'status':'failed','execution_started':False,'execution_completed':False,'transitions':[]}
    rclpy.init();node=rclpy.create_node('autonomous_patrol_acceptance');data={};clients=[]
    def state(message):data['robot']=message
    def patrol(message):
        value=json.loads(message.data);data['patrol']=value
        key={k:value[k] for k in ('mission_id','phase','visited','planned')}
        if not report['transitions'] or any(report['transitions'][-1][k]!=v for k,v in key.items()):
            report['transitions'].append(dict(key,wall_s=time.monotonic()-started))
    subscriptions=[node.create_subscription(CoupledStatus,'/coupled/status',state,10),
                   node.create_subscription(String,'/coupled/patrol_status',patrol,10)]
    started=time.monotonic()
    def wait(condition,timeout=15.):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01)
            if condition():return
        raise TimeoutError('patrol timeout: '+str(data.get('patrol')))
    def call(kind,topic,request):
        client=node.create_client(kind,topic);clients.append(client)
        assert client.wait_for_service(timeout_sec=5.),topic
        future=client.call_async(request);wait(future.done);return future.result()
    try:
        wait(lambda:'robot' in data and data['robot'].map_ready and data['robot'].feedback_ready)
        assert data['robot'].phase not in ('PLANNING','ALIGNING','EXECUTING','SETTLING'),'existing work active'
        assert call(Trigger,'/simulation/stop',Trigger.Request()).success
        end=time.monotonic()+.5
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
        initial=list(data['robot'].state);report['initial_state']=initial
        request=StartPatrol.Request(task_id='autonomous_patrol_acceptance',model_signature=data['robot'].model_signature,
            map_revision=data['robot'].map_revision,speed_limits=[.4,.6,.5])
        accepted=call(StartPatrol,'/coupled/start_patrol',request)
        assert accepted.accepted,accepted.reason
        mission=accepted.mission_id;report['mission_id']=mission
        wait(lambda:data['robot'].plan_id==mission and data['robot'].execution_started or
             data.get('patrol',{}).get('mission_id')==mission and data['patrol']['phase']=='HOLD',20.)
        report['execution_started']=data['robot'].execution_started
        wait(lambda:data.get('patrol',{}).get('mission_id')==mission and data['patrol']['phase'] in ('COMPLETED','HOLD'),args.timeout_s)
        result_file=folder/'patrol'/mission/'mission.yaml'
        result=yaml.safe_load(result_file.read_text());report['mission']=result;report['mission_report']=str(result_file)
        report['execution_completed']=data['robot'].execution_completed
        assert result['success'] and result['visited']==result['inspection_points'],result['reason']
        assert report['execution_started'] and report['execution_completed']
        assert result['return_error_m']<.04
        assert result['planning_stops']==0
        assert all(v['measured_speed_m_s']>.02 for v in result['visits']),'stopped at an inspection point'
        assert all(v['arm_error_rad']<.04 for v in result['visits']),'arm missed inspection configuration'
        assert any(v['requested_while_moving'] for v in result['segments'][1:]),'no planning was observed in motion'
        latencies=sorted(v['planning_ms'] for v in result['segments'])
        report['planning_ms']={'min':latencies[0],'median':latencies[len(latencies)//2],'max':latencies[-1]}
        assert latencies[-1]<args.max_planning_ms,'planning latency exceeds the acceptance budget'
        report['status']='passed'
    except Exception as error:
        report['reason']=str(error)
    finally:
        report['wall_s']=time.monotonic()-started
        (folder/f'patrol_validation_{number}.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(report,ensure_ascii=False,indent=2))
        node.destroy_node();rclpy.shutdown()
    return 0 if report['status']=='passed' else 1


if __name__=='__main__':raise SystemExit(main())
