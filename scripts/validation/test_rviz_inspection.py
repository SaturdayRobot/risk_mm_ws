#!/usr/bin/env python3
"""Observe the real RViz data pipeline; never publish motion or invoke services."""
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import time
from zoneinfo import ZoneInfo
import yaml

import rclpy
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from risk_mm_interfaces.msg import CoupledStatus
from visualization_msgs.msg import MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',type=Path,required=True)
    parser.add_argument('--wait-complete-s',type=float,default=0.,help='observe a real patrol, including active path and moving local window')
    parser.add_argument('--mission-id',help='require completion of this exact patrol, excluding prior successful missions')
    args=parser.parse_args()
    session=json.loads(args.session.read_text());assert int(os.environ['ROS_DOMAIN_ID'])==session['domain_id']
    patrol_file=Path(session.get('selected_inputs',{}).get('patrol_file',
        str(Path(session['report_dir'])/'inputs/mm_bringup/config/patrol.yaml')))
    expected_goals=len(yaml.safe_load(patrol_file.read_text())['goals'])
    folder=Path(session['report_dir'])/'rviz_checks'/datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S')
    folder.mkdir(parents=True)
    report={'status':'failed','scope':'actual read-only ESDF map/path/marker subscriptions; no motion interface'}
    rclpy.init();node=rclpy.create_node('rviz_inspection_acceptance');data={};origins=set();active_count=0
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def receive(key,message):
        nonlocal active_count
        data[key]=message
        if key=='local':origins.add((round(message.info.origin.position.x,3),round(message.info.origin.position.y,3)))
        if key=='active' and message.poses:active_count+=1
    subs=[]
    for kind,topic,key in [(OccupancyGrid,'global_costmap','global'),(OccupancyGrid,'local_costmap','local'),
                          (RosPath,'validated_route','route'),(RosPath,'active_path','active'),(RosPath,'measured_path','measured'),
                          (MarkerArray,'world_markers','world'),(MarkerArray,'task_markers','markers')]:
        subs.append(node.create_subscription(kind,'/inspection/'+topic,lambda m,k=key:receive(k,m),qos))
    subs.append(node.create_subscription(CoupledStatus,'/coupled/status',lambda m:receive('status',m),10))
    started=time.monotonic()
    try:
        deadline=started+max(8.,args.wait_complete_s)
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            ready=all(k in data for k in ['global','local','route','active','measured','world','markers','status'])
            if not ready:continue
            state=data['status']
            if args.wait_complete_s:
                if (state.plan_id==args.mission_id if args.mission_id else state.plan_id.startswith('patrol_')) and state.execution_completed and state.execution_success:break
            elif any(v>=0 for v in data['global'].data) and data['route'].poses:break
        assert all(k in data for k in ['global','local','route','active','measured','world','markers','status']),list(data)
        state=data['status'];assert state.map_ready and state.feedback_ready,state.reason
        global_map,local=data['global'],data['local']
        for grid in [global_map,local]:
            assert grid.header.frame_id=='world'
            assert grid.info.resolution>0 and len(grid.data)==grid.info.width*grid.info.height
            assert all(-1<=v<=100 for v in grid.data)
        assert sum(v>=0 for v in global_map.data)>0,'global map unknown despite ready source'
        assert any(v>0 for v in global_map.data),'no obstacle distance costs'
        assert any(v==0 for v in global_map.data),'no free-distance cells'
        assert local.info.width<global_map.info.width and local.info.height<global_map.info.height
        assert abs(local.info.resolution-global_map.info.resolution)<1e-8
        checked=0
        for row in range(local.info.height):
            for col in range(local.info.width):
                x=local.info.origin.position.x+(col+.5)*local.info.resolution
                y=local.info.origin.position.y+(row+.5)*local.info.resolution
                gx=math.floor((x-global_map.info.origin.position.x)/global_map.info.resolution)
                gy=math.floor((y-global_map.info.origin.position.y)/global_map.info.resolution)
                actual=local.data[row*local.info.width+col]
                if 0<=gx<global_map.info.width and 0<=gy<global_map.info.height:
                    assert actual==global_map.data[gy*global_map.info.width+gx],(gx,gy,actual)
                    checked+=1
                else:assert actual==-1,'outside global coverage must be unknown'
        assert checked>0
        points=[m for m in data['markers'].markers if m.ns=='inspection_points' and m.action==m.ADD]
        assert len(points)==expected_goals,(len(points),expected_goals)
        assert len(data['world'].markers[0].points)>0
        route=data['route'];assert route.header.frame_id=='world' and len(route.poses)>1
        jumps=[math.hypot(a.pose.position.x-b.pose.position.x,a.pose.position.y-b.pose.position.y) for a,b in zip(route.poses,route.poses[1:])]
        assert max(jumps,default=0.)<.2,'disconnected validated route'
        if args.wait_complete_s:
            if args.mission_id:assert state.plan_id==args.mission_id,(state.plan_id,args.mission_id)
            assert state.execution_completed and state.execution_success,state.reason
            assert active_count>0,'no live active segment displayed'
            assert len(origins)>1,'local map did not follow the actual robot'
            assert len(data['measured'].poses)>20
            for marker in points:
                distance=min(math.hypot(p.pose.position.x-marker.pose.position.x,p.pose.position.y-marker.pose.position.y) for p in route.poses)
                assert distance<.08,(marker.id,distance)
        report.update(status='passed',global_size=[global_map.info.width,global_map.info.height],
            resolution_m=global_map.info.resolution,global_known_cells=sum(v>=0 for v in global_map.data),
            cost_range=[min(global_map.data),max(global_map.data)],local_size=[local.info.width,local.info.height],
            matching_local_cells=checked,local_origins_observed=len(origins),active_messages_observed=active_count,
            validated_route_points=len(route.poses),measured_path_points=len(data['measured'].poses),
            maximum_path_step_m=max(jumps,default=0.),inspection_markers=len(points),
            world_box_edges=len(data['world'].markers[0].points)//2,plan_id=state.plan_id,
            execution_completed=state.execution_completed,execution_success=state.execution_success)
        report['requested_mission_id']=args.mission_id
    except Exception as error:report['reason']=str(error)
    finally:
        report['wall_s']=time.monotonic()-started
        (folder/'validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
        node.destroy_node();rclpy.shutdown()
    return 0 if report['status']=='passed' else 1


if __name__=='__main__':raise SystemExit(main())
