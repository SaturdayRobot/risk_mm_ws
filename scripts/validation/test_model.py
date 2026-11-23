#!/usr/bin/env python3
"""Current URDF / native FK, gradient and self-collision regression; isolated ROS."""
from pathlib import Path
import json, os, subprocess, tempfile, math
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory as share, get_package_prefix
from mm_model.description import load_model
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation

WS=Path(__file__).resolve().parents[2]

def main():
    urdf=Path(share('rangermini_description'))/'urdf/rangermini_xarm6.urdf'
    config=yaml.safe_load((WS/'src/mm_bringup/config/robot.yaml').read_text())
    q=config['stow'];params=load_model(urdf)
    root=ET.parse(urdf).getroot()
    joints={j.get('name'):j for j in root.findall('joint')}
    parents={j.find('child').get('link'):j for j in joints.values()}
    chain=[];link='xarm6_link6'
    while link!='base_link':
        j=parents[link];chain.append(j);link=j.find('parent').get('link')
    expected=np.eye(4)
    for j in reversed(chain):
        origin=j.find('origin');local=np.eye(4)
        if origin is not None:
            local[:3,:3]=Rotation.from_euler('xyz',[float(x) for x in origin.get('rpy','0 0 0').split()]).as_matrix()
            local[:3,3]=[float(x) for x in origin.get('xyz','0 0 0').split()]
        if j.get('name') in config['arm_joint_names']:
            i=config['arm_joint_names'].index(j.get('name'))
            axis=np.array([float(x) for x in j.find('axis').get('xyz').split()]);axis/=np.linalg.norm(axis)
            local[:3,:3]=local[:3,:3]@Rotation.from_rotvec(axis*q[i]).as_matrix()
            limit=j.find('limit')
            if not float(limit.get('lower'))<=q[i]<=float(limit.get('upper')):raise ValueError('stow outside URDF limit')
        expected=expected@local
    params.update({'test.q':q,'test.expected_ee':expected.flatten().tolist()})
    env=dict(os.environ,ROS_DOMAIN_ID='163',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR='/tmp/risk_mm_model_logs')
    with tempfile.TemporaryDirectory(prefix='risk_mm_model_') as folder:
        path=Path(folder)/'params.yaml';path.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
        executable=Path(get_package_prefix('mm_config'))/'lib/mm_config/model_check'
        result=subprocess.run([str(executable),'--ros-args','--params-file',str(path)],env=env,text=True,capture_output=True,timeout=60)
    report={'model_id':config['model_id'],'returncode':result.returncode,'stdout':result.stdout,'stderr':result.stderr,
            'scope':'FK, analytic gradient and sampled self collision at stow; not continuous path safety'}
    (WS/'reports').mkdir(exist_ok=True)
    (WS/'reports/model.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(result.stdout,end='');print(result.stderr,end='')
    return result.returncode

if __name__=='__main__':raise SystemExit(main())
