#!/usr/bin/env python3
"""Exercise original REMANI module initialization without giving motion permission."""
from pathlib import Path
import json,os,subprocess,tempfile
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory as share
from mm_model.description import load_model

def main():
    ws=Path(__file__).resolve().parents[2]
    params=yaml.safe_load((ws/'src/mm_bringup/config/remani.yaml').read_text())['/**']['ros__parameters']
    params.update(load_model(Path(share('rangermini_description'))/'urdf/rangermini_xarm6.urdf'))
    executable=Path(get_package_prefix('remani_adapter'))/'lib/remani_adapter/coupled_check'
    with tempfile.TemporaryDirectory(prefix='risk_mm_init_') as folder:
        config=Path(folder)/'params.yaml'
        config.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
        command=[str(executable),'--ros-args','--params-file',str(config)]
        try:
            result=subprocess.run(command,text=True,capture_output=True,timeout=30,
                env=dict(os.environ,ROS_DOMAIN_ID='163',ROS_LOCALHOST_ONLY='1'))
            report={'returncode':result.returncode,'stdout':result.stdout,'stderr':result.stderr,
                    'planning_success':None,'execution_success':None,'scope':'initialization only'}
        except subprocess.TimeoutExpired as e:
            report={'returncode':124,'stdout':str(e.stdout),'stderr':str(e.stderr),
                    'planning_success':None,'execution_success':None,'scope':'initialization timed out'}
    (ws/'reports/initialization.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(report['stdout'],end='');print(report['stderr'],end='')
    return report['returncode']

if __name__=='__main__':raise SystemExit(main())
