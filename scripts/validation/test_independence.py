#!/usr/bin/env python3
"""Audit local resources and sourced prefixes without consulting the old workspace."""
from pathlib import Path
import hashlib,json,os,xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_prefix, get_package_share_directory as share

def main():
    ws=Path(__file__).resolve().parents[2]
    packages={ET.parse(p).getroot().find('name').text for p in (ws/'src').rglob('package.xml')}
    try:
        for key in ['AMENT_PREFIX_PATH','COLCON_PREFIX_PATH','CMAKE_PREFIX_PATH','PYTHONPATH','LD_LIBRARY_PATH']:
            assert 'h_acmp_ws' not in os.environ.get(key,''),key
        for package in packages:assert Path(get_package_prefix(package)).is_relative_to(ws),package
        assert not [p for p in (ws/'src').rglob('*') if p.is_symlink()]
        urdf=ET.parse(Path(share('rangermini_description'))/'urdf/rangermini_xarm6.urdf')
        meshes=urdf.findall('.//mesh')
        for mesh in meshes:
            package,relative=mesh.get('filename')[10:].split('/',1)
            assert (Path(share(package))/relative).is_file(),mesh.get('filename')
        upstream=ws/'third_party/remani_original'
        manifest=json.loads((upstream/'UPSTREAM.json').read_text())
        for name,sha in manifest['git_blob_sha'].items():
            data=(upstream/name).read_bytes()
            assert hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()==sha,name
        report={'status':'passed','packages':len(packages),'local_mesh_references':len(meshes),
                'verified_upstream_files':len(manifest['git_blob_sha']),
                'ament_prefix_path':os.environ.get('AMENT_PREFIX_PATH','')}
    except Exception as error:
        report={'status':'failed','reason':str(error)}
    (ws/'reports/independence.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    return 0 if report['status']=='passed' else 1

if __name__=='__main__':raise SystemExit(main())
