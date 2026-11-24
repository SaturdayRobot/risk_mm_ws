#!/usr/bin/env python3
"""Audit local resources and sourced prefixes without consulting the old workspace."""
from pathlib import Path
import difflib,hashlib,json,os,re,xml.etree.ElementTree as ET
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
        patch=(ws/'docs/remani_ros2.patch').read_text()
        pairs=list(zip(re.findall(r'^--- (.+)$',patch,re.M),re.findall(r'^\+\+\+ (.+)$',patch,re.M)))
        assert len(pairs)==10,'unexpected REMANI patch inventory'
        expected=''.join(''.join(difflib.unified_diff(
            [] if original=='/dev/null' else (upstream/original).read_text().splitlines(keepends=True),
            (ws/ported).read_text().splitlines(keepends=True),fromfile=original,tofile=ported))
            for original,ported in pairs)
        assert patch==expected,'REMANI port patch is stale'
        world_manifest=json.loads((ws/'src/mm_bringup/worlds/SOURCE.json').read_text())
        for asset in world_manifest['files']:
            local=ws/asset['local']
            installed=Path(share('mm_bringup'))/'worlds'/local.name
            for path in (local,installed):
                assert hashlib.sha256(path.read_bytes()).hexdigest()==asset['sha256'],str(path)
        native=Path(get_package_prefix('mm_bringup'))/'lib/mm_bringup'
        for name in ('prepare_sim','sim_control','sim_control_gui','simulation_runtime'):
            assert (native/name).read_bytes()[:4]==b'\x7fELF',name
        probe=Path(get_package_prefix('risk_mm_runtime'))/'lib/risk_mm_runtime/coupled_probe'
        assert probe.read_bytes()[:4]==b'\x7fELF','coupled_probe'
        assert not (ws/'src/mm_bringup/mm_bringup').exists(),'abandoned Python simulation package'
        report={'status':'passed','packages':len(packages),'local_mesh_references':len(meshes),
                'verified_upstream_files':len(manifest['git_blob_sha']),
                'remani_patch_matches_sources':True,
                'local_world_assets':len(world_manifest['files']),'native_sim_executables':4,
                'ament_prefix_path':os.environ.get('AMENT_PREFIX_PATH','')}
    except Exception as error:
        report={'status':'failed','reason':str(error)}
    (ws/'reports/independence.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    return 0 if report['status']=='passed' else 1

if __name__=='__main__':raise SystemExit(main())
