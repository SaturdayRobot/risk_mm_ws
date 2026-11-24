#!/usr/bin/env python3
"""Inventory the planning meshes and C++ Gazebo proxies; check both start poses."""
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory as share
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from mm_model.description import load_model, origin
from mm_model.wheel_ik import WHEEL_ORDER, WHEEL_POSITIONS


def main():
    ws = Path(__file__).resolve().parents[2]
    previous = ws / 'reports/collision_model.json'
    if previous.exists():
        index = 1
        while (ws / f'reports/collision_model_attempt_{index}.json').exists():
            index += 1
        previous.rename(ws / f'reports/collision_model_attempt_{index}.json')
    report = {'status': 'failed', 'scope': 'static geometry/configuration and two sampled poses; no continuous safety claim'}
    try:
        model = Path(share('rangermini_description')) / 'urdf/rangermini_xarm6.urdf'
        source = ET.parse(model).getroot()
        command = [str(Path(get_package_prefix('mm_bringup')) / 'lib/mm_bringup/prepare_sim')]
        prepared = subprocess.check_output(command, text=True, timeout=10.)
        sim = ET.fromstring(prepared)
        (ws / 'reports/prepared_robot.urdf').write_text(prepared)
        assert source.get('name') == sim.get('name') == 'rangermini_xarm6'
        assert {l.get('name') for l in source.findall('link')} == {l.get('name') for l in sim.findall('link')}
        assert 'package://' not in prepared and '$(find' not in prepared
        inventory = []
        for link in sim.findall('link'):
            for collision in link.findall('collision'):
                shape = list(collision.find('geometry'))[0]
                assert shape.tag in ('box', 'cylinder')
                if shape.tag == 'box':
                    assert all(v > 0. for v in map(float, shape.get('size').split()))
                else:
                    assert float(shape.get('radius')) > 0 and float(shape.get('length')) > 0
                frame = collision.find('origin')
                inventory.append({'link': link.get('name'), 'shape': shape.tag,
                    'dimensions': dict(shape.attrib), 'xyz': frame.get('xyz', '0 0 0') if frame is not None else '0 0 0',
                    'rpy': frame.get('rpy', '0 0 0') if frame is not None else '0 0 0'})
        assert len(inventory) == 23
        for link in sim.findall('link'):
            if link.get('name').startswith('wheel_'):
                shape = link.find('collision/geometry/cylinder')
                assert float(shape.get('radius')) == .1 and float(shape.get('length')) == .075
        params = yaml.safe_load((ws / 'src/mm_bringup/config/remani.yaml').read_text())['/**']['ros__parameters']
        derived = load_model(model)
        assert params['mm.manipulator_thickness'] == derived['mm.manipulator_thickness'] == .03
        params.update(derived)
        assert np.allclose(np.array(derived['controller.wheel_positions']).reshape(4, 2),
                           [WHEEL_POSITIONS[name] for name in WHEEL_ORDER])
        config = yaml.safe_load((ws / 'src/mm_bringup/config/robot.yaml').read_text())
        names = config['arm_joint_names']
        parents = {j.find('child').get('link'): j for j in source.findall('joint')}

        def fk(q, tip):
            chain = []
            while tip != 'base_link':
                joint = parents[tip]
                chain.append(joint)
                tip = joint.find('parent').get('link')
            result = np.eye(4)
            for joint in reversed(chain):
                local = origin(joint)
                if joint.get('name') in names:
                    axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                    local[:3, :3] = local[:3, :3] @ Rotation.from_rotvec(axis * q[names.index(joint.get('name'))]).as_matrix()
                result = result @ local
            return result

        display = fk(config['display_arm'], 'head_camera_link')
        mount = source.find("joint[@name='head_mount']")
        assert np.allclose(origin(mount)[:3, 3], [-.0117, -.0072, .057])
        assert np.allclose(origin(mount)[:3, :3], Rotation.from_euler('z', np.pi).as_matrix())
        confirmed_face = np.array([.31270015, .94820117, .05597460])
        confirmed_face /= np.linalg.norm(confirmed_face)
        head = fk(config['display_arm'], 'head_link')
        assert np.linalg.norm(display[:3, 0] - head[:3, :3] @ confirmed_face) < 1e-6
        assert display[2, 2] > .999
        assert np.linalg.norm(display[:3, 0] - [-1., 0., 0.]) < 1e-6
        assert display[2, 3] < 1.4
        lidar = fk(config['display_arm'], 'lidar_link')
        front_camera = fk(config['display_arm'], 'camera_color_optical_frame')
        assert lidar[0, 3] > 0. and front_camera[0, 3] > 0.
        assert np.linalg.norm(lidar[:3, :3] - np.eye(3)) < 1e-6
        assert np.linalg.norm(front_camera[:3, 2] - [1., 0., 0.]) < 1e-6
        for wheel in ['fl', 'fr', 'rl', 'rr']:
            transform = fk([0.] * 6, 'wheel_' + wheel)
            axis = np.fromstring(source.find(f"joint[@name='joint_wheel_{wheel}']/axis").get('xyz'), sep=' ')
            # This is the retained model +X; user front is base_control -X.
            assert np.linalg.norm(transform[:3, :3] @ axis - [0., 1., 0.]) < 1e-6
        checks = {}
        for key in ('stow', 'display_arm'):
            fixture = dict(params, **{'test.q': config[key],
                'test.expected_ee': fk(config[key], 'xarm6_link6').flatten().tolist()})
            with tempfile.TemporaryDirectory(prefix='risk_mm_collision_') as folder:
                path = Path(folder) / 'params.yaml'
                path.write_text(yaml.safe_dump({'/**': {'ros__parameters': fixture}}))
                result = subprocess.run([str(Path(get_package_prefix('mm_config')) / 'lib/mm_config/model_check'),
                    '--ros-args', '--params-file', str(path)], text=True, capture_output=True, timeout=20.,
                    env=dict(os.environ, ROS_DOMAIN_ID='183', ROS_LOCALHOST_ONLY='1'))
            checks[key] = {'returncode': result.returncode, 'stdout': result.stdout, 'stderr': result.stderr}
            if result.returncode:
                report['pose_checks'] = checks
                raise RuntimeError('native model check failed: ' + key)
        report.update(status='passed', urdf_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
            gazebo_collision_bodies=inventory, self_collision_enabled=sim.find('gazebo/selfCollide').text == 'true',
            planner_arm_sphere_radius_m=derived['mm.manipulator_thickness'],
            planner_base_sphere_radius_m=derived['mm.mobile_base_check_radius'],
            planner_base_samples=len(derived['mm.base_collision_points']) // 3,
            planner_arm_samples=[len(derived[f'mm.arm_collision_points_{i}']) // 3 for i in range(6)],
            pose_checks=checks, display_camera_xyz=display[:3, 3].tolist(),
            camera_flange_mount=dict(mount.find('origin').attrib),
            display_camera_forward=display[:3, 0].tolist(), display_camera_up=display[:3, 2].tolist())
        with (ws / 'reports/gazebo_collision_bodies.csv').open('w') as stream:
            writer = csv.DictWriter(stream, fieldnames=['link', 'shape', 'dimensions', 'xyz', 'rpy'])
            writer.writeheader()
            writer.writerows(inventory)
    except Exception as error:
        report['reason'] = str(error)
    (ws / 'reports/collision_model.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ('gazebo_collision_bodies', 'pose_checks')}, ensure_ascii=False, indent=2))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
