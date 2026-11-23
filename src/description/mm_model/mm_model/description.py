"""Derive planner geometry from the same URDF used by Gazebo."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

ARM_NAMES = [f'xarm6_joint{i}' for i in range(1, 7)]


def load_arm_posture(path, name='q_ready'):
    postures = yaml.safe_load(Path(path).read_text())
    if name not in postures:
        raise ValueError(f'Unknown arm posture: {name}')
    angles = [float(value) for value in postures[name]]
    if len(angles) != len(ARM_NAMES) or not all(map(math.isfinite, angles)):
        raise ValueError(f'Arm posture {name} requires six finite joint angles')
    return angles


def origin(element):
    out = np.eye(4)
    el = element.find('origin')
    if el is None:
        return out
    out[:3, 3] = np.fromstring(el.get('xyz', '0 0 0'), sep=' ')
    r, p, y = np.fromstring(el.get('rpy', '0 0 0'), sep=' ')
    cx, sx, cy, sy, cz, sz = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    out[:3, :3] = np.array([[cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx],
                           [sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx],
                           [-sy, cy*sx, cy*cx]])
    return out


def mesh_path(uri):
    if uri.startswith('package://'):
        package, relative = uri[10:].split('/', 1)
        return Path(get_package_share_directory(package)) / relative
    if uri.startswith('file://'):
        return Path(uri[7:])
    raise ValueError(f'Unsupported mesh URI: {uri}')


def vertices_for(link):
    # The chassis includes collision-only accessories hidden from the viewer.
    # Planning must retain their geometry even when they have no visual.
    elements = link.findall('collision') if link.get('name') == 'base_link' else link.findall('visual')[:1]
    vertices = []
    for element in elements:
        mesh = element.find('geometry/mesh')
        if mesh is None:
            continue
        path = mesh_path(mesh.get('filename'))
        if path.suffix.lower() != '.stl':
            path = path.with_suffix('.stl')
        data = path.read_bytes()
        count = int.from_bytes(data[80:84], 'little')
        dtype = np.dtype([('normal','<f4',3),('vertices','<f4',(3,3)),('attr','<u2')])
        v = np.frombuffer(data, dtype=dtype, offset=84, count=count)['vertices'].reshape(-1, 3).astype(float)
        v *= np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
        vertices.append((np.c_[v, np.ones(len(v))] @ origin(element).T)[:, :3])
    return np.concatenate(vertices) if vertices else None


def box_spheres(vertices, radius):
    low, high = vertices.min(axis=0), vertices.max(axis=0)
    # Each box cell fits inside its sphere, including the corners.
    n = np.maximum(1, np.ceil((high-low)/(2*radius/math.sqrt(3)))).astype(int)
    axes = [np.linspace(low[i], high[i], 2*n[i]+1)[1::2] for i in range(3)]
    return np.array(np.meshgrid(*axes, indexing='ij')).reshape(3,-1).T


def load_model(path):
    root = ET.parse(path).getroot()
    links = {l.get('name'): l for l in root.findall('link')}
    joints = {j.get('name'): j for j in root.findall('joint')}
    parent_joint = {j.find('child').get('link'): j for j in joints.values()}
    def fixed_transform(link, ancestor):
        result = np.eye(4)
        while link != ancestor:
            j = parent_joint[link]
            if j.get('type') != 'fixed':
                raise ValueError(f'Unexpected moving joint {j.get("name")} in fixed chain')
            result = origin(j) @ result
            link = j.find('parent').get('link')
        return result
    mount = fixed_transform(joints[ARM_NAMES[0]].find('parent').get('link'), 'base_link')
    radius = .030
    result = {'mm.model_id': root.get('name'), 'mm.arm_joint_names': ARM_NAMES,
              'mm.arm_mount_transform': mount.flatten().tolist(),
              'mm.manipulator_thickness': radius, 'mm.mobile_base_dof': 2,
              'mm.manipulator_dof': 6, 'mm.urdf_source': str(path)}
    origins, axes, minimum, maximum, velocities, resources = [], [], [], [], [], []
    for i, name in enumerate(ARM_NAMES):
        j = joints[name]
        origins.extend(origin(j).flatten().tolist())
        axes.extend(np.fromstring(j.find('axis').get('xyz'), sep=' ').tolist())
        limit = j.find('limit')
        minimum.append(float(limit.get('lower'))); maximum.append(float(limit.get('upper')))
        velocities.append(float(limit.get('velocity')))
        child = j.find('child').get('link'); link = links[child]
        verts = vertices_for(link)
        points = box_spheres(verts, radius)
        owners=[float(i)]*len(points)
        visual = link.find('visual/geometry/mesh')
        resources.append(mesh_path(visual.get('filename')).as_uri())
        # Include the camera and other fixed end-effector attachments in link 6.
        if i == 5:
            for other in links:
                if other == child:
                    continue
                try:
                    trans = fixed_transform(other, child)
                except (KeyError, ValueError):
                    continue
                v = vertices_for(links[other])
                if v is not None:
                    p = box_spheres(v, radius)
                    points = np.r_[points, (np.c_[p,np.ones(len(p))] @ trans.T)[:,:3]]
                    owners.extend([6.0]*len(p))
        result[f'mm.arm_collision_points_{i}'] = points.flatten().tolist()
        result[f'mm.arm_collision_owners_{i}'] = owners
    # Official xArm6 SRDF (humble), plus the fixed camera-to-link6 attachment.
    # https://github.com/xArm-Developer/xarm_ros2/blob/humble/xarm_moveit_config/srdf/_xarm6_macro.srdf.xacro
    pairs=[(0,1),(1,2),(2,3),(3,4),(4,5),(0,2),(1,3),(2,4),(2,5),(3,5),(4,6),(5,6)]
    result['mm.self_collision_disabled_pairs']=[float(v) for pair in pairs for v in pair]
    result.update({'mm.arm_joint_origins' : origins, 'mm.arm_joint_axes': axes,
                   'mm.manipulator_min_pos': minimum, 'mm.manipulator_max_pos': maximum,
                   'mm.arm_velocity_limits': velocities, 'mm.arm_mesh_resources': resources})
    base_vertices = vertices_for(links['base_link'])
    owners=np.zeros(len(base_vertices),dtype=int)
    # Fixed arm pedestal/adapter belong to the mounting assembly. Its contact
    # with proximal arm housings is exempt; the chassis remains checked.
    for other in links:
        if other == 'base_link':continue
        try:trans=fixed_transform(other,'base_link')
        except (KeyError,ValueError):continue
        vertices=vertices_for(links[other])
        if vertices is not None:
            base_vertices=np.r_[base_vertices,(np.c_[vertices,np.ones(len(vertices))]@trans.T)[:,:3]]
            owners=np.r_[owners,np.ones(len(vertices),dtype=int)]
    pitch = .04
    cells,inverse=np.unique(np.floor(base_vertices/pitch).astype(int),axis=0,return_inverse=True)
    cell_owners=np.ones(len(cells),dtype=int)
    np.minimum.at(cell_owners,inverse,owners)  # chassis takes precedence in mixed cells
    base_points=(cells+.5)*pitch
    result['mm.base_collision_points'] = base_points.flatten().tolist()
    result['mm.base_collision_owners'] = cell_owners.astype(float).tolist()
    result['mm.mobile_base_check_radius'] = .04
    result['mm.base_mesh_resource'] = mesh_path(links['base_link'].find('visual/geometry/mesh').get('filename')).as_uri()
    result['controller.wheel_positions'] = [float(v) for w in ['fl','fr','rl','rr']
        for v in origin(joints[f'joint_steer_{w}'])[:2,3]]
    return result
