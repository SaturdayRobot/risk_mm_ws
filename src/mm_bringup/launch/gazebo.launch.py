"""Independent Gazebo plant, C++ coupled runtime and operator controls."""
import os
import hashlib
import json
import math
import yaml
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory as share, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from mm_model.description import load_model
from mm_model.identity import robot_signature


def assemble(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    bringup = Path(share('mm_bringup'))
    world = bringup / 'worlds' / (get('world') + '.world')
    if not world.is_file():
        raise ValueError('Unknown local world: ' + str(world))
    if not os.environ.get('GAZEBO_MASTER_URI', '').startswith('http://127.0.0.1:'):
        raise ValueError('Use scripts/in_env with an isolated GAZEBO_MASTER_URI')
    env = {'GAZEBO_MODEL_DATABASE_URI': '', 'GAZEBO_MODEL_PATH': '/usr/share/gazebo-11/models'}
    sim = {'use_sim_time': True}
    gui_parameters = dict(sim, world_name=get('world'))
    gui_goal = json.loads(get('gui_initial_goal'))
    if gui_goal:
        if len(gui_goal) != 9 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in gui_goal):
            raise ValueError('gui_initial_goal must contain nine finite model-frame goal values')
        gui_parameters['initial_goal'] = [float(v) for v in gui_goal]
    # Keep the copied source world immutable; add only a scene-state publisher.
    tree = ET.parse(world)
    element = tree.getroot().find('world')
    plugin = ET.SubElement(element, 'plugin', name='risk_mm_scene_state', filename='libgazebo_ros_state.so')
    ET.SubElement(ET.SubElement(plugin, 'ros'), 'namespace').text = '/gazebo'
    ET.SubElement(plugin, 'update_rate').text = '30.0'
    descriptor, temporary_world = tempfile.mkstemp(prefix='risk_mm_scene_', suffix='.world')
    os.close(descriptor)
    tree.write(temporary_world)
    def cleanup(context):
        Path(temporary_world).unlink(missing_ok=True)
        return []
    urdf = Path(share('rangermini_description')) / 'urdf/rangermini_xarm6.urdf'
    derived = load_model(urdf)
    derived['mm.planning_frame_yaw_in_base_rad'] = yaml.safe_load(
        (bringup / 'config/manual_control.yaml').read_text())['control_frame_yaw_in_base_rad']
    coupled = dict(derived, **sim)
    coupled.update({
        'model_signature': robot_signature(urdf, [bringup / ('config/' + name) for name in ('robot.yaml', 'remani.yaml', 'simulation_collision.yaml', 'coupled_execution.yaml', 'manual_control.yaml')] + [Path(share('rangermini_moveit_config')) / 'config/rangermini_xarm6.srdf'], model_data=derived),
        'robot_urdf_file': str(urdf), 'robot_urdf_sha256': hashlib.sha256(urdf.read_bytes()).hexdigest(),
        'robot_srdf_file': str(Path(share('rangermini_moveit_config')) / 'config/rangermini_xarm6.srdf'),
        'world_file': str(world), 'world_metadata_file': str(world.with_suffix('.json')),
        'report_directory': get('report_directory'), 'grid_map.external_snapshots': True})
    coupled['patrol_file'] = str(bringup / 'config/patrol.yaml')
    runtime = Node(package='mm_bringup', executable='simulation_runtime',
                   parameters=[str(bringup / 'config/remani.yaml'), str(bringup / 'config/coupled_execution.yaml'), coupled], output='screen')
    server = ExecuteProcess(cmd=['gzserver', '--verbose', temporary_world, '-s', 'libgazebo_ros_init.so',
                                 '-s', 'libgazebo_ros_factory.so', '--ros-args',
                                 '--params-file', str(bringup / 'config/gazebo.yaml')],
                            additional_env=env, output='screen')
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
                 arguments=['-topic', 'robot_description', '-entity', 'rangermini',
                            '-x', get('x'), '-y', get('y'), '-z', '.003', '-Y', get('yaw'), '-timeout', '120'])
    controllers = Node(package='controller_manager', executable='spawner', output='screen',
                       arguments=['joint_state_broadcaster', 'arm_position_controller',
                                  'steer_position_controller', 'wheel_velocity_controller',
                                  '--controller-manager-timeout', '90'])

    def spawned(event, context):
        return [controllers] if event.returncode == 0 else [EmitEvent(event=Shutdown(reason='Robot spawn failed'))]

    def activated(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Controller activation failed'))]
        return [runtime,
                Node(package='mm_bringup', executable='sim_control_gui', parameters=[gui_parameters], output='screen',
                     condition=IfCondition(LaunchConfiguration('controls')))]

    return [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
            RegisterEventHandler(OnProcessExit(target_action=runtime, on_exit=[
                EmitEvent(event=Shutdown(reason='C++ simulation runtime exited'))])),
            server,
            ExecuteProcess(cmd=['gzclient'], additional_env=env, output='screen',
                           condition=IfCondition(LaunchConfiguration('gui'))),
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[sim, {'robot_description': ParameterValue(Command([
                     str(Path(get_package_prefix('mm_bringup')) / 'lib/mm_bringup/prepare_sim')]), value_type=str)}]),
            spawn,
            RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=spawned)),
            RegisterEventHandler(OnProcessExit(target_action=controllers, on_exit=activated)),
            RegisterEventHandler(OnProcessExit(target_action=server, on_exit=[
                EmitEvent(event=Shutdown(reason='Gazebo server exited'))])),
            Node(package='rviz2', executable='rviz2', parameters=[sim],
                 arguments=['-d', str(Path(share('rangermini_description')) / 'rviz/display.rviz')],
                 condition=IfCondition(LaunchConfiguration('rviz')))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('report_directory', default_value=str(Path(os.environ.get('RISK_MM_ROOT', str(Path.cwd()))) / 'reports/runtime')),
        DeclareLaunchArgument('gui_initial_goal', default_value='[]'),
        DeclareLaunchArgument('world', default_value='s6_factory_inspection_v1'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('controls', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('x', default_value='-8.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='3.141592653589793'),
        OpaqueFunction(function=assemble)])
