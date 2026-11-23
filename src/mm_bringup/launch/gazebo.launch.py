"""Independent Gazebo plant and manual controls; no planner execution wiring."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory as share, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    server = ExecuteProcess(cmd=['gzserver', '--verbose', str(world), '-s', 'libgazebo_ros_init.so',
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
        return [Node(package='mm_bringup', executable='sim_control', parameters=[sim], output='screen'),
                Node(package='mm_bringup', executable='sim_control_gui', parameters=[sim], output='screen',
                     condition=IfCondition(LaunchConfiguration('controls')))]

    return [server,
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
        DeclareLaunchArgument('world', default_value='s6_factory_inspection_v1'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('controls', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('x', default_value='-8.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=assemble)])
