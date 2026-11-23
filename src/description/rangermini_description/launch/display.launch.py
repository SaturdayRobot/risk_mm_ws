"""Inspect the current complete robot in RViz2, independently of Gazebo."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package=Path(get_package_share_directory('rangermini_description'))
    description=(package/'urdf/rangermini_xarm6.urdf').read_text()
    gui=LaunchConfiguration('use_gui')
    config=yaml.safe_load((Path(get_package_share_directory('mm_bringup'))/'config/robot.yaml').read_text())
    defaults={'zeros':dict(zip(config['arm_joint_names'],config['display_arm']))}
    return LaunchDescription([
        DeclareLaunchArgument('use_gui',default_value='true'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',
             parameters=[{'robot_description':ParameterValue(description,value_type=str)}],output='screen'),
        Node(package='joint_state_publisher_gui',executable='joint_state_publisher_gui',parameters=[defaults],condition=IfCondition(gui)),
        Node(package='joint_state_publisher',executable='joint_state_publisher',parameters=[defaults],condition=UnlessCondition(gui)),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(package/'rviz/display.rviz')])])
