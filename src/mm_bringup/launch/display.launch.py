"""Assemble the model viewer with this workspace's single display initial state."""
import json
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config = yaml.safe_load((Path(share('mm_bringup')) / 'config/robot.yaml').read_text())
    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(share('rangermini_description')) / 'launch/display.launch.py')),
            launch_arguments={
                'use_gui': LaunchConfiguration('use_gui'),
                'joint_defaults': json.dumps(dict(zip(config['arm_joint_names'], config['display_arm']))),
            }.items())])
