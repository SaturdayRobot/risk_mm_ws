from pathlib import Path
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    urdf=(Path(share('rangermini_description'))/'urdf/rangermini_xarm6.urdf').read_text()
    return LaunchDescription([Node(package='robot_state_publisher',executable='robot_state_publisher',
        parameters=[{'robot_description':ParameterValue(urdf,value_type=str)}])])
