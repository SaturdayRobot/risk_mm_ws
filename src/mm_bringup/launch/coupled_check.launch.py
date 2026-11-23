from pathlib import Path
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from mm_model.description import load_model

def generate_launch_description():
    model = Path(share('rangermini_description'))/'urdf/rangermini_xarm6.urdf'
    config = Path(share('mm_bringup'))/'config/remani.yaml'
    check = Node(package='remani_adapter', executable='coupled_check', output='screen',
                 parameters=[str(config), load_model(model),
                             {'test.run_plan': ParameterValue(LaunchConfiguration('run_plan'), value_type=bool)}])
    return LaunchDescription([
        DeclareLaunchArgument('run_plan', default_value='false'),
        RegisterEventHandler(OnProcessExit(target_action=check,
            on_exit=[EmitEvent(event=Shutdown(reason='initialization check finished'))])),
        check])
