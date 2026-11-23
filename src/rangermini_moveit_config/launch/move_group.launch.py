"""Planning only; no controller connection or execution capability."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    root=Path(share('rangermini_moveit_config'))/'config'
    robot=(Path(share('rangermini_description'))/'urdf/rangermini_xarm6.urdf').read_text()
    def load(name):return yaml.safe_load((root/name).read_text())
    params={'robot_description':ParameterValue(robot,value_type=str),
            'robot_description_semantic':ParameterValue((root/'rangermini_xarm6.srdf').read_text(),value_type=str),
            'robot_description_kinematics':load('kinematics.yaml'),
            'robot_description_planning':load('joint_limits.yaml'),
            'planning_pipelines':['ompl'], 'default_planning_pipeline':'ompl',
            'ompl':load('ompl_planning.yaml'), 'allow_trajectory_execution':False,
            'publish_robot_description_semantic':True,
            'disable_capabilities':'move_group/MoveGroupExecuteTrajectoryAction move_group/MoveGroupMoveAction'}
    return LaunchDescription([
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':params['robot_description']}]),
        Node(package='moveit_ros_move_group',executable='move_group',parameters=[params],output='screen')])
