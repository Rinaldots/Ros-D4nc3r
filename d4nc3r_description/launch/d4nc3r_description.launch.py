import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro

def generate_launch_description():
    
    namespace_arg = DeclareLaunchArgument('namespace', default_value='d4nc3r1', description='Default namespace')
    namespace = LaunchConfiguration('namespace')
    # Obtains d4nc3r_description's share directory path.
    pkg_d4nc3r_description = get_package_share_directory('d4nc3r_description')
    # Obtain urdf from xacro files using a default value for robot_name_arg.
    from launch.substitutions import Command, PathJoinSubstitution
    from launch_ros.substitutions import FindPackageShare

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('d4nc3r_description'),
            'urdf',
            'd4nc3r.urdf.xacro'
        ]),
        ' yaml_config_dir:=', os.path.join(pkg_d4nc3r_description, 'config', 'd4nc3r'),
        ' robot_name_arg:=', namespace
    ])
    params = {
        'robot_description': ParameterValue(robot_description_content, value_type=str),
        'publish_frequency': 5.0,
        'prefix': namespace,
    }

    # Robot state publisher
    rsp = Node(package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=namespace,
                output='screen',
                parameters=[params],
                remappings={('/robot_description', 'robot_description'),}
    )
    return LaunchDescription([
        
        namespace_arg,
        rsp,
    ])
