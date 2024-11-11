import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    
    # Arguments
    rsp_argument = DeclareLaunchArgument('rsp', default_value='true',
                          description='Run robot state publisher node.')
    namespace_arg = DeclareLaunchArgument('namespace', default_value='d4nc3r1', description='Default namespace')
    namespace = LaunchConfiguration('namespace')
    # Obtains d4nc3r_description's share directory path.
    pkg_d4nc3r_description = get_package_share_directory('d4nc3r_description')

    # Obtain urdf from xacro files.
    arguments = {'yaml_config_dir': os.path.join(pkg_d4nc3r_description, 'config', 'd4nc3r')}
    doc = xacro.process_file(os.path.join(pkg_d4nc3r_description, 'urdf', 'd4nc3r.urdf.xacro'), mappings = arguments)
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc,
              'publish_frequency': 30.0}

    # Robot state publisher
    rsp = Node(package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=namespace,
                output='screen',
                parameters=[params],
                condition=IfCondition(LaunchConfiguration('rsp'))
    )

    return LaunchDescription([
        
        namespace_arg,
        rsp_argument,
        rsp,
    ])
