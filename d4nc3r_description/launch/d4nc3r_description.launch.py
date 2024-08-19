import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    # Arguments
    rsp_argument = DeclareLaunchArgument('rsp', default_value='true',
                          description='Run robot state publisher node.')

    # Obtains d4nc3r_description's share directory path.
    pkg_d4nc3r_description = get_package_share_directory('d4nc3r_description')

    # Obtain urdf from xacro files.
    arguments = {'yaml_config_dir': os.path.join(pkg_d4nc3r_description, 'config', 'd4nc3r')}
    doc = xacro.process_file(os.path.join(pkg_d4nc3r_description, 'urdf', 'd4nc3r.urdf'), mappings = arguments)
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc,
              'publish_frequency': 30.0,
              'use_sim_time': use_sim_time}

    # Robot state publisher
    rsp = Node(package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace='',
                output='screen',
                parameters=[params],
                condition=IfCondition(LaunchConfiguration('rsp'))
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        rsp_argument,
        rsp,
    ])
