from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'ekf.yaml')],
           ),
])


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    namespace_args = DeclareLaunchArgument(
        'namespace',
        default_value='d4nc3r1',
        description='Namespace for the nodes'
    )
    
    pkg_share = get_package_share_directory('d4nc3r_handler')
    robot_localization_file_path = os.path.join(pkg_share, 'config/ekf.yaml')
    
    ekf_node = Node(
        package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("d4nc3r_handler"), 'config', 'ekf.yaml')],
            remappings=[
                ('odometry/filtered', '/d4nc3r1/odometry/filtered'),
            ],
           )
    
    
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('d4nc3r_description'), 'launch', 'd4nc3r_description.launch.py'
        )]),
        launch_arguments={'namespace': namespace}.items(),
    )
    
    return LaunchDescription([
        namespace_args,
        ekf_node,
        description_launch,
    ])