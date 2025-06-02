
import os
import launch
import launch_ros
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Arguments
    pkg_share = launch_ros.substitutions.FindPackageShare(package='d4nc3r_description').find('d4nc3r_description')
    rsp_argument = DeclareLaunchArgument('rsp', default_value='true',
                          description='Run robot state publisher node.')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    default_model_path = os.path.join(pkg_share, 'urdf/d4nc3r.urdf.xacro')    
    # Obtains d4nc3r_description's share directory path.
    pkg_d4nc3r_description = get_package_share_directory('d4nc3r_description')
    # Obtain urdf from xacro files.

    arguments = {'yaml_config_dir': os.path.join(pkg_d4nc3r_description, 'config', 'd4nc3r')}
    doc = xacro.process_file(os.path.join(pkg_d4nc3r_description, 'urdf', 'd4nc3r.urdf.xacro'), mappings = arguments)
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc,
              'publish_frequency': 30.0,
              'use_sim_time': use_sim_time}
    
    # Robot state publisher
    rsp = Node(package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace='',
                output='both',
                parameters=[params],
                condition=IfCondition(LaunchConfiguration('rsp'))
    )
    #gazebo

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path],
    )
    
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'sam_bot', '-topic', 'robot_description'],
        output='screen'
    )
   
    

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen'),
        joint_state_publisher_node,
        rsp_argument,
        rsp,
        spawn_entity,
        
                
    ])
