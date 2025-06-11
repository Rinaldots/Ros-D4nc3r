from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
# If you need to load parameters from a YAML file, you might need these:
# import os
# from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # O argumento 'namespace' (default 'turtlebot') parece ser para fontes de dados externas.
    # O 'output_namespace' (default 'd4nc3r1') é o que queremos para o nosso robô.
    # Se 'dgb_bb_markers' vier de um namespace 'turtlebot', o remapping precisaria disso.
    # Por agora, vamos focar em namespacing o yolo_to_twist_node e seus outputs.

    # namespace_arg_for_input = LaunchConfiguration('namespace_for_input')
    # namespace_for_input_args = DeclareLaunchArgument(
    #     'namespace_for_input',
    #     default_value='turtlebot', # Ou deixe vazio se dgb_bb_markers for global
    #     description='Namespace for the input marker topic if applicable'
    # )

    output_namespace = LaunchConfiguration('output_namespace')
    output_namespace_args = DeclareLaunchArgument(
        'output_namespace',
        default_value='d4nc3r1',
        description='Namespace for the d4nc3r robot and its outputs'
    )

    
    yolo_to_twist_cpp_node = Node(
        package='d4nc3r_handler',  # Or your_package_name_for_yolo_to_twist
        executable='yolo_to_twist_node', # The name of your C++ executable
        name='yolo_to_twist_node',
        namespace=output_namespace, # Executa o nó no namespace do robô
        output='screen',
        parameters=[{
            'child_frame_id': 'base_link',
            'header_frame_id': 'odom' # Assumindo que o frame odom também é namespaced
        }],
        remappings=[
            ('marker', 'dgb_bb_markers'), # Assumindo que 'dgb_bb_markers' é um tópico global ou seu namespace é tratado em outro lugar
            # O tópico 'yolo_cords' será automaticamente '/<output_namespace>/yolo_cords' devido ao namespace do nó
        ],
    )
    
    return LaunchDescription([
        # namespace_for_input_args,
        output_namespace_args,
        yolo_to_twist_cpp_node,
    ])