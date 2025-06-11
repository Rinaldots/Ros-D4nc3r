# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    namespace = LaunchConfiguration("namespace")
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="d4nc3r1",
            description="namespace for the robot and its controllers.",
        )
    )
    

      # Get URDF via xacro
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("d4nc3r_description"), "urdf", "d4nc3r.urdf.xacro"]
            ),
        ]),
        value_type=str
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("d4nc3r_control"),
            "config",
            "d4nc3r_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            ("/diffbot_base_controller/cmd_vel", (namespace,"/cmd_vel")),
            ("/joint_states", (namespace,"/joint_states")),
            ("/diffbot_base_controller/odom", (namespace,"/odom_unfiltered")),
            ("/robot_description", (namespace,"/robot_description")),
            ("/tf", (namespace,"/tf")),
            ("/tf_static", (namespace,"/tf_static")),
            ("/joint_states", (namespace,"/joint_states")),
            ("/diffbot_base_controller/transition_event", (namespace,"/transition_event")),
            ("/dynamic_joint_states", (namespace,"/dynamic_joint_states")),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "controller_manager"],  # removido /
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "controller_manager"],  # removido /
    
    )

    # Delay rviz start after `joint_state_broadcaster`
    

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    nodes = [
        control_node,
        #robot_state_pub_node,
        robot_controller_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
    ]

    # Return a flat list of launch actions
    return LaunchDescription(declared_arguments + nodes)
