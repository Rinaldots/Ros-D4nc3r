# BSD 3-Clause License
#
# Copyright (c) 2023, Ekumen Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import (Node)
from launch import LaunchDescription
import rclpy
from rclpy.node import Node
import launch.logging


import sys



def get_number_and_topic_list():
    rclpy.init()  
    
    node_dummy = Node("_ros2cli_dummy_to_show_topic_list")
    
    topic_list = node_dummy.get_topic_names_and_types()
    
    node_dummy.destroy_node()
    
    rclpy.shutdown()
    
    # Filtra e processa a lista de tópicos
    prefixo = '/d4nc3r'
    topic_names = [item[0] for item in topic_list]  # Extraí apenas os nomes dos tópicos
    d4nc3r_topic_list = [item.split(prefixo, 1)[1].split('/')[0] for item in topic_names if item.startswith(prefixo)]
    prefix_topic_list = list(set(d4nc3r_topic_list))  # Remove duplicatas
    
    # Retorna o proximo d4nc3r
    num = 0
    while num in prefix_topic_list:
        num += 1
    return num

def generate_launch_description():
          
    pkg_d4nc3r_gz = get_package_share_directory('d4nc3r_gz')
    next_robot = get_number_and_topic_list()
    print(f'/d4nc3r{next_robot}')
    bringup_cmd_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_d4nc3r_gz, 'launch', 'spawn_robot.launch.py')),
            launch_arguments={'namespace': f'd4nc3r{next_robot}',
                              'entity': f'd4nc3r_{next_robot}'}.items()),
    ])
    
    ld = LaunchDescription()   
    
    ld.add_action(bringup_cmd_group)
    
    
    return ld