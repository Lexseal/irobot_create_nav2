# Copyright 2019 Open Source Robotics Foundation, Inc.
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
#
# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('irobot_create_nav2'),
            'map',
            'lab_map.yaml'))

    model = "empty_belly"  # choices: create3, empty_belly

    default_model_path = os.path.join(get_package_share_directory('irobot_create_nav2'),
                                      f'urdf/{model}.urdf')

    # param_file_name = str(LaunchConfiguration('params_file', default='create.yaml'))
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('irobot_create_nav2'),
            'param',
            f'{model}.yaml'))

    # need this for the laser to base_link transform
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_model_path])}]
    )

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    rplidar_launch_file_dir = os.path.join(get_package_share_directory("rplidar_ros"), "launch")

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/localization_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rplidar_launch_file_dir, '/rplidar_a1_launch.py'])
        ),

        # launch a node called rs_to_scan inside the irobot_create_nav2 package
        Node(
            package='irobot_create_nav2',
            executable='rs_to_scan.py',
            name='rs_to_scan'
        ),

        robot_state_publisher_node
    ])
