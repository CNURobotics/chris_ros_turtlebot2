# Copyright 2022 CHRISLab, Christopher Newport University
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
# Author: David Conner

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')  # For simulations
    autostart = LaunchConfiguration('autostart', default='true')
    namespace = LaunchConfiguration('namespace', default='')

    default_map_file_name = 'creech_map_050.yaml'

    default_map_package_prefix = os.path.join(get_package_share_directory('chris_world_models'), "maps")

    map_yaml_file_path = LaunchConfiguration("map_yaml_file_path",
                                             default=os.path.join(default_map_package_prefix, default_map_file_name))

    localization_name = "amcl"  # Fake the standard package
    ms_name = 'map_server'      # Presumes map server launched earlier

    lifecycle_nodes = [ms_name]

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument('namespace', default_value=namespace,
                              description='Top-level namespace'),

        Node(package='chris_ros_turtlebot2',
             executable='fake_localization',
             name=localization_name,
             output='screen',
             parameters=[],
             remappings=remappings),

        Node(package='nav2_map_server',
             executable='map_server',
             name="map_server",
             output='screen',
             emulate_tty=True,  # https://github.com/ros2/launch/issues/188
             parameters=[{'use_sim_time': use_sim_time,
                          'yaml_filename': map_yaml_file_path}],
             ),

        Node(package='nav2_lifecycle_manager',
             executable='lifecycle_manager',
             name='lifecycle_manager',
             output='screen',
             # emulate_tty=True,  # https://github.com/ros2/launch/issues/188
             parameters=[{'use_sim_time': use_sim_time},
                         {'autostart': autostart},
                         {'node_names': lifecycle_nodes}])
    ])
