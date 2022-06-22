# Copyright 2022, CHRISLab, Christopher Newport University
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
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import yaml

def configure_hokuyo(context):
    urg_node_dir = get_package_share_directory('urg_node')
    param_file = os.path.join(urg_node_dir, 'launch',
        'urg_node_' + context.launch_configurations['sensor_interface'] + '.yaml')
    if os.path.exists(param_file):
        return [SetLaunchConfiguration('param', param_file)]

def generate_launch_description():

    turtlebot_desc_dir = get_package_share_directory('chris_ros_turtlebot2')

    xacro_file = os.path.join(turtlebot_desc_dir,
                              'robots',
                              'kobuki_hexagons_astra_hokuyo.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true') # Only for gazebo launch
    model_pose = LaunchConfiguration('model_pose', default='0.0, 0.0, 0.04')
    model_name = LaunchConfiguration('model_name', default='turtlebot')
    model_namespace = LaunchConfiguration('model_namespace', default='')
    model_urdf_file = LaunchConfiguration('model_urdf_file', default=xacro_file)

    print("Launching Turtlebot2 simulation ... ")

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml(),
              'use_sim_time': use_sim_time}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Gazebo lib publishes wheel joint states and odom msg & tf via libgazebo plugins
    # specified in URDF
    model_spawner = Node(package='chris_ros_turtlebot2',
                         executable='model_spawner',
                         parameters=[{'model_name':      ParameterValue(model_name,      value_type=str),
                                      'model_namespace': ParameterValue(model_namespace, value_type=str),
                                      'model_urdf_file': ParameterValue(model_urdf_file, value_type=str),
                                      'model_pose':      ParameterValue(model_pose,      value_type=str)}],
                         output='screen')

    # Confiugre laser filter for Hokuyo LiDAR
    filters_file = os.path.join(turtlebot_desc_dir, 'param', 'laser_filters.yaml')
    with open(filters_file, 'r') as f:
        filter_params = yaml.safe_load(f)['scan_to_scan_filter_chain']['ros__parameters']
        filter_params['use_sim_time'] = use_sim_time

    laser_filters = Node(package="laser_filters", executable="scan_to_scan_filter_chain",
        output="screen", name="laser_filter",
        remappings=[("scan", 'hokuyo_node/scan_raw'), ("scan_filtered", "hokuyo_node/scan")],
        parameters=[filter_params]
    )

    image_flip = Node(package="image_flip", executable="image_flip_node",
        output="screen", name="camera_flip",
        #remappings=[("image",         '/camera/image_raw'),
        #            ('rotated_image', '/camera_rotated/image_rotated')],
        parameters=[{'use_sim_time': use_sim_time,
                     'rotation_steps': 2, # 2 = 180 degrees
                     # Foxy does not have resolve_topic_name, so use parameters instead
                     'in_image_topic_name': ParameterValue('/camera/image_raw', value_type=str),
                     'out_image_topic_name': ParameterValue('/camera_rotated/image_raw', value_type=str)
                     }]
    )

    # Add all the nodes and then launch
    launch_description = LaunchDescription([
        node_robot_state_publisher,
        model_spawner,
        laser_filters,
        image_flip
        ])

    return launch_description
