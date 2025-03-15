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
#import xacro
import yaml


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, TimerAction, SetLaunchConfiguration
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource



def configure_hokuyo(context):
    urg_node_dir = get_package_share_directory('urg_node')
    param_file = os.path.join(urg_node_dir, 'launch',
        'urg_node_' + context.launch_configurations['sensor_interface'] + '.yaml')
    if os.path.exists(param_file):
        return [SetLaunchConfiguration('param', param_file)]


def generate_launch_description():

    turtlebot_pkg = 'chris_ros_turtlebot2'
    turtlebot_desc_dir = get_package_share_directory(turtlebot_pkg)
    declared_arguments = [
        DeclareLaunchArgument('x', default_value='0.0', description='Initial x position'),
        DeclareLaunchArgument('y', default_value='0.0', description='Initial y position'),
        DeclareLaunchArgument('z', default_value='0.04', description='Initial z position'),
        DeclareLaunchArgument('roll', default_value='0.0', description='Initial roll'),
        DeclareLaunchArgument('pitch', default_value='0.0', description='Initial pitch'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Initial yaw'),
        DeclareLaunchArgument('tbot_name', default_value='tbot0', description='Robot name'),
        DeclareLaunchArgument('use_rviz',  default_value='false',  description="Whether to start RViz."),
        DeclareLaunchArgument('model_urdf_file',
                              default_value='kobuki_hexagons_realsense_hokuyo.urdf.xacro',
                              description="Robot model with sensors."),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')  # Only for gazebo launch
    model_urdf_file = LaunchConfiguration('model_urdf_file')
    tbot_name = LaunchConfiguration('tbot_name', default='tbot0')

    print(f"Launching Turtlebot2 simulation from '{turtlebot_desc_dir}' ... ",flush=True)


    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(turtlebot_pkg), "robots", model_urdf_file]  # include base frame
            ),
            " ",
            "robot_name:='",tbot_name,"' ",
            "use_mock_hardware:=false ",
            "mock_sensor_commands:=false ",
            "sim_gazebo:=true ",
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(turtlebot_pkg), "param", "turtlebot.rviz"]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
    )
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/camera/depth/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/depth/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/hokuyo_node/scan_raw@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/hokuyo_node/scan_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            ["/model/", tbot_name, "/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry"],
       ],
       remappings=[
           (["/model/", tbot_name, "/odometry"], [tbot_name, "/ground_truth"])
       ],
        output="screen",
    )

    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]
    #     ),
    #     launch_arguments=[("gz_args", ["-r ", PathJoinSubstitution(
    #         [FindPackageShare(turtlebot_pkg), "robots", "chrislab.sdf"]
    #     )])],
    # )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot",
        arguments=["-topic",
                   "robot_description",
                   "-name", tbot_name,
                   "-x", LaunchConfiguration('x'),
                   "-y", LaunchConfiguration('y'),
                   "-z", LaunchConfiguration('z'),
                   "-R", LaunchConfiguration('roll'),
                   "-P", LaunchConfiguration('pitch'),
                   "-Y", LaunchConfiguration('yaw'),
                   ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

   # Delay loading and activation of `joint_state_broadcaster` after start of ros2_control_node
    delay_joint_state_broadcaster_spawner_after_ros2_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_entity,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[joint_state_broadcaster_spawner],
                ),
            ],
        )
    )

    robot_controller_names = ['tbot0']
    robot_controller_spawners = []
    for controller in robot_controller_names:
        robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"],
            )
        ]

    inactive_robot_controller_names = []
    inactive_robot_controller_spawners = []
    for controller in inactive_robot_controller_names:
        inactive_robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager", "--inactive"],
            )
        ]

    # Delay loading and activation of robot_controller_names after `joint_state_broadcaster`
    delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for i, controller in enumerate(robot_controller_spawners):
        delay_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=robot_controller_spawners[i - 1]
                    if i > 0
                    else joint_state_broadcaster_spawner,
                    on_exit=[controller],
                )
            )
        ]

    # Delay start of inactive_robot_controller_names after other controllers
    delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for i, controller in enumerate(inactive_robot_controller_spawners):
        delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=inactive_robot_controller_spawners[i - 1]
                    if i > 0
                    else robot_controller_spawners[-1],
                    on_exit=[controller],
                )
            )
        ]


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

    relay_odom = Node(
        name="relay_odom",
        package="topic_tools",
        executable="relay",
        arguments=[PathJoinSubstitution(["/", tbot_name, "odom"]),
                   "/odom",
        ],
        output="screen",
    )

    relay_cmd_vel = Node(
        name="relay_cmd_vel",
        package="topic_tools",
        executable="relay",
        arguments=["/cmd_vel",
                   PathJoinSubstitution(["/", tbot_name, "cmd_vel"]),
        ],
        output="screen",
    )

    return LaunchDescription(
        declared_arguments +
        [   robot_state_pub_node,
            rviz_node,
            # gazebo,
            spawn_entity,
            gz_bridge_node,
            laser_filters,
            relay_odom,
            relay_cmd_vel,
            delay_joint_state_broadcaster_spawner_after_ros2_control_node,
        ]
        + delay_robot_controller_spawners_after_joint_state_broadcaster_spawner
        + delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner
    )

