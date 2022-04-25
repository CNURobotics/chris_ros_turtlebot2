import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml

def generate_launch_description():
    turtlebot_desc_dir = get_package_share_directory('chris_ros_turtlebot2')

    xacro_file = os.path.join(turtlebot_desc_dir,
                              'robots',
                              'kobuki_hexagons_astra_hokuyo.urdf.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Configure Kobuki base node
    params_file = os.path.join(turtlebot_desc_dir, 'param', 'kobuki_node_params.yaml')
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']
    kobuki_ros_node = Node(package='kobuki_node',
                           executable='kobuki_ros_node',
                           output='screen',
                           parameters=[params],
                           remappings=[('commands/velocity', 'cmd_vel')])

    # Configure Hokuyo LiDAR node
    param_file_path = os.path.join(turtlebot_desc_dir, 'param', 'hokuyo_serial.yaml')
    with open(param_file_path, 'r') as f:
        hokuyo_params = yaml.safe_load(f)['urg_node']['ros__parameters']

    hokuyo_node = Node(package='urg_node', executable='urg_node_driver', output='screen',
                       parameters=[hokuyo_params],
                       remappings=[('scan', 'hokuyo_node/scan_raw')])

    # Confiugre laser filter for Hokuyo LiDAR
    filters_file = os.path.join(turtlebot_desc_dir, 'param', 'laser_filters.yaml')
    with open(filters_file, 'r') as f:
        filter_params = yaml.safe_load(f)['scan_to_scan_filter_chain']['ros__parameters']

    laser_filters = Node(package="laser_filters", executable="scan_to_scan_filter_chain",
        output="screen", name="laser_filter",
        remappings=[("scan", 'hokuyo_node/scan_raw'), ("scan_filtered", "hokuyo_node/scan")],
        parameters=[filter_params]
    )

    ## Static transform between Kobuki base and Hokuyo LiDAR
    #node = Node(package = "tf2_ros",
    #                   executable = "static_transform_publisher",
    #                   arguments = ["0 0 0.3 0 0 0 base_link laser"])

    # Add all the nodes and then launch
    print("Set up launch description")
    launch_description = LaunchDescription()
    launch_description.add_action(node_robot_state_publisher)
    launch_description.add_action(kobuki_ros_node)
    launch_description.add_action(hokuyo_node)
    launch_description.add_action(laser_filters)

    return launch_description
