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
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import yaml

def generate_launch_description():
    turtlebot_desc_dir = get_package_share_directory('chris_ros_turtlebot2')

    cam_param_path = os.path.join(
        turtlebot_desc_dir,
        'param',
        'astra.yaml'
    )
    with open(cam_param_path, 'rt') as fin:
        astra_params = yaml.safe_load(fin)['astra_camera']['ros__parameters']

    namespace = '/camera'

    astra_cam = launch_ros.actions.ComposableNodeContainer(
            name='container',
            namespace=namespace,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Just the driver
                launch_ros.descriptions.ComposableNode(
                    package='openni2_camera',
                    plugin='openni2_wrapper::OpenNI2Driver',
                    name='astra_camera',
                    parameters=[astra_params],
                    namespace=namespace,
                ),
            ],
            output='screen',
    )


    # Add all the nodes and then launch
    launch_description = LaunchDescription()
    launch_description.add_action(astra_cam)

    return launch_description
