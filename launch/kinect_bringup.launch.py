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

    cam_param_path = os.path.join(
        turtlebot_desc_dir,
        'param',
        'kinect.yaml'
    )
    with open(cam_param_path, 'rt') as fin:
        astra_params = yaml.safe_load(fin)['kinect_camera']['ros__parameters']

    namespace = '/camera'
    kinect_camera = launch_ros.actions.ComposableNodeContainer(
            name='container',
            namespace=namespace,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Just the driver
                launch_ros.descriptions.ComposableNode(
                    package='openni2_camera',
                    plugin='openni2_wrapper::OpenNI2Driver',
                    name='kinect_camera',
                    parameters=[astra_params],
                    namespace=namespace,
                ),
                launch_ros.descriptions.ComposableNode(
                    package='image_flip',
                    plugin='image_flip::ImageFlipNode',
                    name='camera_flip',
                    #remappings=[("image",         'camera/image_raw'),
                    #            ('rotated_image', 'camera_rotated/image_rotated')],
                    parameters=[{'rotation_steps': 0, # pass through to preserve standard demo topics
                                 # Foxy does not have resolve_topic_name, so use parameters instead
                                 'in_image_topic_name': ParameterValue('/camera/image_raw', value_type=str),
                                 'out_image_topic_name': ParameterValue('/camera_rotated/image_raw', value_type=str)
                    namespace='',
                ),
            ],
            output='screen',
    )


    # Add all the nodes and then launch
    launch_description = LaunchDescription()
    launch_description.add_action(kinect_camera)

    return launch_description
