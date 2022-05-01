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
        'astra.yaml'
    )
    with open(cam_param_path, 'rt') as fin:
        astra_params = yaml.safe_load(fin)['astra_camera']['ros__parameters']

    namespace = '/camera'

    # Based on camera_with_cloud from openni2_camera fork by MikeFerguson
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

                # Create XYZRGB point cloud
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzrgbNode',
                    name='points_xyzrgb',
                    namespace=namespace,
                    parameters=[{'queue_size': 5}],
                    remappings=[('rgb/image_rect_color', 'image_raw'),
                                ('rgb/camera_info', 'camera_info'),
                                ('depth_registered/image_rect', 'depth/image_raw'),
                                ('points', 'depth/points'), ],
                ),

            ],
            output='screen',
    )


    # TODO - Make this a composable nodelet as well !
    image_flip = Node(package="image_flip", executable="image_flip_node",
        output="screen", name="camera_flip",
        remappings=[("image",         'camera/image_raw'),
                    ('rotated_image', 'camera_rotated/image_rotated')],
        parameters=[{'rotation_steps': 2, # 2 = 180 degrees
                     # Foxy does not have resolve_topic_name, so use parameters instead
                     'in_image_topic_name': 'camera/image_raw',
                     'out_image_topic_name': 'camera_rotated/image_raw'}]
    )

    # Add all the nodes and then launch
    launch_description = LaunchDescription()
    launch_description.add_action(astra_cam)
    launch_description.add_action(image_flip)

    return launch_description
