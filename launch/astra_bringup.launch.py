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

    config = os.path.join(
        turtlebot_desc_dir,
        'param',
        'astra.yaml'
    )
    with open(config, 'rt') as fin:
       params = yaml.safe_load(fin)['astra_camera_node']['ros__parameters'] 

    print(params) 

    # Astra is not updated for ROS 2
    # astra_cam = Node(package="astra_camera",
    #                  executable="astra_camera_node",
    #                  name="astra_camera_node",
    #                  output="screen",
    #                  emulate_tty=True,
    #                  parameters=[config]
    #                 )
    astra_cam = Node(package='usb_cam',
                     executable='usb_cam_node_exe',
                     output='screen',
                     name="astra_camera_node",
                     # namespace=ns,
                     parameters=[params]
                    )

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
