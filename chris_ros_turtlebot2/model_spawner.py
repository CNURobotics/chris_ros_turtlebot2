"""
Based on spawn_turtlebot.py
https://zmk5.github.io/general/demo/2019/07/15/ros2-spawning-entity.html


Script used to spawn a single URDf model in a generic position
"""
import csv
import os
import sys
import rclpy
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from gazebo_msgs.srv import SpawnEntity
import xacro

def make_spawn_request(name, urdf_file_path, robot_namespace, pose_args):
    try:
        if "xacro" in urdf_file_path:
            description_config = xacro.process_file(urdf_file_path)
            urdf_xml = description_config.toxml()
        else:
            urdf_xml = open(urdf_file_path, 'r').read()

        if isinstance(urdf_xml, str) and len(urdf_xml) > 1:
            request = SpawnEntity.Request()
            request.name = name
            request.xml = urdf_xml
            request.robot_namespace = robot_namespace
            request.initial_pose.position.x = pose_args[0]
            request.initial_pose.position.y = pose_args[1]
            request.initial_pose.position.z = pose_args[2]
            if len(pose_args) > 6:
                request.initial_pose.orientation.x = pose_args[3]
                request.initial_pose.orientation.y = pose_args[4]
                request.initial_pose.orientation.z = pose_args[5]
                request.initial_pose.orientation.w = pose_args[6]
            return request
        else:
            print(f"Invalid XML for {name}\n {e}")

    except Exception as e:
        print(f"Failed to generate request for {name}\n {e}")
        print(">", pose_args)
        pass

    return None

def main():

    """ Main for spawning URDF model node """

    argv = sys.argv[1:]  # skip node name
    if "-h" in argv or "--help" in argv:
        print("Usage of model_spawner node:")
        print("   No required arguments - uses parameters")
        sys.exit(-1)

    # Start model_spawner node
    rclpy.init()

    node = rclpy.create_node("model_spawner")
    node.declare_parameter('model_name', "")
    node.declare_parameter('model_namespace', "")
    node.declare_parameter('model_urdf_file', "")
    node.declare_parameter('model_pose', "")


    # NOTE: This is simple csv string x,y,z or x,y,z,qx,qy,qz,qw values
    #    and not a pose message format
    model_pose = node.get_parameter('model_pose').get_parameter_value().string_value
    if len(model_pose) == 0:
        model_pose = None

    model_name = node.get_parameter('model_name').get_parameter_value().string_value
    model_urdf_file = node.get_parameter('model_urdf_file').get_parameter_value().string_value
    model_namespace = node.get_parameter('model_namespace').get_parameter_value().string_value

    print(f"Request model spawn from parameters:")
    pose_args = [float(val.strip()) for val in model_pose.split(",")]
    print(f"    name: >{model_name}<")
    print(f"    namespace: >{model_namespace}<")
    print(f"    pose: >{pose_args}<")
    print(f"    urdf: >{model_urdf_file}<")

    request = make_spawn_request(model_name,
                                 model_urdf_file,
                                 model_namespace,
                                 pose_args)

    if not request:
        print("No valid model to spawn!")
        print(request)

    else:

        node.get_logger().info(
            'Creating Service client to connect to `/spawn_entity`')
        client = node.create_client(SpawnEntity, "/spawn_entity")

        node.get_logger().info("Connecting to `/spawn_entity` service...")
        if not client.service_is_ready():
            client.wait_for_service()
            node.get_logger().info("...connected!")

        name = request.name
        try:
            assert isinstance(request, SpawnEntity.Request), f"Not Request type ({type(request)})"
            node.get_logger().info(f"Sending service request for {request.name} to `/spawn_entity` service ...")
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future)
            if future.result() is not None:
                result = future.result()
                if not result.success:
                    print(f"    Failed to spawn model {name} : {result.status_message}")
                else:
                    print(f"    Successfully spawned {name}!")
            else:
                raise RuntimeError(
                    'Null result when calling service: %r' % future.exception())
        except Exception as e:
            print(f"  Exception: {type(e)}, {e}")
            print(f"    Request for {name}: ({request})")

        node.get_logger().info("Done! Shutting down spawner node.")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


if __name__ == "__main__":
    main()
