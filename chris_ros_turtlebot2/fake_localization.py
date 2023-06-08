import math
import numpy as np 

import message_filters 

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry 

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class FakeLocalization(Node):

    def __init__(self):
        super().__init__('fake_localization')


        self.ground_truth_sub = message_filters.Subscriber(self, Odometry, 'ground_truth')
        self.ground_truth_cache = message_filters.Cache(self.ground_truth_sub, 10) 
        self.ground_truth_cache.registerCallback(self.ground_truth_cb)

        self._last_truth = Odometry() 
 
        # Declare and acquire parameters
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').get_parameter_value().string_value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').get_parameter_value().string_value
        self.global_frame = self.declare_parameter('map_frame', 'map').get_parameter_value().string_value
        self.tolerance = self.declare_parameter('transform_tolerance', 0.100).get_parameter_value().double_value
        self.timer_period_s = self.declare_parameter('timer_period_s', 0.067).get_parameter_value().double_value

        assert self.tolerance > self.timer_period_s, "Must have larger transform tolerance than update period!"

        # @todo - make this world to map transform configurable
        self.t_mw = np.eye(4)  # Transform from world frame to map frame 

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_caster  = TransformBroadcaster(self)

        self.update_timer = self.create_timer(self.timer_period_s, self.update_cb)

        # Create turtle2 velocity publisher
        self.map_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'amcl_pose', 1)

    @staticmethod
    def _make_transform(posn, quat): 
        """
        Given translation and orientation in normalized quaternion,
        return a 4D homogeneous transform matrix
        https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm

        Make use of Python duck typing to work with several message forms
        """
        t = np.zeros((4, 4))

        # Rotation matrix (by column)
        t[0][0] = 1-2*quat.y*quat.y - 2*quat.z*quat.z
        t[1][0] = 2.0*(quat.x*quat.y + quat.z*quat.w)
        t[2][0] = 2.0*(quat.x*quat.z - quat.y*quat.w)

        t[0][1] = 2.0*(quat.x*quat.y - quat.z*quat.w)
        t[1][1] = 1-2*quat.x*quat.x - 2*quat.z*quat.z
        t[2][1] = 2.0*(quat.y*quat.z + quat.x*quat.w)

        t[0][2] = 2.0*(quat.x*quat.z + quat.y*quat.w)
        t[1][2] = 2.0*(quat.y*quat.z - quat.x*quat.w)
        t[2][2] = 1-2*quat.x*quat.x - 2*quat.y*quat.y

        # Translation
        t[0][3] = posn.x
        t[1][3] = posn.y
        t[2][3] = posn.z
        t[3][3] = 1.0 

        return t

    @staticmethod
    def _get_pose(transform):
        """
        Given 4x4 homogeneous transform, return position and quaternion
        """
        posn = Point(x=transform[0][3], y=transform[1][3], z=transform[2][3])

        m00 = transform[0,0]
        m01 = transform[0,1]
        m02 = transform[0,2]
        m10 = transform[1,0]
        m11 = transform[1,1]
        m12 = transform[1,2]
        m20 = transform[2,0]
        m21 = transform[2,1]
        m22 = transform[2,2]

        qw = np.sqrt(1 + m00 + m11 + m22)/2.0
        qx = (m21 - m12)/(4*qw)
        qy = (m02 - m20)/(4*qw)
        qz = (m10 - m01)/(4*qw)

        return posn, Quaternion(x=qx, y=qy, z=qz, w=qw)  
    
    def ground_truth_cb(self, msg):
        # Assuming that GT is more frequent than odom
        self.get_logger().debug(f'Truth : {msg.header.stamp} {msg.pose.pose.position} {msg.pose.pose.orientation}')


    def update_cb(self):

        try:
            # Find latest inverse transform of odom frame in base frame
            tbo = self.tf_buffer.lookup_transform(
                    self.base_frame,  # to frame
                    self.odom_frame,  # from frame
                    rclpy.time.Time()) # Get the latest transform available
        except TransformException as ex:

            self.get_logger().error(
                    f'\n\nCould not transform {self.odom_frame} to {self.base_frame}: {ex}')
            return 


        odom_time = rclpy.time.Time.from_msg(tbo.header.stamp)
        prior_truth = self.ground_truth_cache.getElemBeforeTime(odom_time)
        after_truth = self.ground_truth_cache.getElemAfterTime(odom_time)

        if prior_truth is None:
            self.get_logger().error( 
                f'\n\nCould not transform {self.odom_frame} to {self.base_frame} at {tbo.header.stamp}\n {prior_truth}\n {after_truth}')
            return 

        if after_truth is None:
            # This is the best that we have
            self._last_truth = prior_truth 
        else:
            dtp = odom_time - rclpy.time.Time.from_msg(prior_truth.header.stamp)
            dta = rclpy.time.Time.from_msg(after_truth.header.stamp)  - odom_time  

            # For now, just  use the closest point in time instead of interpolating
            if dtp < dta:
                self._last_truth = prior_truth 
            else:
                self._last_truth = after_truth 

        self.get_logger().debug(
            f'Processing ground truth transforms at {tbo.header.stamp} and {self._last_truth.header.stamp}')
        t_bo = self._make_transform(tbo.transform.translation, tbo.transform.rotation) 
        t_wb = self._make_transform(self._last_truth.pose.pose.position, self._last_truth.pose.pose.orientation)
 
        t_mb = np.dot(self.t_mw, t_wb) # base in map frame 

        t_mo = np.dot(t_mb, t_bo) # odom frame in map frame

        trans, quat = self._get_pose(t_mo) 

        # Broadcast the transform from odom to map (as AMCL convention)
        t = TransformStamped()
        t.header.stamp = (rclpy.time.Time.from_msg(tbo.header.stamp) + rclpy.time.Duration(seconds=self.tolerance)).to_msg()
        t.header.frame_id = self.global_frame
        t.child_frame_id = self.odom_frame 

        t.transform.translation.x = trans.x
        t.transform.translation.y = trans.y
        t.transform.translation.z = trans.z

        t.transform.rotation.x = quat.x
        t.transform.rotation.y = quat.y
        t.transform.rotation.z = quat.z
        t.transform.rotation.w = quat.w
        self.tf_caster.sendTransform(t)

        # Robot's estimated pose in the map, with covariance. 
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = tbo.header.stamp 
        pose.header.frame_id = self.global_frame 
        pose.pose.pose.position = trans 
        pose.pose.pose.orientation = quat 
        self.map_pose_pub.publish(pose) 

def main(args=None):
    rclpy.init(args=args)
    node = FakeLocalization()
    try:
        node.get_logger().info(f'{node.get_name()} - begin processing transforms ...')
 
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("Keyboard interrupt!")
        pass

    print(f"Shut down fake localization!")
