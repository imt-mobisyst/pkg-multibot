#!/usr/bin/python3
import rclpy
import math as m
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Pose
from include.helpers import getYaw, getQuaternionFromEuler


class StageFakeLocalization(Node):

    def __init__(self, name='fake_localization'):
        super().__init__(name)

        # Init subscriber
        self.create_subscription(Odometry, "ground_truth", self.groundTruthCallback, 10)
        self.create_subscription(Odometry, "odom", self.odomCallback, 10)
        
        # Init publisher
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(0.05, self.tf_timer_callback)

        self.lastGlobalPose = Pose()
        self.lastOdomPose = Pose()

    def groundTruthCallback(self, msg:Odometry):
        # Save pose
        self.lastGlobalPose = msg.pose.pose

    def odomCallback(self, msg:Odometry):
        # Save pose
        self.lastOdomPose = msg.pose.pose


    def tf_timer_callback(self):
        msg = TransformStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.child_frame_id = "odom"

        # Calculate map2odom tf based on robot position/rotation in odom and map
        ## Rotation between odom and map
        yaw = getYaw(self.lastGlobalPose.orientation) - getYaw(self.lastOdomPose.orientation)

        ## Translation between odom and map (based on previous rotation)
        msg.transform.translation.x = self.lastGlobalPose.position.x - (self.lastOdomPose.position.x * m.cos(yaw) - self.lastOdomPose.position.y * m.sin(yaw))
        msg.transform.translation.y = self.lastGlobalPose.position.y - (self.lastOdomPose.position.x * m.sin(yaw) + self.lastOdomPose.position.y * m.cos(yaw))
        
        msg.transform.translation.z = self.lastGlobalPose.position.z

        msg.transform.rotation = getQuaternionFromEuler(0, 0, yaw)
        
        self.tf_broadcaster.sendTransform(msg)



def main(args=None):
    rclpy.init(args=args)
    node = StageFakeLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()