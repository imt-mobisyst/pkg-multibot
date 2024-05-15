#!/usr/bin/python3
from os import getenv
import numpy as np, time
import rclpy
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry

from include.robot_mvt import RobotMovement
from include.helpers import getEulerFromQuaternion

from nav2_msgs.action import NavigateToPose
from rclpy.action.server import ServerGoalHandle

class StageRobotMovement(RobotMovement):

    def __init__(self):
        super().__init__('stage_robot_mvt', vel_topic='cmd_vel')

        # Init subscriptions
        self.create_subscription(Odometry, 'ground_truth', self.pose_callback, 10)

        # Init variables
        self.pose = Pose()

        self.speedFactor = 0.1
        self.distanceTolerance = 0.1



    def pose_callback(self, msg:Odometry):
        self.pose = msg.pose.pose

    def getRobotPosition(self) -> Point:
        p = Point()
        p.x = self.pose.position.x
        p.y = self.pose.position.y
        p.z = 0.0
        return p

    def getRobotAngle(self) -> float:
        return getEulerFromQuaternion(self.pose.orientation)['yaw']







def main(args=None):
    rclpy.init(args=args)
    node = StageRobotMovement()
    executor = MultiThreadedExecutor() # Multi threads to run the service server in parallel with the pose callback
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()