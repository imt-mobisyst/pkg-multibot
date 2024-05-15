#!/usr/bin/python3
from os import getenv
import rclpy
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point
from turtlesim.msg import Pose

from include.robot_mvt import RobotMovement

class TurtleMovement(RobotMovement):

    def __init__(self):
        super().__init__('turtle_mvt', vel_topic='turtle1/cmd_vel')

        # Log DDS server
        if(getenv('ROS_DISCOVERY_SERVER') is not None):
            self.get_logger().info("RUNNING on DDS \"" + getenv('ROS_DISCOVERY_SERVER') + "\"")

        # Init subscriptions
        self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)

        # Init variables
        self.pose.x = 5.544444561004639
        self.pose.y = 5.544444561004639

        self.targetPos = None



    def pose_callback(self, msg:Pose):
        self.pose = msg

    def getRobotPosition(self) -> Point:
        p = Point()
        p.x = self.pose.x
        p.y = self.pose.y
        p.z = 0.0
        return p

    def getRobotAngle(self) -> float:
        return self.pose.theta







def main(args=None):
    rclpy.init(args=args)
    node = TurtleMovement()
    executor = MultiThreadedExecutor() # Multi threads to run the service server in parallel with the pose callback
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()