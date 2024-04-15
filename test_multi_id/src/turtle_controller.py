#!/usr/bin/python3
import rclpy
from rclpy.node import Node

import numpy as np

from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped, Twist
from turtlesim.msg import Pose

class TurtleFollow(Node):

    def __init__(self):
        super().__init__('turtle_follow')

        # Init subscriptions
        self.create_subscription(PointStamped, '/target', self.target_callback, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Init publishers
        self.distanceToTargetPublisher = self.create_publisher(Float32, '/distanceToTarget', 10)
        self.velPublisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Init loop
        self.create_timer(0.01, self.loop)

        self.pos = np.array([5.544445, 5.544445])

        self.target = None

    def pose_callback(self, msg:Pose):
        self.pos = np.array([msg.x, msg.y])

    def target_callback(self, msg:PointStamped):
        self.target = np.array([msg.point.x, msg.point.y])

        distanceToPoint = np.linalg.norm(self.target - self.pos)

        res = Float32()
        res.data = distanceToPoint

        self.distanceToTargetPublisher.publish(res)


    def loop(self):
        # if self.target is not None:
        #     self.velPublisher.publish(Twist()) # Stop moving
        #     return
        

        # vel = Twist()


        # self.velPublisher.publish(vel)
        pass


        







def main(args=None):
    rclpy.init(args=args)
    node = TurtleFollow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()