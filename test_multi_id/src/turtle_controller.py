#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from os import getenv

import numpy as np

from test_multi_id_interfaces.msg import DistanceToTarget
from geometry_msgs.msg import PointStamped, Twist
from turtlesim.msg import Pose
from std_msgs.msg import Int8

class TurtleFollow(Node):

    def __init__(self):
        super().__init__('turtle_follow')

        self.ROS_DOMAIN_ID = int(getenv('ROS_DOMAIN_ID'))

        # Init subscriptions
        self.create_subscription(PointStamped, '/target', self.target_callback, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.create_subscription(Int8, '/assignedRobot', self.assignedRobot_callback, 10)

        # Init publishers
        self.distanceToTargetPublisher = self.create_publisher(DistanceToTarget, '/distanceToTarget', 10)
        self.velPublisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Init loop
        self.create_timer(0.01, self.loop)

        # Init variables

        self.pose = Pose()
        self.pose.x = 5.44
        self.pose.y = 5.44

        self.distanceTolerance = 0.01

        self.targetPos = None
        self.goToTarget = False

    def pose_callback(self, msg:Pose):
        self.pose = msg

    def target_callback(self, msg:PointStamped):
        if(self.goToTarget and self.targetPos is None):
            return
        
        self.targetPos = msg.point

        distanceToPoint = self.euclidean_distance(self.targetPos)

        res = DistanceToTarget()
        res.robot_id = self.ROS_DOMAIN_ID # Send the domain ID that the robot is currently in
        res.distance = distanceToPoint

        self.distanceToTargetPublisher.publish(res)

    def assignedRobot_callback(self, msg:Int8):
        assignedRobotId = int(msg.data)
        print(f"msg assigned robot {msg}")

        if(assignedRobotId == self.ROS_DOMAIN_ID):# If the robot assigned is this one, tell it to move
            self.goToTarget = True
        else: # If the robot assigned is not this one, reset
            self.targetPos = None


    def euclidean_distance(self, targetPos):
        """Euclidean distance between current pose and the goal."""
        return np.sqrt(np.square(targetPos.y - self.pose.y) + np.square(targetPos.x - self.pose.x))

    
    def linear_vel(self, targetPos, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(targetPos)   
    
    def steering_angle(self, targetPos):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return np.arctan2(targetPos.y - self.pose.y, targetPos.x - self.pose.x)
    
    def angular_vel(self, goal_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)


    def loop(self):
        if self.targetPos is None or self.goToTarget == False:
            # self.velPublisher.publish(Twist()) # Stop moving
            return
        

        vel_msg = Twist()

        if self.euclidean_distance(self.targetPos) >= self.distanceTolerance:

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(self.targetPos)
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = self.angular_vel(self.targetPos)

        else:
            # If reached point, reset variables
            self.targetPos = None
            self.goToTarget = False


        self.velPublisher.publish(vel_msg)






def main(args=None):
    rclpy.init(args=args)
    node = TurtleFollow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()