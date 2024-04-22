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
        
        self.speedFactor = 0.2

        self.distanceTolerance = 0.01

        self.targetPos = None

        self.queue = []



    def pose_callback(self, msg:Pose):
        self.pose = msg

    def target_callback(self, msg:PointStamped):
        # Save position        
        self.targetPos = msg.point

        # Calculate cost (total distance) to go to that position
        initPose = self.pose if len(self.queue) == 0 else self.queue[-1]
        totalDistance = self.totalQueueDistance() + self.euclidean_distance(self.targetPos, initPose)

        # Send it to the operator to see which turtle gets assigned to it
        res = DistanceToTarget()
        res.robot_id = self.ROS_DOMAIN_ID # Send the domain ID that the robot is currently in
        res.distance = totalDistance

        self.distanceToTargetPublisher.publish(res)

    def assignedRobot_callback(self, msg:Int8):
        assignedRobotId = int(msg.data)
        print(f"msg assigned robot {msg}")

        if(assignedRobotId == self.ROS_DOMAIN_ID):# If the robot assigned is this one, tell it to move
            self.queue.append(self.targetPos)
        
        # If the robot assigned is not this one OR the position has been added to queue => reset
        self.targetPos = None


    def totalQueueDistance(self):
        if len(self.queue) == 0:
            return 0
        
        dist = 0
        positions = [self.pose] + self.queue
        for i in range(len(positions)-1):
            dist += self.euclidean_distance(positions[i+1], positions[i])

        return dist
            


    def euclidean_distance(self, targetPos, initPose=None):
        """Euclidean distance between current pose and the goal."""
        if initPose is None:
            initPose = self.pose

        return np.sqrt(np.square(targetPos.y - initPose.y) + np.square(targetPos.x - initPose.x))

    
    def linear_vel(self, targetPos, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(targetPos)   
    
    def steering_angle(self, targetPos):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return np.arctan2(targetPos.y - self.pose.y, targetPos.x - self.pose.x)
    
    def angular_vel(self, goal_pose, constant=10):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)


    def loop(self):
        if len(self.queue) == 0:
            # self.velPublisher.publish(Twist()) # Stop moving
            return
        

        vel_msg = Twist()

        # Go to first pose in the queue
        if self.euclidean_distance(self.queue[0]) >= self.distanceTolerance:

            # Linear velocity in the x-axis.
            # vel_msg.linear.x = self.linear_vel(self.queue[0]) * self.speedFactor
            vel_msg.linear.x = 4.0 * self.speedFactor
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = self.angular_vel(self.queue[0])

        else:
            # If reached point, reset variables
            self.queue.pop(0) # Remove first pos from the queue


        self.velPublisher.publish(vel_msg)






def main(args=None):
    rclpy.init(args=args)
    node = TurtleFollow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()