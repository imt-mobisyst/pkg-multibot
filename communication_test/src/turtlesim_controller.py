#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from os import getenv

import numpy as np

from communication_test_interfaces.msg import DistanceToTarget
from geometry_msgs.msg import PoseStamped, Twist
from turtlesim.msg import Pose
from std_msgs.msg import Int8
from visualization_msgs.msg import Marker

class TurtleFollow(Node):

    def __init__(self):
        super().__init__('turtle_follow')

        # Declare ROS parameters
        self.declare_parameter('robot_id', 1)

        # Init subscriptions
        self.create_subscription(PoseStamped, '/goal_pose', self.target_callback, 10)
        self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.create_subscription(Int8, '/assignedRobot', self.assignedRobot_callback, 10)

        # Init publishers
        self.distanceToTargetPublisher = self.create_publisher(DistanceToTarget, '/distanceToTarget', 10)
        self.velPublisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.markerPublisher = self.create_publisher(Marker, '/turtle_marker', 10)

        # Init loop
        self.create_timer(1/60.0, self.loop)

        # Init variables
        self.pose = Pose()
        self.pose.x = 5.44
        self.pose.y = 5.44
        
        self.speedFactor = 0.2

        self.distanceTolerance = 0.01

        self.targetPos = None

        self.queue = []

    def paramInt(self, name):
        return self.get_parameter(name).get_parameter_value().integer_value


    def get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]


    def pose_callback(self, msg:Pose):
        self.pose = msg

    def publishPoseMarker(self):
        # Publish marker for vizualisation in rviz
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = self.get_clock().now().to_msg()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 0
        marker.ns = "robot" + str(self.paramInt('robot_id'))
        marker.id = self.paramInt('robot_id')

        # Set the scale of the marker
        marker.scale.x = 1.0
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set the pose of the marker
        q = self.get_quaternion_from_euler(0, 0, self.pose.theta)
        marker.pose.position.x = self.pose.x
        marker.pose.position.y = self.pose.y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        self.markerPublisher.publish(marker)

    def target_callback(self, msg:PoseStamped):
        # Save position        
        self.targetPos = msg.pose.position

        # Calculate cost (total distance) to go to that position
        initPose = self.pose if len(self.queue) == 0 else self.queue[-1]
        totalDistance = self.totalQueueDistance() + self.euclidean_distance(self.targetPos, initPose)

        # Send it to the operator to see which turtle gets assigned to it
        res = DistanceToTarget()
        res.robot_id = self.paramInt('robot_id') # Send the domain ID that the robot is currently in
        res.distance = totalDistance

        self.distanceToTargetPublisher.publish(res)

    def assignedRobot_callback(self, msg:Int8):
        assignedRobotId = int(msg.data)
        print(f"msg assigned robot {msg}")

        if(assignedRobotId == self.paramInt('robot_id')):# If the robot assigned is this one, tell it to move
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
        # Publish pose marker every 0.01s
        self.publishPoseMarker()

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