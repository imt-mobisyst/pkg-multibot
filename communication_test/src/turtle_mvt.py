#!/usr/bin/python3
from os import getenv
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

import numpy as np

from geometry_msgs.msg import PoseStamped, Twist, Point
from turtlesim.msg import Pose

from nav2_msgs.action import NavigateToPose

class TurtleMovement(Node):

    def __init__(self):
        super().__init__('turtle_mvt')

        # Log DDS server
        if(getenv('ROS_DISCOVERY_SERVER') is not None):
            self.get_logger().info("RUNNING on DDS \"" + getenv('ROS_DISCOVERY_SERVER') + "\"")

        # Declare ROS parameters
        self.declare_parameter('robot_id', 1)
        self.get_logger().info(f"Robot {self.paramInt('robot_id')} started")

        # Init subscriptions
        self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)

        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'moveToPose',
            self.moveToPoseCallback)

        # Init publishers
        self.velPublisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        
        # Init variables
        self.pose = Pose()
        self.pose.x = 5.544444561004639
        self.pose.y = 5.544444561004639
       
        self.speedFactor = 0.2

        self.distanceTolerance = 0.01

        self.targetPos = None
        self.goal_handle = None

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
    
    def angular_vel(self, goal_pose, constant=50):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        angle = (self.steering_angle(goal_pose) - self.pose.theta)
        return constant * angle


    def createFeedbackMsg(self):
        feedback_msg = NavigateToPose.Feedback()
        feedback_msg.current_pose = PoseStamped()
        feedback_msg.current_pose.pose.position.x = self.pose.x
        feedback_msg.current_pose.pose.position.y = self.pose.y
        feedback_msg.current_pose.pose.position.z = 0.0
        feedback_msg.current_pose.pose.orientation.y = self.pose.y
        q = self.get_quaternion_from_euler(0, 0, self.pose.theta)
        feedback_msg.current_pose.pose.orientation.x = q[0]
        feedback_msg.current_pose.pose.orientation.y = q[1]
        feedback_msg.current_pose.pose.orientation.z = q[2]
        feedback_msg.current_pose.pose.orientation.w = q[3]

        feedback_msg.distance_remaining = self.euclidean_distance(self.targetPos)
        
        # self.get_logger().info('Feedback: {0}'.format(feedback_msg.distance_remaining))

        return feedback_msg

    def moveToPoseCallback(self, goal_handle:ServerGoalHandle):
        self.get_logger().info('Executing goal...')

        request:NavigateToPose = goal_handle.request
        self.targetPos:Point = request.pose.pose.position

        # First rotate approximately to the correct rotation
        while np.abs(self.pose.theta - self.steering_angle(self.targetPos)) >= np.pi / 180.0:
            vel_msg = Twist()

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 4.0 * np.sign(self.angular_vel(self.targetPos)) * self.speedFactor


            # Rotate in the correct direction
            self.velPublisher.publish(vel_msg)


            # Send feedback to action client
            feedback_msg = self.createFeedbackMsg()
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(0.01)

        while self.euclidean_distance(self.targetPos) >= self.distanceTolerance:
            vel_msg = Twist()

            # Linear velocity in the x-axis.
            vel_msg.linear.x = 4.0 * self.speedFactor
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = self.angular_vel(self.targetPos) * self.speedFactor

            # Move in the correct direction
            self.velPublisher.publish(vel_msg)



            # Send feedback to action client
            feedback_msg = self.createFeedbackMsg()
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(0.01)


        # Finish action
        goal_handle.succeed()

        # Send action result
        result = NavigateToPose.Result()
        return result

        






def main(args=None):
    rclpy.init(args=args)
    node = TurtleMovement()
    executor = MultiThreadedExecutor() # Multi threads to run the service server in parallel with the pose callback
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()