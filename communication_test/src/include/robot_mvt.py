#!/usr/bin/python3
from os import getenv
import time
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

import numpy as np

from geometry_msgs.msg import PoseStamped, Twist, Point
from turtlesim.msg import Pose

from nav2_msgs.action import NavigateToPose

from include.helpers import getQuaternionFromEuler

class RobotMovement(Node):

    def __init__(self, name='robot_mvt', vel_topic='cmd_vel'):
        super().__init__(name)

        # Declare ROS parameters
        self.declare_parameter('robot_id', 1)
        self.get_logger().info(f"Robot {self.paramInt('robot_id')} started")

        # Log DDS server
        if(getenv('ROS_DISCOVERY_SERVER') is not None):
            self.get_logger().info("RUNNING on DDS \"" + getenv('ROS_DISCOVERY_SERVER') + "\"")

        # Start action server
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.navigateToPoseCallback)

        # Init publishers
        self.velPublisher = self.create_publisher(Twist, vel_topic, 10)
        
        # Init variables
        self.pose = Pose()
       
        self.speedFactor = 0.2
        self.angleTolerance = np.pi / 180.0
        self.distanceTolerance = 0.01

        self.targetPos = None

    def paramInt(self, name):
        return self.get_parameter(name).get_parameter_value().integer_value


    def pose_callback(self, msg):
        raise NotImplementedError

    def getRobotPosition(self) -> Point:
        raise NotImplementedError

    def getRobotAngle(self) -> float:
        raise NotImplementedError


    def euclidean_distance(self, targetPos, initPose=None):
        if initPose is None:
            initPose = self.getRobotPosition()

        return np.sqrt(np.square(targetPos.y - initPose.y) + np.square(targetPos.x - initPose.x))

    
    def linear_vel(self, targetPos, constant=1.5):
        return constant * self.euclidean_distance(targetPos)   
    
    def steering_angle(self, targetPos):
        return np.arctan2(targetPos.y - self.getRobotPosition().y, targetPos.x - self.getRobotPosition().x)
    
    def angular_vel(self, goal_pose, constant=50):
        angle = (self.steering_angle(goal_pose) - self.getRobotAngle())
        return constant * angle


    def createFeedbackMsg(self):
        feedback_msg = NavigateToPose.Feedback()
        feedback_msg.current_pose = PoseStamped()
        feedback_msg.current_pose.pose.position = self.getRobotPosition()
        feedback_msg.current_pose.pose.orientation = getQuaternionFromEuler(0, 0, self.getRobotAngle())

        feedback_msg.distance_remaining = self.euclidean_distance(self.targetPos)
        
        # self.get_logger().info('Feedback: {0}'.format(feedback_msg.distance_remaining))

        return feedback_msg

    def navigateToPoseCallback(self, goal_handle:ServerGoalHandle):
        self.get_logger().info('Executing goal...')

        request:NavigateToPose = goal_handle.request
        self.targetPos:Point = request.pose.pose.position

        # First rotate approximately to the correct rotation
        while np.abs(self.getRobotAngle() - self.steering_angle(self.targetPos)) >= self.angleTolerance:
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

        # Stop robot
        self.velPublisher.publish(Twist())

        # Finish action
        goal_handle.succeed()

        # Send action result
        result = NavigateToPose.Result()
        return result
