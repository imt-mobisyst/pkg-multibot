#!/usr/bin/python3
import rclpy
from os import getenv
import threading


from include.robot_controller import RobotController

from geometry_msgs.msg import Point
from turtlesim.msg import Pose

class TurtleController(RobotController):

    def __init__(self):
        super().__init__('turtle_controller')

        # Log DDS server
        if(getenv('ROS_DISCOVERY_SERVER') is not None):
            self.get_logger().info("RUNNING on DDS \"" + getenv('ROS_DISCOVERY_SERVER') + "\"")

        # Init turtlesim specific subscriptions
        self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)


        # Init variables
        self.pose = Pose()
        self.pose.x = 5.544444561004639
        self.pose.y = 5.544444561004639
        threading.Timer(1.0, self.publishPoseMarker).start() # Publish the first marker 1s after starting to let rviz launch


    def pose_callback(self, msg:Pose):
        # If not moved since last callback, do nothing
        if(msg.x == self.pose.x and msg.y == self.pose.y and msg.theta == self.pose.theta):
            return
        
        # If moved, update pose and marker
        self.pose = msg
        self.publishPoseMarker()

    def getRobotPosition(self):
        p = Point()
        p.x = self.pose.x
        p.y = self.pose.y
        p.z = 0.0
        
        return p

    def getRobotAngle(self):
        return self.pose.theta





def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()