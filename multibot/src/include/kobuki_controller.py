#!/usr/bin/python3
import rclpy
import numpy as np


from include.robot_controller import RobotController
from include.helpers import getYaw, euclideanDistance

from geometry_msgs.msg import Pose


import rclpy.duration
from tf2_ros import TransformException
import rclpy.time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 

class KobukiController(RobotController):

    def __init__(self, name='kobuki_controller'):
        super().__init__(name)

        # Init turtlesim specific subscriptions
        # -> Position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Init pose loop
        self.create_timer(1/10.0, self.getPose)
        self.retryCount = 0

        # Init variables
        self.pose = Pose()

        # Visualization
        self.markerScale = 0.7

    
    # Get position and rotation of the robot

    def getPose(self):

        now = rclpy.time.Time()
        try:
            # Get TF between 'map' and 'base_link'
            newTF = self.tf_buffer.lookup_transform(
                    'map',
                    'base_footprint',
                    now).transform


            # Create pos from TF
            newPose = Pose()
            newPose.position
            newPose.position.x = newTF.translation.x
            newPose.position.y = newTF.translation.y
            newPose.position.z = 0.0
            newPose.orientation = newTF.rotation


            # If not moved since last callback, do nothing
            if(self.pose is not None and euclideanDistance(newPose.position, self.pose.position) < 0.05 and np.abs(getYaw(newPose.orientation) - getYaw(self.pose.orientation)) < 0.05):
                return
            
            # Store new pose
            self.pose = newPose
            
            # If moved significantly, update marker
            self.publishPoseMarker()

            # Reset retry count
            self.retryCount = 0
        
        except TransformException as transEx:
            if self.retryCount > 0:
                self.retryCount -=1

            if self.retryCount == 0:
                self.get_logger().info( f"tf: 'base_footprint' -> 'map' : IMPOSSIBLE\n\t > {transEx}" )
                self.retryCount = 20


    def getRobotPosition(self):
        return self.pose.position

    def getRobotAngle(self):
        return getYaw(self.pose.orientation)
    




def main(args=None):
    rclpy.init(args=args)
    node = KobukiController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()