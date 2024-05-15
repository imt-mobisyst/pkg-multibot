#!/usr/bin/python3
import rclpy
import threading


from include.robot_controller import RobotController

from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import Odometry

from include.helpers import getEulerFromQuaternion

class StageRobotController(RobotController):

    def __init__(self):
        super().__init__('stage_robot_controller')

        # Init stage specific subscriptions
        self.create_subscription(Odometry, 'ground_truth', self.pose_callback, 10)

        # Init variables
        self.pose = Pose()
        threading.Timer(1.0, self.publishPoseMarker).start() # Publish the first marker 1s after starting to let rviz launch


    def pose_callback(self, msg:Odometry):
        newPose = msg.pose.pose
        # If not moved since last callback, do nothing
        if(self.pose and newPose.position.x == self.pose.position.x and newPose.position.y == self.pose.position.y and getEulerFromQuaternion(newPose.orientation)['roll'] == getEulerFromQuaternion(self.pose.orientation)['roll']):
            return
        
        # If moved, update pose and marker
        self.pose = newPose
        self.publishPoseMarker()

    def getRobotPosition(self):
        p = Point()
        p.x = self.pose.position.x
        p.y = self.pose.position.y
        p.z = 0.0
        
        return p

    def getRobotAngle(self):
        return getEulerFromQuaternion(self.pose.orientation)['roll']





def main(args=None):
    rclpy.init(args=args)
    node = StageRobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()