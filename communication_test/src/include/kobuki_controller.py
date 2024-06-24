#!/usr/bin/python3
import rclpy


from include.robot_controller import RobotController
from include.helpers import createPoint, getYaw

from geometry_msgs.msg import PoseWithCovarianceStamped
from turtlesim.msg import Pose

class KobukiController(RobotController):

    def __init__(self, name='kobuki_controller'):
        super().__init__(name)

        # Init turtlesim specific subscriptions
        # -> Position
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.pose_callback, 10)

        # Init variables
        self.pose = Pose()

        # Visualization
        self.markerScale = 0.7


    # Get position and rotation of the robot

    def pose_callback(self, msg:PoseWithCovarianceStamped):
        newPose = msg.pose.pose
        # If not moved since last callback, do nothing
        if(self.pose is not None and newPose.position.x == self.pose.position.x and newPose.position.y == self.pose.position.y and getYaw(newPose.orientation) == getYaw(self.pose.orientation)):
            return
        
        # If moved, update pose and marker
        self.pose = newPose
        self.publishPoseMarker()

    def getRobotPosition(self):
        return createPoint(self.pose.position.x, self.pose.position.y)

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