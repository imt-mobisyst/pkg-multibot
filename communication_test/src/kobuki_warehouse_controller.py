#!/usr/bin/python3
import rclpy


from include.warehouse_controller import WarehouseRobotController

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from include.helpers import getYaw, createPoint

class KobukiWarehouseController(WarehouseRobotController):

    def __init__(self):
        super().__init__('kobuki_controller')

        # Init stage specific subscriptions
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
    node = KobukiWarehouseController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()