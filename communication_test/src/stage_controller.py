#!/usr/bin/python3
import rclpy


from include.robot_controller import RobotController

from std_msgs.msg import Int8
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

from include.helpers import getEulerFromQuaternion, createPoint

class StageRobotController(RobotController):

    def __init__(self):
        super().__init__('stage_robot_controller')

        # Init stage specific subscriptions
        self.create_subscription(Odometry, 'ground_truth', self.pose_callback, 10)
        self.create_subscription(Marker, '/package_marker', self.package_callback, 10)

        # Init variables
        self.pose = Pose()

        self._targetPackage = None

        self._depositSpots = {
            "red": createPoint(6.250, 2.40),
            "green": createPoint(-1.896, -5.0),
            "blue": createPoint(6.250, 6.25),
            "yellow": createPoint(-3.70, 2.75)
        }

    def pose_callback(self, msg:Odometry):
        newPose = msg.pose.pose
        # If not moved since last callback, do nothing
        if(self.pose is not None and newPose.position.x == self.pose.position.x and newPose.position.y == self.pose.position.y and getEulerFromQuaternion(newPose.orientation)['yaw'] == getEulerFromQuaternion(self.pose.orientation)['yaw']):
            return
        
        # If moved, update pose and marker
        self.pose = newPose
        self.publishPoseMarker()

    def getRobotPosition(self):
        return createPoint(self.pose.position.x, self.pose.position.y)

    def getRobotAngle(self):
        return getEulerFromQuaternion(self.pose.orientation)['yaw']

    # Stop listening to goal poses
    def goalPose_callback(self, msg):
        return

    def package_callback(self, msg:Marker):
        # Save package data
        self._targetPackage = {
            'id': msg.id,
            'colorName': msg.ns,
            'position': msg.pose.position
        }

        # Calculate bid and send it to the operator
        self.sendBid(self._targetPackage['position'])


    def assignedRobot_callback(self, msg:Int8):
        assignedRobotId = int(msg.data)
        print(f"Robot {assignedRobotId} goes to the target")

        if(assignedRobotId == self.paramInt('robot_id') and self._targetPackage is not None):# If the robot assigned is this one, tell it to move
            # Go take the package
            self.queue.append(self._targetPackage["position"])
            # Bring it to the deposit spot corresponding to its color
            self.queue.append(self._depositSpots[self._targetPackage["colorName"]])
        
        # If the robot assigned is not this one OR the position has been added to queue => reset
        self._targetPackage = None

def main(args=None):
    rclpy.init(args=args)
    node = StageRobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()