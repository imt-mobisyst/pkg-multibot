#!/usr/bin/python3
import rclpy


from include.robot_controller import RobotController

from std_msgs.msg import Int8
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

from include.helpers import getYaw, createPoint
from package_dispenser import PACKAGE_COLORS

class StageRobotController(RobotController):

    def __init__(self):
        super().__init__('stage_robot_controller')

        # Init stage specific subscriptions
        self.create_subscription(Odometry, 'ground_truth', self.pose_callback, 10)
        self.create_subscription(Marker, '/package_marker', self.package_callback, 10)

        # Init stage specific publisher
        self.markerCleaner = self.create_publisher(MarkerArray, '/package_marker_array', 10)
        self.depositMarkerPublisher = self.create_publisher(Marker, '/package_deposit_marker', 10)

        # Init variables
        self.pose = Pose()

        self._targetPackage = None

        self._depositSpots = {
            "red":    createPoint(6.328, 2.392),
            "green":  createPoint(-2.136, -5.496),
            "blue":   createPoint(6.328, 6.328),
            "yellow": createPoint(-4.952, 1.816)
        }

        self._packageQueue = []

    def pose_callback(self, msg:Odometry):
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

    # Stop listening to goal poses
    def goalPose_callback(self, msg):
        return

    def package_callback(self, msg:Marker):
        # Save package data
        self._targetPackage = {
            'id': msg.id,
            'colorName': msg.ns,
            'position': msg.pose.position,
            'picked_up': False
        }

        # Calculate bid and send it to the operator
        self.sendBid(self._targetPackage['position'])


    def assignedRobot_callback(self, msg:Int8):
        assignedRobotId = int(msg.data)
        self.get_logger().info(f"Robot {assignedRobotId} goes to the target")

        if(assignedRobotId == self.paramInt('robot_id') and self._targetPackage is not None):# If the robot assigned is this one, tell it to move
            # Go take the package
            self.queue.append(self._targetPackage["position"])
            # Bring it to the deposit spot corresponding to its color
            self.queue.append(self._depositSpots[self._targetPackage["colorName"]])

            # Save the packet that you are supposed to move
            self._packageQueue.append(self._targetPackage)
        
        # If the robot assigned is not this one OR the position has been added to queue => reset
        self._targetPackage = None

    def removeSpawnMarker(self, id, ns):
        """Remove the marker with this specific id"""
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'map'

        marker.id = id
        marker.ns = ns
        marker.action = Marker.DELETE

        marker_array._markers.append(marker)

        self.markerCleaner.publish(marker_array)

    def addDepositMarker(self, package, depositPos):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()



        # set shape
        marker.type = 1 # Cube
        marker.id = package['id']
        marker.ns = package['colorName']

        # Set the scale of the marker
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        # Set the color
        color = PACKAGE_COLORS[package['colorName']]
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position = depositPos

        self.depositMarkerPublisher.publish(marker)


    def goalSucceeded(self):
        actualPackage = self._packageQueue[0]

        # If you reached the package, pick up
        if actualPackage['picked_up'] == False:
            # Remove the package marker
            self.removeSpawnMarker(self._packageQueue[0]['id'], self._packageQueue[0]['colorName'])

            # Change package state in queue
            self._packageQueue[0]['picked_up'] = True
            
        else:
            # Deposit package
            self.addDepositMarker(self._packageQueue[0], self.getRobotPosition())

            # Package correctly arrived at deposit spot, remove it from queue
            self._packageQueue.pop(0)


def main(args=None):
    rclpy.init(args=args)
    node = StageRobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()