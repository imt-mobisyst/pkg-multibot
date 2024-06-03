#!/usr/bin/python3
import rclpy


from include.robot_controller import RobotController

from std_msgs.msg import Int8
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

from include.helpers import getYaw, createPoint
from include.package import Package
from include.tasks import StoreTask

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


    def package_callback(self, msg:Marker):
        # Save package data
        self._targetPackage = Package(id=msg.id, colorName=msg.ns, position=msg.pose.position)

        # Calculate bid and send it to the operator
        self.sendBid(self._targetPackage.position)


    def assignedRobot_callback(self, msg:Int8):
        assignedRobotId = int(msg.data)
        self.get_logger().info(f"Robot {assignedRobotId} goes to the target")

        if(assignedRobotId == self.paramInt('robot_id') and self._targetPackage is not None):# If the robot assigned is this one, tell it to move
            # Create a store task (go to the package and bring it to the deposit spot corresponding to its color)
            task = StoreTask(self._targetPackage)
            self.queue.addTask(task)
        
        # If the robot assigned is not this one OR the position has been added to queue => reset
        self._targetPackage = None

    def removeSpawnMarker(self, package:Package):
        """Remove the marker with this specific id"""
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'map'

        marker.id = package.id
        marker.ns = package.colorName
        marker.action = Marker.DELETE

        marker_array._markers.append(marker)

        self.markerCleaner.publish(marker_array)

    def addDepositMarker(self, package:Package, depositPos):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()



        # set shape
        marker.type = 1 # Cube
        marker.id = package.id
        marker.ns = package.colorName

        # Set the scale of the marker
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        # Set the color
        color = package.color()
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position = depositPos

        self.depositMarkerPublisher.publish(marker)


    def goalSucceeded(self):
        actualPackage:Package = self.queue.tasks[0].package

        # If you reached the package, pick up
        if actualPackage.pickedUp == False:
            # Remove the package marker
            self.removeSpawnMarker(self.queue.tasks[0].package)

            # Change package state in queue
            self.queue.tasks[0].package.pickedUp = True
            
        else:
            # Deposit package
            self.addDepositMarker(self.queue.tasks[0].package, self.getRobotPosition())


def main(args=None):
    rclpy.init(args=args)
    node = StageRobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()