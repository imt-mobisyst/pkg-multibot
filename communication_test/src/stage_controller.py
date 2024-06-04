#!/usr/bin/python3
import rclpy


from include.robot_controller import RobotController

from std_msgs.msg import Int8
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

from include.helpers import getYaw, createPoint
from include.package import Package, PackageState
from include.tasks import StoreTask, RetrieveTask

class StageRobotController(RobotController):

    def __init__(self):
        super().__init__('stage_robot_controller')

        # Init stage specific subscriptions
        # -> Position
        self.create_subscription(Odometry, 'ground_truth', self.pose_callback, 10)
        # -> Store task
        self.create_subscription(Marker, '/package_marker', self.package_callback, 10)
        # -> Retrieve task
        self.create_subscription(Marker, '/retrieve_marker', self.retrieve_callback, 10)

        # Init stage specific publisher
        self.spawnMarkerCleaner = self.create_publisher(MarkerArray, '/package_marker_array', 10)
        self.depositMarkerCleaner = self.create_publisher(MarkerArray, '/package_deposit_marker_array', 10)

        self.depositMarkerPublisher = self.create_publisher(Marker, '/package_deposit_marker', 10)
        self.retrievedMarkerPublisher = self.create_publisher(Marker, '/package_retrieved_marker', 10)

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


    # Task reception

    def package_callback(self, msg:Marker):
        # Save package data
        self._targetPackage = Package(id=msg.id, colorName=msg.ns, position=msg.pose.position)

        # Calculate bid and send it to the operator
        self.sendBid(self._targetPackage.position)

    def retrieve_callback(self, msg:Marker):
        # Save package data
        self._targetPackage = Package(id=msg.id, colorName=msg.ns, position=msg.pose.position, state=PackageState.STORED)

        # Calculate bid and send it to the operator
        self.sendBid(self._targetPackage.position)


    def assignedRobot_callback(self, msg:Int8):
        assignedRobotId = int(msg.data)
        self.get_logger().info(f"Robot {assignedRobotId} goes to the target")

        if(assignedRobotId == self.paramInt('robot_id') and self._targetPackage is not None):# If the robot assigned is this one, tell it to move
            
            if self._targetPackage.state == PackageState.SPAWNED:
                # Create a store task (go to the package and bring it to the deposit spot corresponding to its color)
                task = StoreTask(self._targetPackage)
            else:
                # Create a retrieve task (go to the package and bring it to the retrieval spot)
                task = RetrieveTask(self._targetPackage)

            self.queue.addTask(task)
        
        # If the robot assigned is not this one OR the position has been added to queue => reset
        self._targetPackage = None

    def removeMarker(self, package:Package, pub):
        """Remove the marker with this specific id"""
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'map'

        marker.id = package.id
        marker.ns = package.colorName
        marker.action = Marker.DELETE

        marker_array._markers.append(marker)

        pub.publish(marker_array)


    def goalSucceeded(self):
        # Get REFERENCE to task
        task = self.queue.tasks[0]

        if isinstance(task, StoreTask):

            # If you reached the package, pick up
            if task.package.state == PackageState.SPAWNED:
                # Remove the package marker from '/package_marker'
                self.removeMarker(task.package, self.spawnMarkerCleaner)

                # Change package state in queue
                task.package.state = PackageState.STORING
                
            else:
                # Deposit package
                # -> Update package
                task.package.position = self.getRobotPosition() # Change package position
                task.package.state = PackageState.STORED        # Change package state
                # -> Publish marker
                task.package.publishMarker(self.depositMarkerPublisher, self)



        elif isinstance(task, RetrieveTask):

            # If you reached the package, pick up
            if task.package.state == PackageState.STORED:
                # Remove the package marker from '/package_deposit_marker'
                self.removeMarker(task.package, self.depositMarkerCleaner)

                # Change package state in queue
                task.package.state = PackageState.RETRIEVING
                
            else:
                # Drop package
                # -> Update package
                task.package.position = self.getRobotPosition() # Change package position
                task.package.state = PackageState.RETRIEVED     # Change package state
                # -> Publish marker
                task.package.publishMarker(self.retrievedMarkerPublisher, self)


def main(args=None):
    rclpy.init(args=args)
    node = StageRobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()