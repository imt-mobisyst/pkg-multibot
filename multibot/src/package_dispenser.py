#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

from random import random

from include.package import Package

class PackageDispenserNode(Node):

    MAX_PACKAGES_WAITING = 4

    def __init__(self):
        super().__init__('package_dispenser')

        self.declare_parameter('period', 1)
        self.declare_parameter('rate', 0.07) # 7% package spawn every second by default

        # Set package positions
        self.declare_parameter('is_simulation', True)
        Package.setSimulation(self.paramBool('is_simulation'))

        # Init publishers
        self.markerPublisher = self.create_publisher(Marker, '/package_marker', 10)
        self.spawnedMarkerCleaner = self.create_publisher(MarkerArray, '/package_marker_array', 10)
        self.depositMarkerCleaner = self.create_publisher(MarkerArray, '/package_deposit_marker_array', 10)
        self.retrievedMarkerCleaner = self.create_publisher(MarkerArray, '/package_retrieved_marker_array', 10)

        # Init subscriber
        self.create_subscription(PointStamped, "/clicked_point", self.toggleSpawn, 10)
        self.create_subscription(MarkerArray, '/package_marker_array', self.packagePickedUp, 10)
        
        # Init loop
        self.create_timer(self.paramInt('period'), self.loop)


        # Clear previous markers
        self.clearMarkers()

        # Variables
        self._spawn = False # By default, don't start spawning packages
        self._nbPackagesWaitingStore = 0
    
    def paramInt(self, name):
        return self.get_parameter(name).get_parameter_value().integer_value
    def paramDouble(self, name):
        return self.get_parameter(name).get_parameter_value().double_value
    def paramBool(self, name):
        return self.get_parameter(name).get_parameter_value().bool_value
    


    def toggleSpawn(self, _):
        # If already active, deactivate
        if self._spawn:
            self.get_logger().info("STOP SPAWNING PACKAGES")
            self._spawn = False
            return
        
        
        if Package.nbPackages == 0:

            # Spawn a first package
            self.spawnRandomPackage()

        # Enable future spawns
        self.get_logger().info("START SPAWNING PACKAGES")
        self._spawn = True

    def packagePickedUp(self, _):
        # Prevent first clear from being listened
        if self._nbPackagesWaitingStore == 0:
            return
        
        # Log if starts spawning again
        if self._nbPackagesWaitingStore == self.MAX_PACKAGES_WAITING:
            self.get_logger().info("Number of packages OK, restarting spawn")

        # Update number of waiting packages
        self._nbPackagesWaitingStore -= 1


    def spawnRandomPackage(self):
        # Create a random package
        package = Package.random()

        # Update number of waiting packages
        self._nbPackagesWaitingStore += 1
        if self._nbPackagesWaitingStore == self.MAX_PACKAGES_WAITING:
            self.get_logger().info("Max number of packages reached, stopping spawn")

        # Publish marker (for visualization but also information for the robots)
        package.publishMarker(self.markerPublisher, self)


    def loop(self):
        # Only spawn if toggled and not too much packages
        canSpawn = self._spawn and self._nbPackagesWaitingStore < self.MAX_PACKAGES_WAITING

        # There is a probability of spawning a package every second (ex: 10% if rate=0.1)
        if canSpawn and random() < self.paramDouble('rate') :
            # Spawn a package with a random color and a random spot
            self.spawnRandomPackage()



    def clearMarkers(self):
        self.get_logger().info("Clearing previous package markers")
        
        marker_array = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)

        self.spawnedMarkerCleaner.publish(marker_array)
        self.depositMarkerCleaner.publish(marker_array)
        self.retrievedMarkerCleaner.publish(marker_array)




def main(args=None):
    rclpy.init(args=args)
    node = PackageDispenserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()