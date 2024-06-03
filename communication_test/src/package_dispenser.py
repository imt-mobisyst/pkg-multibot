#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

from random import random

from include.package import Package

class PackageDispenserNode(Node):

    def __init__(self):
        super().__init__('package_dispenser')

        self.declare_parameter('period', 1)
        self.declare_parameter('rate', 0.05) # 5% package spawn every second by default

        # Init publishers
        self.markerPublisher = self.create_publisher(Marker, '/package_marker', 10)
        self.markerCleaner = self.create_publisher(MarkerArray, '/package_marker_array', 10)
        self.depositMarkerCleaner = self.create_publisher(MarkerArray, '/package_deposit_marker_array', 10)

        # Init subscriber
        self.create_subscription(PointStamped, "/clicked_point", self.toggleSpawn, 10)
        
        # Init loop
        self.create_timer(self.paramInt('period'), self.loop)


        # Clear previous markers
        self.clearMarkers()


        # Variables
        self._spawn = False # By default, don't start spawning packages
    
    def paramInt(self, name):
        return self.get_parameter(name).get_parameter_value().integer_value
    def paramDouble(self, name):
        return self.get_parameter(name).get_parameter_value().double_value
    


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


    def spawnRandomPackage(self):
        # Create a random package
        package = Package.random()

        # Publish marker (for visualization but also information for the robots)
        self.publishPackageMarker(package)


    def loop(self):
        # There is a probability of spawning a package every second (ex: 10% if rate=0.1)
        if self._spawn and random() < self.paramDouble('rate'):
            # Spawn a package with a random color and a random spot
            self.spawnRandomPackage()



    def clearMarkers(self):
        self.get_logger().info("Clearing previous package markers")
        
        marker_array = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)

        self.markerCleaner.publish(marker_array)
        self.depositMarkerCleaner.publish(marker_array)

    def publishPackageMarker(self, package:Package):
        # Publish marker for vizualisation in rviz
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        # set shape
        marker.type = 1 # Cube
        marker.ns = package.colorName
        marker.id = package.id

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
        marker.pose.position = package.position

        self.markerPublisher.publish(marker)





def main(args=None):
    rclpy.init(args=args)
    node = PackageDispenserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()