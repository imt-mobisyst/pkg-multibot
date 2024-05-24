#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from random import random, choice as randomChoice

from include.helpers import createPoint

PACKAGE_COLORS = {
    'red':    [0.941, 0.502, 0.502],
    'green':  [0.565, 0.933, 0.565],
    'blue':   [0.529, 0.808, 0.980],
    'yellow': [0.933, 0.867, 0.510],
}

class PackageDispenserNode(Node):

    def __init__(self):
        super().__init__('package_dispenser')

        self.declare_parameter('period', 1)
        self.declare_parameter('rate', 0.05) # 5% package spawn every second by default

        # Init publishers
        self.markerPublisher = self.create_publisher(Marker, '/package_marker', 10)
        self.markerCleaner = self.create_publisher(MarkerArray, '/package_marker_array', 10)
        
        # Init loop
        self.create_timer(self.paramInt('period'), self.loop)


        # Clear previous markers
        self.clearMarkers()

        # Variables

        self._spawnSpots:list[Point] = [
            createPoint(-6.328, 6.328), # Top left dispenser
            createPoint(6.328, -6.328)  # Bottom right dispenser
        ]
        self._spotSize = 2 # Square of 2x2m

        self._totalPackagesSpawned = 0
    
    def paramInt(self, name):
        return self.get_parameter(name).get_parameter_value().integer_value
    def paramDouble(self, name):
        return self.get_parameter(name).get_parameter_value().double_value

    def spawnPackage(self, spot, color, colorName):
        # Create unique package ID
        packageID = self._totalPackagesSpawned

        # Publish marker (for visualization but also information for the robots)
        self.publishMarker(spot, color, colorName, packageID)

        # Save new total number of packages
        self._totalPackagesSpawned += 1


    def loop(self):
        # There is a  probability of spawning a package every second (ex: 10% if rate=0.1)
        if random() < self.paramDouble('rate'):
            # Each deposit point and color has a the same chance of being chosen
            targetSpot = randomChoice(self._spawnSpots)
            packageColorName = randomChoice(list(PACKAGE_COLORS.keys()))
            packageColor = PACKAGE_COLORS[packageColorName]

            # Add variation to the spawn point
            packageSpot = createPoint(targetSpot.x + (random()-0.5)*1.5, targetSpot.y + (random()-0.5)*1.5)

            # Spawn a package of the selected color at the selected spot
            self.spawnPackage(packageSpot, packageColor, packageColorName)

    def clearMarkers(self):
        marker_array = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)

        self.markerCleaner.publish(marker_array)

    def publishMarker(self, pos:Point, color, colorName, id):
        # Publish marker for vizualisation in rviz
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = self.get_clock().now().to_msg()

        # set shape
        marker.type = 1 # Cube
        marker.ns = colorName
        marker.id = id

        # Set the scale of the marker
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        # Set the color
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position = pos

        self.markerPublisher.publish(marker)





def main(args=None):
    rclpy.init(args=args)
    node = PackageDispenserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()