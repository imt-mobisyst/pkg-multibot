#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from random import random, choice as randomChoice

from include.helpers import createPoint

class PackageDispenserNode(Node):

    def __init__(self):
        super().__init__('package_dispenser')

        self.declare_parameter('period', 1)

        # Init publishers
        self.markerPublisher = self.create_publisher(Marker, '/package_marker', 10)
        self.markerCleaner = self.create_publisher(MarkerArray, '/package_marker_array', 10)
        
        # Init loop
        self.create_timer(self.paramInt('period'), self.loop)


        # Clear previous markers
        self.clearMarkers()

        # Variables

        self._spawnSpots:list[Point] = [
            createPoint(-6.998, 6.998), # Top left dispenser
            createPoint(6.998, -6.998)  # Bottom right dispenser
        ]
        self._spotSize = 2 # Square of 2x2m

        self._packageColors = [
            {'name': 'red', 'rgb': [0.941, 0.502, 0.502]},
            {'name': 'green', 'rgb': [0.565, 0.933, 0.565]},
            {'name': 'blue', 'rgb': [0.529, 0.808, 0.980]},
            {'name': 'yellow', 'rgb': [0.933, 0.867, 0.510]},
        ]

        self._totalPackagesSpawned = 0
    
    def paramInt(self, name):
        return self.get_parameter(name).get_parameter_value().integer_value

    def spawnPackage(self, spot, color):
        # Create unique package ID
        packageID = self._totalPackagesSpawned

        # Publish marker (for visualization but also information for the robots)
        self.publishMarker(spot, color, packageID)

        # Save new total number of packages
        self._totalPackagesSpawned += 1


    def loop(self):
        # There is a 10% probability of spawning a package every second
        if random() < 0.1:
            # Each deposit point and color has a the same chance of being chosen
            targetSpot = randomChoice(self._spawnSpots)
            packageColor = randomChoice(self._packageColors)

            # Add variation to the spawn point
            packageSpot = createPoint(targetSpot.x + (random()-0.5)*1.5, targetSpot.y + (random()-0.5)*1.5)

            # Spawn a package of the selected color at the selected spot
            self.spawnPackage(packageSpot, packageColor)

    def clearMarkers(self):
        marker_array = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)

        self.markerCleaner.publish(marker_array)

    def publishMarker(self, pos:Point, color, id):
        # Publish marker for vizualisation in rviz
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = self.get_clock().now().to_msg()

        # set shape
        marker.type = 1 # Cube
        marker.ns = color["name"]
        marker.id = id

        # Set the scale of the marker
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        # Set the color
        marker.color.r = float(color["rgb"][0])
        marker.color.g = float(color["rgb"][1])
        marker.color.b = float(color["rgb"][2])
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