from random import random, choice as randomChoice
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from include.helpers import createPoint

class Package():

    # Static variables
    nbPackages = 0

    colors = {
        'red':    [0.941, 0.502, 0.502],
        'green':  [0.565, 0.933, 0.565],
        'blue':   [0.529, 0.808, 0.980],
        'yellow': [0.933, 0.867, 0.510],
    }

    spawnSpots = [
        createPoint(-6.328, 6.328), # Top left dispenser
        createPoint(6.328, -6.328)  # Bottom right dispenser
    ]

    depositSpots = {
        "red":    createPoint(6.328, 2.392),
        "green":  createPoint(-3.704, -6.328),
        "blue":   createPoint(6.328, 6.328),
        "yellow": createPoint(-4.952, 1.816)
    }
    
    retrievalSpot = createPoint(6.328, -1.784)


    # Functions

    def __init__(self, colorName:str, position:Point, pickedUp = False, id=None):
        self.colorName = colorName
        self.position = position
        self.pickedUp = pickedUp

        if id is None:
            # Automatic ID generation
            self.id = Package.nbPackages
            Package.nbPackages += 1
        else:
            self.id = id

    
    def random():
        """Create a random package to spawn"""
        # Each deposit point and color has a the same chance of being chosen
        targetSpot = randomChoice(Package.spawnSpots)
        packageColorName = randomChoice(list(Package.colors.keys()))

        # Add variation to the spawn point
        packageSpot = createPoint(targetSpot.x + (random()-0.5)*1.5, targetSpot.y + (random()-0.5)*1.5)

        return Package(packageColorName, packageSpot)


    def depositSpot(self):
        """Get the position that the package should be stored at"""
        return Package.depositSpots[self.colorName]
    

    def color(self):
        """Get the color of the package ([R,G,B])"""
        return Package.colors[self.colorName]
    
    def publishMarker(self, publisher, node):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = node.get_clock().now().to_msg()

        # set shape
        marker.type = 1 # Cube
        marker.id = self.id
        marker.ns = self.colorName

        # Set the scale of the marker
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        # Set the color
        color = self.color()
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position = self.position

        publisher.publish(marker)