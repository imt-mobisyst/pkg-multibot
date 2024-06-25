from enum import Enum
from random import random, choice as randomChoice
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from include.helpers import createPoint

class PackageState(Enum):
    SPAWNED = 1     # When the package is waiting at the spawn spot
    STORING = 2     # When the package is being moved from the spawn spot to it's deposit spot
    STORED = 3      # When the package is in stock (at it's deposit spot)
    RETRIEVING = 4  # When the package is being moved from the deposit spot to the retrieval spot
    RETRIEVED = 5   # When the package is waiting at the retrieval spot

class Package():

    # Static variables (simulation values)
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
    
    retrievalSpotPoint = createPoint(6.328, -1.784)

    markerScale = 1.0
    squareSize = 1.5


    # Different positions when running on the real robot
    def setSimulation(isSimulation):
        if isSimulation:
            return
        
        # Remove yellow color
        Package.colors = {
            'red':    Package['red'],
            'green':  Package['green'],
            'blue':   Package['blue'],
        }

        # Set points in the real map coordinates
        Package.spawnSpots = [
            createPoint(-0.618, -2.852),
            createPoint(-3.127, 2.344)
        ]


        Package.depositSpots = {
            "red":    createPoint(-0.823, 0.649),
            "green":  createPoint(-2.332, -1.527),
            "blue":   createPoint(-4.358, -1.149)
        }
        
        # TODO: Set real retrieval point
        Package.retrievalSpotPoint = createPoint(-1,-1)

        Package.markerScale = 0.5
        Package.squareSize = 1


    # Functions

    def __init__(self, colorName:str, position:Point, state = PackageState.SPAWNED, id=None):
        self.colorName = colorName
        self.position = position
        self.state = state

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
        packageSpot = Package.randomizeSpot(targetSpot)

        return Package(packageColorName, packageSpot)



    def randomizeSpot(center:Point):
        """Create a random point in a square around a center"""        
        return createPoint(center.x + (random()-0.5) * Package.squareSize,
                           center.y + (random()-0.5) * Package.squareSize)



    def depositSpot(self):
        """Get the position that the package should be stored at"""
        return Package.randomizeSpot(Package.depositSpots[self.colorName])
    
    def retrievalSpot():
        """Get the position that the package should be retrieved at"""
        return Package.randomizeSpot(Package.retrievalSpotPoint)

    

    def color(self):
        """Get the color of the package ([R,G,B])"""
        return Package.colors[self.colorName]
    
    def publishMarker(self, publisher, node):
        """Publish a marker at the current package position, with the correct color"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = node.get_clock().now().to_msg()

        # set shape
        marker.type = 1 # Cube
        marker.id = self.id
        marker.ns = self.colorName

        # Set the scale of the marker
        marker.scale.x = 0.3 * Package.markerScale
        marker.scale.y = 0.3 * Package.markerScale
        marker.scale.z = 0.3 * Package.markerScale

        # Set the color
        color = self.color()
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position = self.position

        publisher.publish(marker)