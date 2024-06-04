#!/usr/bin/python3
import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from static_operator import Operator

from visualization_msgs.msg import Marker
from include.package import Package

class StageOperator(Operator):

    def __init__(self):
        self.robotIPaddresses = {}
        
        # Init node with default operator behaviour
        super().__init__('stage_operator')

        # Init subscribers
        # -> Update stock
        self.create_subscription(Marker, '/package_deposit_marker', self.packageDepositCallback, 10)
        # -> Retrieve task
        self.create_subscription(String, '/retrieve', self.retrieveCallback, 10)


        # Init publisher
        self.retrievePosePublisher = self.create_publisher(Marker, '/retrieve_marker', 10)
    

        # Init empty stock
        self.stock:dict[str, list[Package]] = {}
        for c in Package.colors:
            self.stock[c] = []

    def packageDepositCallback(self, msg:Marker):
        # Create package based on marker received
        package = Package(id=msg.id, colorName=msg.ns, position=msg.pose.position)
        
        # Add package to the stock if valid color
        if package.colorName in self.stock:
            self.stock[package.colorName].append(package)
        else:
            self.get_logger().info("Invalid package color")
    
    def retrieveCallback(self, msg:String):
        colorName = msg.data

        # Check if a package of this color is in stock
        if colorName not in self.stock:
            self.get_logger().info("This color doesn't exist")
            return
        if len(self.stock[colorName]) == 0:
            self.get_logger().info("There are currently no packages of this color in stock")
            return
        
        package = self.stock[colorName].pop(0) # Get the first package (oldest)

        # Send message (Marker) to robots to request an auction bid
        package.publishMarker(self.retrievePosePublisher, self)


def main(args=None):
    rclpy.init(args=args)
    node = StageOperator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()