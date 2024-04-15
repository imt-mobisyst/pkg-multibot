#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

class Operator(Node):

    def __init__(self):
        super().__init__('operator')
        self.subscription = self.create_subscription(Float32, 'distanceToTarget', self.turtleDistance_callback, 10)

    def turtleDistance_callback(self, msg):
        print(msg)







def main(args=None):
    rclpy.init(args=args)
    node = Operator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()