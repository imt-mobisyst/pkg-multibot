#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class ParasitNode(Node):

    def __init__(self):
        super().__init__('parasit')


        # Declare ROS parameters
        self.declare_parameter('ip_address', '')
        self.declare_parameter('period', 5)

        if(self.paramString('ip_address') == ''):
            self.get_logger().error("`ip_address` ROS2 parameter must be defined for ParasitNode")
            self.destroy_node()
            return

        # Init publishers
        self.publisher = self.create_publisher(String, '/announcement', 10)
        
        # Init loop
        self.create_timer(self.paramInt('period'), self.sendMessage)

    def paramString(self, name):
        return self.get_parameter(name).get_parameter_value().string_value

    def paramInt(self, name):
        return self.get_parameter(name).get_parameter_value().integer_value
    

    def sendMessage(self):
        """
        Send a message every 5s with the robot ip
        """
        str = String()
        str.data = self.paramString('ip_address')

        self.publisher.publish(str)







def main(args=None):
    rclpy.init(args=args)
    node = ParasitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()