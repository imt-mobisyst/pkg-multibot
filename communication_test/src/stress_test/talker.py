#!/usr/bin/python3
import rclpy
from rclpy.node import Node

import rclpy.time
from std_msgs.msg import String

class TalkerNode(Node):

    def __init__(self):
        super().__init__('talker')


        # Declare ROS parameters
        self.declare_parameter('duration', 10.0)
        self.declare_parameter('frequency', 100.0)

        # Log
        self.get_logger().info(f"Starting to publish at {self.paramDouble('frequency')}Hz for {self.paramDouble('duration')}s\n")

        # Init publishers
        self.publisher = self.create_publisher(String, '/test', 10)
        
        # Init loop
        self.create_timer(1.0 / self.paramDouble('frequency'), self.sendMessage)

        self.totalNbMessages = 0

        self.initTime = self.get_clock().now()
    
    def paramDouble(self, name):
        return self.get_parameter(name).get_parameter_value().double_value
    

    def sendMessage(self):
        """
        Send a message to another computer
        """

        # Check current time
        current_time = self.get_clock().now()
        duration = (current_time - self.initTime).nanoseconds * 10**(-9)

        
        if duration > self.paramDouble('duration'):
            self.get_logger().info(f"-> Successfully sent {self.totalNbMessages} messages")

            # Tell listener that it's the end
            str = String()
            str.data = f"END {self.totalNbMessages}"
            self.publisher.publish(str)

            # Kill node
            self.destroy_node()
            exit()
            

        # Send message
        str = String()
        str.data = f"Hello world {self.totalNbMessages+1}"

        self.publisher.publish(str)

        self.totalNbMessages += 1
        self.get_logger().info(f"Sending message n°{self.totalNbMessages} at time {duration}")









def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()