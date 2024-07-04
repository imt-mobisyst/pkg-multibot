#!/usr/bin/python3
import rclpy
from rclpy.node import Node

import rclpy.time
from std_msgs.msg import String

class ListenerNode(Node):

    def __init__(self):
        super().__init__('listener')

        # Init subscribers
        self.publisher = self.create_subscription(String, '/test', self.receiveMessage, 10)

        self.get_logger().info('Listening...')
    

        self.totalNbMessages = 0
    
    

    def receiveMessage(self, msg:String):
        """
        Receive a message to another computer
        """

        if self.totalNbMessages == 0:
            self.initTime = self.get_clock().now()



        # Check current time
        current_time = self.get_clock().now()
        duration = (current_time - self.initTime).nanoseconds * 10**(-9)

        
        # Save data
        self.totalNbMessages += 1
        self.get_logger().info(f"Received message n°{self.totalNbMessages} ({msg.data}) at time {duration}")
            
            









def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()