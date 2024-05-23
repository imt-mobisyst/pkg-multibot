#!/usr/bin/python3
import rclpy
from std_msgs.msg import String
from static_operator import Operator

class DynamicOperator(Operator):

    def __init__(self):
        self.robotIPaddresses = {}
        
        # Init node with default operator behaviour
        super().__init__('dynamic_operator')

        # Declare ROS parameters
        self.declare_parameter('robot_timeout', 30)

        # Init subscriber
        self.create_subscription(String, '/announcement', self.announcementCallback, 10)

        # Periodically clean the robots list
        self.create_timer(10, self.cleanRobotsList)
        
        

    def getNbRobots(self):
        return len(self.robotIPaddresses)

    def announcementCallback(self, msg:String):
        # Get IP
        robotIP = msg.data

        if robotIP not in self.robotIPaddresses.keys():
            # Log info
            self.get_logger().info(f"New robot added with IP {robotIP} - Now at {self.getNbRobots() + 1} robot(s)")
        
        # Add robot to list if not already in
        self.robotIPaddresses[msg.data] = self.get_clock().now()



    def cleanRobotsList(self):
        """Clean old robots that have not sent data in a while"""

        updatedDict = {}
        
        for addr in self.robotIPaddresses.keys():
            timestamp = self.robotIPaddresses[addr]
            now = self.get_clock().now()

            durationInSeconds = (now - timestamp).nanoseconds * 1.0 / 1e9
            
            if(durationInSeconds < self.paramInt('robot_timeout')):
                updatedDict[addr] = timestamp
            else:
                self.get_logger().info(f"Removed robot with IP {addr}")

        
        # Log
        if len(updatedDict) < len(self.robotIPaddresses):
            self.get_logger().info(f"-> Now at {len(updatedDict)} robot(s)")
        
        # Update values
        self.robotIPaddresses = updatedDict




def main(args=None):
    rclpy.init(args=args)
    node = DynamicOperator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()