#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8
from test_multi_id_interfaces.msg import DistanceToTarget

class Operator(Node):

    nbRobots = 2

    def __init__(self):
        super().__init__('operator')

        # Declare ROS parameters
        self.declare_parameter('nb_robots', 2)

        print(f"Init node with {self.paramInt('nb_robots')} robot(s)")

        # Init subscriber
        self.create_subscription(DistanceToTarget, '/distanceToTarget', self.turtleDistance_callback, 10)

        # Init publisher
        self.assignedRobotPublisher = self.create_publisher(Int8, '/assignedRobot', 10)
        
        
        self.distanceDict = {}


    def turtleDistance_callback(self, msg:DistanceToTarget):
        # Add value to dict
        self.distanceDict[msg.robot_id] = msg.distance

        # Check if all distances have been received
        if(len(self.distanceDict) == self.paramInt('nb_robots')):

            sortedDistances = sorted(self.distanceDict.items(), key=lambda x:x[1])
            closestRobotId = sortedDistances[0][0]

            print(f"closest {closestRobotId}")

            # Tell robots which one is assigned the task
            res = Int8()
            res.data = int(closestRobotId)
            self.assignedRobotPublisher.publish(res)

            # Reset values
            self.distanceDict.clear()


    def paramInt(self, name):
        return self.get_parameter(name).get_parameter_value().integer_value







def main(args=None):
    rclpy.init(args=args)
    node = Operator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()