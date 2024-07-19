#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from numpy import cos, sin

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from people_msgs.msg import People, Person

from include.helpers import createPoint, getYaw

class CostmapPublisher(Node):
    def __init__(self, name='costmap_publisher'):
        super().__init__(name)

        self.declare_parameter('robot_id', 0)

        # Create subscriber
        self.create_subscription(Marker, '/robot_marker', self.robotPosCallback, 10)

        # Create publisher
        self.peoplePublisher = self.create_publisher(People, 'global_costmap/people', 10)

        # Create timer
        self.create_timer(0.5, self.publishPeople)

        # Store received robot poses
        self.robotPoses:dict[int, Pose] = {}

        


    def robotPosCallback(self, msg:Marker):
        """Store robot pos if not current one"""
        receivedRobotID = int(msg.ns[5:])

        if receivedRobotID != self.robotId():
            self.robotPoses[receivedRobotID] = msg.pose
            

    
    def publishPeople(self):
        """Publish People msg for nav2 localcostmap social plugin"""
        if len(self.robotPoses) == 0:
            return

        msg = People()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        # Add all other robots' poses to the list
        for id, pose in self.robotPoses.items():
            person = Person()
            person.name = f"robot_{id}"
            
            person.position = pose.position
            angle = getYaw(pose.orientation)
            person.velocity = createPoint(cos(angle), sin(angle))

            msg.people.append(person)

        self.peoplePublisher.publish(msg)
    

    def robotId(self) -> int:
        return self.get_parameter('robot_id').get_parameter_value().integer_value
    



def main(args=None):
    rclpy.init(args=args)
    node = CostmapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()