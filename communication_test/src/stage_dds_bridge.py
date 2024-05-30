#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from os import getenv

from nav_msgs.msg import OccupancyGrid, Path
from map_msgs.msg import OccupancyGridUpdate

class StageSimBridge(Node):
    def __init__(self, name='stage_sim_bridge'):
        super().__init__(name)

        self.declare_parameter('robot_id', 0)

        self.get_logger().info(f"Bridge for robot {self.paramInt('robot_id')} has DDS : {getenv('ROS_DISCOVERY_SERVER')}")

        ns = f"robot_{self.paramInt('robot_id')}"

        self.topics = {
            f'{ns}/map' : {
                'type': OccupancyGrid,
                'qos': QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1
                )
            },
            f'{ns}/local_costmap/costmap' : {
                'type': OccupancyGrid,
                'qos' :QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1
                )
            },
            f'{ns}/local_costmap/costmap_updates' : {
                'type': OccupancyGridUpdate,
            },
            f'{ns}/transformed_global_plan': {
                'type': Path,
            },
        }
        

        self.publishersDict = {}

        defaultQos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        for topic in self.topics.keys():
            type = self.topics[topic]['type']
            targetTopicName = self.topics[topic]['target'] if 'target' in self.topics[topic] else topic
            qos = self.topics[topic]['qos'] if 'qos' in self.topics[topic] else defaultQos

            # Init subscriber
            self.create_subscription(type, topic, lambda msg, topic=topic : self.callback(msg, topic), qos)

            # Init publisher
            self.publishersDict[topic] = self.create_publisher(type, targetTopicName, qos)
        
        
    def callback(self, msg, topicName):
        self.publishersDict[topicName].publish(msg)

    def paramInt(self, name):
        return self.get_parameter(name).get_parameter_value().integer_value
    
    
def main(args=None):
    rclpy.init(args=args)
    node = StageSimBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()