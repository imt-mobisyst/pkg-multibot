#!/usr/bin/python3
import rclpy
from time import sleep
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance



class StageInitPose(Node):

    def __init__(self, name='stage_init_pose'):
        super().__init__(name)

        # Declare ROS parameters
        self.declare_parameter('nb_robots', 3)
        self.declare_parameter('wait', 1)

        self.get_logger().info(f"Init node with {self.paramInt('nb_robots')} robot(s)")

        # Wait for other nodes to start
        self.get_logger().info(f"Waiting {self.paramInt('wait')} second(s)")
        sleep(self.paramInt('wait'))

        self.initialPosePublishers = {}
        for i in range(self.paramInt('nb_robots')):
            # Init subscriptions
            self.create_subscription(Odometry, f"/robot_{i}/ground_truth", lambda msg, robot_id=i: self.odomCallback(msg, robot_id), 10)

            # Init publishers
            self.initialPosePublishers[i] = self.create_publisher(PoseWithCovarianceStamped, f'/robot_{i}/initialpose', 10)

        self.robotPoses = {}

    def odomCallback(self, msg:Odometry, robot_id:int):
        # Create object
        newPose = PoseWithCovarianceStamped()
        newPose.header.frame_id = "map"
        newPose.header.stamp = self.get_clock().now().to_msg()
        newPose.pose = msg.pose

        # Save data
        self.robotPoses[robot_id] = newPose

        # Check if received all robots poses
        if len(self.robotPoses) == self.paramInt('nb_robots') :
            # Publish all initialposes
            for i in range(len(self.robotPoses)):
                self.get_logger().info(f"Set initial pose for robot_{i} to (x: {self.robotPoses[i].pose.pose.position.x}, y: {self.robotPoses[i].pose.pose.position.y}, z: {self.robotPoses[i].pose.pose.position.z})")
                self.initialPosePublishers[i].publish(self.robotPoses[i])


            # Stop node
            self.destroy_node()
            exit()

            

    def paramInt(self, name):
        return self.get_parameter(name).get_parameter_value().integer_value




def main(args=None):
    rclpy.init(args=args)
    node = StageInitPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()