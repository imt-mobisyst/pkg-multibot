#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import numpy as np

from communication_test_interfaces.msg import DistanceToTarget
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Int8
from visualization_msgs.msg import Marker
from nav2_msgs.action import NavigateToPose

class RobotController(Node):

    def __init__(self, name='robot_controller'):
        super().__init__(name)

        # Declare ROS parameters
        self.declare_parameter('robot_id', 1)
        self.get_logger().info(f"Robot {self.paramInt('robot_id')} started")

        # Init subscriptions
        self.create_subscription(PoseStamped, '/goal_pose', self.target_callback, 10)
        self.create_subscription(Int8, '/assignedRobot', self.assignedRobot_callback, 10)

        # Init publishers
        self.distanceToTargetPublisher = self.create_publisher(DistanceToTarget, '/distanceToTarget', 10)
        self.markerPublisher = self.create_publisher(Marker, '/turtle_marker', 10)

        # Init action client
        self.action_client = ActionClient(self, NavigateToPose, 'moveToPose')

        # Init loop
        self.create_timer(1/60.0, self.loop)

        # Init variables
        self.targetPos = None

        self.queue = []
        self.robotMoving = False

    def paramInt(self, name):
        return self.get_parameter(name).get_parameter_value().integer_value

    def pose_callback(self, msg):
        raise NotImplementedError

    def getRobotPosition(self) -> Point:
        raise NotImplementedError

    def getRobotAngle(self) -> float:
        raise NotImplementedError



    def get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]


    def publishPoseMarker(self):
        # Publish marker for vizualisation in rviz
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = self.get_clock().now().to_msg()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 0
        marker.ns = "robot" + str(self.paramInt('robot_id'))
        marker.id = self.paramInt('robot_id')

        # Set the scale of the marker
        marker.scale.x = 1.0
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set the pose of the marker
        q = self.get_quaternion_from_euler(0, 0, self.getRobotAngle())
        marker.pose.position = self.getRobotPosition()
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        self.markerPublisher.publish(marker)

    def target_callback(self, msg:PoseStamped):
        # Save position        
        self.targetPos = msg.pose.position

        # Calculate cost (total distance) to go to that position
        initPose = self.getRobotPosition() if len(self.queue) == 0 else self.queue[-1]
        totalDistance = self.totalQueueDistance() + self.euclidean_distance(self.targetPos, initPose)

        # Send it to the operator to see which turtle gets assigned to it
        res = DistanceToTarget()
        res.robot_id = self.paramInt('robot_id') # Send the domain ID that the robot is currently in
        res.distance = totalDistance

        self.distanceToTargetPublisher.publish(res)

    def assignedRobot_callback(self, msg:Int8):
        assignedRobotId = int(msg.data)
        print(f"msg assigned robot {msg}")

        if(assignedRobotId == self.paramInt('robot_id')):# If the robot assigned is this one, tell it to move
            self.queue.append(self.targetPos)
        
        # If the robot assigned is not this one OR the position has been added to queue => reset
        self.targetPos = None

    

    def euclidean_distance(self, targetPos, initPose=None):
        """Euclidean distance between current pose and the goal."""
        if initPose is None:
            initPose = self.getRobotPosition()

        return np.sqrt(np.square(targetPos.y - initPose.y) + np.square(targetPos.x - initPose.x))


    def totalQueueDistance(self):
        if len(self.queue) == 0:
            return 0
        
        dist = 0
        positions = [self.getRobotPosition()] + self.queue
        for i in range(len(positions)-1):
            dist += self.euclidean_distance(positions[i+1], positions[i])

        return dist
            
    def send_goal(self):
        # Create msg
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.pose.position = self.queue[0] # Go to the first element in the queue

        # Connect to action server
        self.action_client.wait_for_server()

        # Send action with callback
        action = self.action_client.send_goal_async(goal_msg)
        action.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.error_code))

        # If successfully arrived at the point
        if(result.error_code == 0):
            self.queue.pop(0) # Remove first pos from the queue
            self.robotMoving = False
        



    def loop(self):
        # Don't do anything if empty queue
        if len(self.queue) == 0:
            return
        
        # Go to each point in the queue
        if not self.robotMoving:
            self.robotMoving = True
            # If not already moving, go to first position in the queue
            self.send_goal()






def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()