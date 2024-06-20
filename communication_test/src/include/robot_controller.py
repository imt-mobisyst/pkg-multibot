#!/usr/bin/python3
from os import getenv
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

from communication_test_interfaces.msg import AuctionBid
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Int8
from visualization_msgs.msg import Marker
from nav2_msgs.action import NavigateToPose

from include.helpers import getQuaternionFromEuler, euclideanDistance
from include.task_queue import TaskQueue
from nav2_msgs.action._navigate_to_pose import NavigateToPose_GetResult_Response
from action_msgs.msg import GoalStatus

class RobotController(Node):

    def __init__(self, name='robot_controller'):
        super().__init__(name)

        # Declare ROS parameters
        self.declare_parameter('robot_id', 1)
        self.get_logger().info(f"Robot {self.paramInt('robot_id')} started")

        # Log DDS server
        if(getenv('ROS_DISCOVERY_SERVER') is not None):
            self.get_logger().info("RUNNING on DDS \"" + getenv('ROS_DISCOVERY_SERVER') + "\"")

        # Init subscriptions
        self.create_subscription(Int8, '/assignedRobot', self.assignedRobot_callback, 10)

        # Init publishers
        self.auctionBidPublisher = self.create_publisher(AuctionBid, '/auctionBid', 10)
        self.markerPublisher = self.create_publisher(Marker, '/robot_marker', 10)

        # Init action client
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Init loop
        self.create_timer(1/60.0, self.loop)

        # Init variables
        self.targetPos = None

        self.queue = TaskQueue()
        self.robotMoving = False

        # Visualization
        self.markerScale = 1.0

    def paramInt(self, name):
        return self.get_parameter(name).get_parameter_value().integer_value

    def pose_callback(self, msg):
        raise NotImplementedError

    def getRobotPosition(self) -> Point:
        raise NotImplementedError

    def getRobotAngle(self) -> float:
        raise NotImplementedError



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
        marker.scale.x = 0.6 * self.markerScale
        marker.scale.y = 0.2 * self.markerScale
        marker.scale.z = 0.2 * self.markerScale

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position = self.getRobotPosition()
        marker.pose.orientation = getQuaternionFromEuler(0, 0, self.getRobotAngle())

        self.markerPublisher.publish(marker)



        
    def sendBid(self, targetPos:Point):

        if self.queue.isEmpty():
            # If nothing in queue, total distance to travel = distance from robot to target
            totalDistance = euclideanDistance(self.getRobotPosition(), targetPos)
        else:
            # If points in queue, add distances : to first point in queue, between points in queue, after last point in queue
            distanceRobotToFirstPoint   = euclideanDistance(self.getRobotPosition(), self.queue.firstPoint())
            distanceLastPointToNewPoint = euclideanDistance(self.queue.lastPoint(), targetPos)

            totalDistance = distanceRobotToFirstPoint + self.queue.totalDistance() + distanceLastPointToNewPoint

        # Send it to the operator to see which robot gets assigned to it
        res = AuctionBid()
        res.robot_id = self.paramInt('robot_id') # Send the domain ID that the robot is currently in
        res.distance = totalDistance

        self.auctionBidPublisher.publish(res)


    def assignedRobot_callback(self, msg:Int8):
        raise NotImplementedError

    
            
    def send_goal(self):
        if self.queue.isEmpty():
            return

        # Create msg
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position = self.queue.firstPoint() # Go to the first element in the queue

        # Connect to action server
        self.action_client.wait_for_server()

        # Send action with callback
        action = self.action_client.send_goal_async(goal_msg)
        action.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future:Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('!!! GOAL REJECTED !!!')
            self.queue.removeFirstPoint() # Remove first pos from the queue
            return

        self.get_logger().info('GOAL ACCEPTED')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future:Future):
        result:NavigateToPose_GetResult_Response = future.result()
        self.get_logger().info(f'Result: Error code {result.result.error_code} | Status {result.status}')
                

        # If successfully arrived at the point
        if(result.result.error_code == 0 and result.status == GoalStatus.STATUS_SUCCEEDED):
            self.queue.removeFirstPoint() # Remove first pos from the queue
            self.robotMoving = False

            # Send event to catch in child classes if needed
            self.get_logger().info('GOAL SUCEEDED')
            self.goalSucceeded()

            # Clean tasks
            self.queue.removeEmptyTasks()
        else: # Probably received status 6 : Goal aborted

            if not self.queue.isEmpty():
                self.get_logger().info('Retrying...')
                self.send_goal()
            else:
                # Send events to catch in child classes if needed                
                self.get_logger().info('DONT RETRY')
                self.goalFailed()
            
        
    def goalSucceeded(self):
        pass
    def goalFailed(self):
        pass


    def loop(self):
        # Don't do anything if empty queue
        if self.queue.isEmpty():
            return
        
        # Go to each point in the queue
        if not self.robotMoving:
            self.robotMoving = True
            # If not already moving, go to first position in the queue
            self.send_goal()
