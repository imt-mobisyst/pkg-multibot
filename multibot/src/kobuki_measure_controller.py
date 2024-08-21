#!/usr/bin/python3
import rclpy


from include.kobuki_controller import KobukiController

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8
from include.tasks import GoalPoseTask
from multibot_interfaces.srv import WifiMeasure

class KobukiMeasureController(KobukiController):

    def __init__(self):
        super().__init__('kobuki_measure_controller')

        # Init turtlesim specific subscriptions
        # -> Goal poses
        self.create_subscription(PoseStamped, '/goal_pose', self.goalPose_callback, 10)

        # Init measure
        # -> Measure service client
        self.client = self.create_client(WifiMeasure, 'wifi_measure')
        # -> Init measure timer
        self.create_timer(1.0, self.measureWifi)



    def goalPose_callback(self, msg:PoseStamped):
        # Save position        
        self.targetPos = msg.pose.position

        # Calculate bid and send it to the operator
        self.sendBid(self.targetPos)

    def assignedRobot_callback(self, msg:Int8):
        assignedRobotId = int(msg.data)
        self.get_logger().info(f"Turtle {assignedRobotId} goes to the target")

        if(assignedRobotId == self.robotId() and self.targetPos is not None):# If the robot assigned is this one, tell it to move
            task = GoalPoseTask(self.targetPos)
            self.queue.addTask(task)
        
        # If the robot assigned is not this one OR the position has been added to queue => reset
        self.targetPos = None


    def measureWifi(self):
        print("MEASURE")
        # Create request
        request = WifiMeasure.Request()
        request.position = self.getRobotPosition()
        request.publish = True

        # Send request
        self.client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = KobukiMeasureController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()