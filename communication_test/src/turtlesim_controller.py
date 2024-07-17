#!/usr/bin/python3
import rclpy


from include.robot_controller import RobotController
from include.helpers import createPoint

from geometry_msgs.msg import Point, PoseStamped
from turtlesim.msg import Pose
from std_msgs.msg import Int8
from include.tasks import GoalPoseTask

class TurtleController(RobotController):

    def __init__(self):
        super().__init__('turtle_controller')

        # Init turtlesim specific subscriptions
        self.create_subscription(PoseStamped, '/goal_pose', self.goalPose_callback, 10)
        self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)

        # Init variables
        self.pose = Pose()


    def pose_callback(self, msg:Pose):
        # If not moved since last callback, do nothing
        if(self.pose is not None and msg.x == self.pose.x and msg.y == self.pose.y and msg.theta == self.pose.theta):
            return
        
        # If moved, update pose and marker
        self.pose = msg
        self.publishPoseMarker()

    def getRobotPosition(self):        
        return createPoint(self.pose.x, self.pose.y)

    def getRobotAngle(self):
        return self.pose.theta

    def goalPose_callback(self, msg:PoseStamped):
        # Save position        
        self.targetPos = msg.pose.position

        # Calculate bid and send it to the operator
        self.sendBid(self.targetPos)

    def assignedRobot_callback(self, msg:Int8):
        assignedRobotId = int(msg.data)
        self.get_logger().info(f"Turtle {assignedRobotId} goes to the target")

        if(assignedRobotId == self.robotId and self.targetPos is not None):# If the robot assigned is this one, tell it to move
            task = GoalPoseTask(self.targetPos)
            self.queue.addTask(task)
        
        # If the robot assigned is not this one OR the position has been added to queue => reset
        self.targetPos = None




def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()