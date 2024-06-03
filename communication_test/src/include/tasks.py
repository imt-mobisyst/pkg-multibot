

from enum import Enum
from geometry_msgs.msg import Point
 
 
class TaskType(Enum):
    GOAL_POSE = 1
    STORE = 2
    RETRIEVAL = 3


class Task:
    nbTasks = 0
    
    def __init__(self, type:TaskType, points:list[Point]):
        self.type = type
        self.points = points

        # Automatic ID generation
        self.taskID = Task.nbTasks
        Task.nbTasks += 1


    def getPoints(self):
        """Get all points in the task"""
        return self.points
    
    def nbPoints(self):
        """Get the number of points in the task"""
        return len(self.points)
    
    def removeFirstPoint(self):
        """Remove first point of the task"""
        self.points.pop(0)


class GoalPoseTask(Task):
    def __init__(self, point:Point):
        super().__init__(TaskType.STORE, [point])


from include.package import Package

class StoreTask(Task):
    def __init__(self, package:Package):
        points = [package.position, package.depositSpot()]

        super().__init__(TaskType.STORE, points)
        
        # Save package
        self.package = package

class RetrieveTask(Task):
    def __init__(self, package:Package):
        points = [package.position, Package.retrievalSpot]

        super().__init__(TaskType.RETRIEVAL, points)

        # Save package
        self.package = package

