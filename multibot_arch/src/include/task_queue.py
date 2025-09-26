from include.tasks import Task
from include.helpers import euclideanDistance

class TaskQueue():
    def __init__(self):
        self.tasks:list[Task] = []


    def addTask(self, task:Task):
        """Add task to the queue"""
        self.tasks.append(task)

    def length(self):
        """Number of tasks in the queue"""
        return len(self.tasks)
    
    def __len__(self):
        return self.length()
    
    def isEmpty(self):
        """Are there any tasks in the queue"""
        return self.length() == 0
        
        
    def totalDistance(self) -> float:
        """Get the total distance between task points in the queue"""
        if self.isEmpty():
            return 0.0
        
        # Get all points the robot will have to travel    
        points = []
        for task in self.tasks:
            points += task.getPoints() 

        # Calculate the distance as the crow flies that the robot will have to travel
        dist = 0
        for i in range(len(points)-1):
            dist += euclideanDistance(points[i+1], points[i])


        return dist
    
    def lastPoint(self):
        """"Get last task point in the queue"""
        return self.tasks[-1].points[-1]
    
    def firstPoint(self):
        """"Get last task point in the queue"""
        return self.tasks[0].points[0]
    
    def removeFirstPoint(self):
        """Remove the first point in the queue (from the first task)"""
        self.tasks[0].removeFirstPoint()

    def removeEmptyTasks(self):
        # Check if there are no points remaining in the task
        if self.tasks[0].nbPoints() == 0 :
            self.tasks.pop(0) # Remove task if all points have been achieved
    