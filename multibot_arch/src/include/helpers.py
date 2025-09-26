from geometry_msgs.msg import Quaternion, Point
import numpy as np
import math as m

def getQuaternionFromEuler(roll, pitch, yaw):

    q = Quaternion()

    q.x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    q.y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    q.z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    q.w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    
    return q

def getYaw(q:Quaternion):
    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return m.atan2(t3, t4)
 
def getEulerFromQuaternion(q:Quaternion):
    t0 = +2.0 * (q.w * q.x + q.y * q.z)
    t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll_x = m.atan2(t0, t1)
    
    t2 = +2.0 * (q.w * q.y - q.z * q.x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = m.asin(t2)
    
    yaw_z = getYaw(q)
    
    return {
        'roll': roll_x,
        'pitch': pitch_y,
        'yaw': yaw_z
    }

def createPoint(x,y,z=0) -> Point:
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    return p



def euclideanDistance(targetPos:Point, initPose:Point):
    return np.sqrt(np.square(targetPos.y - initPose.y) + np.square(targetPos.x - initPose.x))