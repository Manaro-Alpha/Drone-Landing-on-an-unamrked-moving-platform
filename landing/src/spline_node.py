import rospy
from scipy.interpolate import CubicSpline
import numpy as np
import matplotlib.pyplot as plt
import math
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import TwistStamped,Twist,Pose,PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header,String,Int8
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from landing.msg import Waypoint

x0 = 0
y0 = 0
x1 = 0
y1 = 0
x2 = 0
y2 = 0

def getSpline(xi,xf,yi,yf):
    global x1
    global y1

    x1 = math.sqrt(((xi**2) + (xf**2))/2)
    y1 = math.sqrt(((yi**2) + (yf**2))/2)
    x = [xi,x1,xf]
    y = [yi,y1,yf]

    return CubicSpline()

def path():
    global x1
    global y1
    global x0
    global y0
    global x2
    global y2

    path = getSpline(x0,y0,x2,y2)
    path_x = np.linspace(x0,x1,10)
    path_y = path(path_x)

    Waypoint.x_coords = path_x
    Waypoint.y_coords = path_y

    

if __name__ == '__main__':
    rospy.init_node('Waypoint')
    pub = rospy.Publisher('/waypoint',Waypoint,queue_size=10)

    while not rospy.is_shutdown():
        path()
