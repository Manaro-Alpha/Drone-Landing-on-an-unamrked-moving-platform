#!/usr/bin/env python
import rospy 
import time
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import TwistStamped,Twist,Pose,PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, String, Int8
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import math
Kp = 0.25
Kd = 0.001
Ki = 0.001
x = 0
y = 0
z = 0
x_goal = 0
y_goal = 0
z_goal = 2
integral = np.array([0.,0,0])
prev_err = np.array([0.,0,0])
a = 1 

def PID():
    global x
    global y
    global z
    global x_goal
    global y_goal
    global z_goal
    global Kp
    global Kd
    global Ki
    global integral
    global prev_err

    distance = math.sqrt(math.sqrt(((x - x_goal)**2) + ((y - y_goal)**2) + ((z - z_goal)**2)))
    loco = TwistStamped()

    if distance > 0.1:
        current_error = np.array([(x_goal - x),(y_goal - y),(z_goal - z)])
        P = Kp*current_error
        integral += current_error
        I = Ki*integral
        differential = current_error - prev_err
        D = Kd*differential
        prev_err = current_error
    else:
        P = np.array([0,0,0])
        I = np.array([0,0,0])
        D = np.array([0,0,0])

    loco.header.seq = rospy.Time.now
    loco.header.frame_id = 'vel'
    
    loco.twist.linear.x = P[0] + I[0] + D[0]
    loco.twist.linear.y = P[1] + I[1] + D[1]
    loco.twist.linear.z = P[2] + I[2] + D[2]

    
    #prev_err = current_error

    pub.publish(loco)
    print(loco)

def pos_callback(data):
    global x
    global y
    global z

    x = data.pose.position.x
    y = data.pose.position.y
    z = data.pose.position.z

def waypoint_callback(data):
    global x_goal
    global y_goal
    global z_goal

    x_goal = data.pose.position.x
    y_goal = data.pose.position.y
    z_goal = data.pose.position.z

if __name__ == '__main__':
    rospy.init_node('PID')
    pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',TwistStamped,queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        sub_pos = rospy.Subscriber('/mavros/local_position/pose',PoseStamped,pos_callback)
        waypoint_pos = rospy.Subscriber('/mavros/setpoint_position/local',PoseStamped,waypoint_callback)
        PID()
        rate.sleep()



