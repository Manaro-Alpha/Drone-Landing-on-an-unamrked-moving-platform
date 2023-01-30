#!/usr/bin/env python
from pyexpat.errors import XML_ERROR_UNEXPECTED_STATE
from zlib import Z_NO_COMPRESSION
import rospy 
import time
import math
import matplotlib.pyplot as plt
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import TwistStamped,Twist,Pose,PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header,String,Int8
from tf.transformations import euler_from_quaternion,quaternion_from_euler
import numpy as np
x_pos = 0
y_pos = 0
z_pos = 0
x_goal = 0
y_goal = 0
z_goal = 0
command = 0
a = 0

def PurePursuit():
    global command
    global x_pos
    global y_pos
    global z_pos
    global x_goal
    global y_goal
    global z_goal

    command = PoseStamped()
    #command.pose = Pose()
    #command.header = Header()
    command.header.stamp = rospy.Time.now()
    command.header.frame_id = 'Path'

    L = math.sqrt(((x_pos - x_goal)**2) + ((y_pos - y_goal)**2) + ((z_pos - z_goal)**2))
    S = math.sqrt(((x_goal - x_pos)**2) + ((y_goal - y_pos)**2))
    print(S)
    R = (L**2)/(2*S)
    
    z_next = z_pos - 0.4
    s_sol = R - math.sqrt(R**2 - z_next**2)
    theta = math.atan2(x_goal-x_pos,y_goal-y_pos)
    
    command.pose.position.x = s_sol*math.sin(theta)
    command.pose.position.y = s_sol*math.cos(theta)
    command.pose.position.z = z_next
    pub.publish(command)
    
def local_pos_callback(data):
    global x_pos
    global y_pos
    global z_pos

    x_pos = data.pose.position.x
    y_pos = data.pose.position.y
    z_pos = data.pose.position.z
    print(data)

def goal_pos_callback(data):
    global x_goal
    global y_goal
    global z_goal

    x_goal = data.pose.position.x
    y_goal = data.pose.position.y
    z_goal = data.pose.position.z
    print(data)

#def arm():
#    global a
#    i = 0
#    if a==1:

#        print ("Arming")
#        result = arm_sr(value=True)
#        print (result)
#        print ("Setting Offboard Mode")
#        result = mod_sr(custom_mode="OFFBOARD")
#        print (result)
#        a=0
#    while i in range(0,200):
#        result = mod_sr(custom_mode="OFFBOARD")
#        a=0
#        pos=set_p()
#        pos.header.stamp = rospy.Time.now()
#        pub.publish(pos)
#        i +=1
#        rate.sleep()
#    i = 0

#def set_p():
#    pos=PoseStamped()
#    pos.header = Header()
#    #enter position for the drone
#    pos.pose.position.x= 0
#    pos.pose.position.y= 0
#    pos.pose.position.z= 8
#    return pos

if __name__ == '__main__':
    rospy.init_node('PurePursuit')
    pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    sub_goal_pos = rospy.Subscriber('/goal_location',PoseStamped,goal_pos_callback)
    #arm_sr = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    #mod_sr = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    rate = rospy.Rate(10)
    #arm()
    while not rospy.is_shutdown():
        #arm()
        sub_goal_pos = rospy.Subscriber('/goal_location',PoseStamped,goal_pos_callback)
        sub_local_pos = rospy.Subscriber('/mavros/local_position/pose',PoseStamped,local_pos_callback)
        #sub_goal_pos = rospy.Subscriber('/goal_location',PoseStamped,goal_pos_callback)
        PurePursuit()
        rate.sleep()