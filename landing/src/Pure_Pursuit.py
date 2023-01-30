#!/usr/bin/env python
from pyexpat.errors import XML_ERROR_UNEXPECTED_STATE
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
z_lim = 11111111111111111111111111111
x = 1
i = 0
def initial_pos():
    global x
    global i
    #global z_lim
    global x_pos
    global y_pos
    global z_pos
    global command
    command = PoseStamped()
    command.pose =  Pose()
    command.header = Header()
    command.header.stamp = rospy.Time.now()
    rate = rospy.Rate(10)
    now = rospy.get_rostime()
    rate = rospy.Rate(10) # Hz
    rate1 = rospy.Rate(5)
    if x==1:
      print ("Arming")
      result = arm_sr(value=True)
      print (result)
      print ("Setting Offboard Mode")
      result = mod_sr(custom_mode="OFFBOARD")
      print (result)
      x=0
    while i in range(0,100):
        result = mod_sr(custom_mode="OFFBOARD")
        x=0
        pos=set_p()
        pos.header.stamp = rospy.Time.now()
        pos_pub.publish(pos)
        i +=1
        rate.sleep()
    while not rospy.is_shutdown():
        sub = rospy.Subscriber("/goal_location", PoseStamped, pid_callback)
        sub1 = rospy.Subscriber("/mavros/local_position/pose", PoseStamped,callback)
        if(z_lim<.2):
            break
        rate.sleep()
def callback(data):
        global x_pos
        global y_pos
        global z_pos
       
        x_pos = data.pose.position.x        
        y_pos = data.pose.position.y
        z_pos = data.pose.position.z
def set_p():
    pos=PoseStamped()
    pos.header = Header()
    #enter position for the drone
    pos.pose.position.x= 0
    pos.pose.position.y= 0
    pos.pose.position.z= 8 
    return pos

def pose_call(data):
    po=PoseStamped()
    po.header = Header()
    po.pose = data.pose.pose
    pos_pub.publish(po)
    
    
    
def pid_callback(data):
    #orientation_q = data.pose.orientation
    position_p = data.pose.position

    # alph = Pose()
    # alph.position.x     = position_p.y
    # alph.position.y     = position_p.x
    # alph.position.z     = position_p.z
    # alph.orientation.x  = orientation_q.x
    # alph.orientation.y  = orientation_q.y
    # alph.orientation.z  = orientation_q.z
    # alph.orientation.w  = orientation_q.w
    # print(alph)

    x = round(position_p.x,4)
    y = round(position_p.y,4)
    z = round(position_p.z,4)
    #oreantation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    #(roll, pitch, yaw) = euler_from_quaternion(oreantation_list)
    command = PoseStamped()
    command.header = Header()

    const = 0.01
    theta = math.atan2(x,y)
    z_next = z - const*z
    S = math.sqrt((x - x_pos)**2 + (y - y_pos)**2)
    T = math.sqrt((x - x_pos)**2 + (y - y_pos)**2 + (z - z_pos)**2)
    R = (T**2)/(2*S)
    
    s_sol = R - math.sqrt(R**2 - z_next**2) 
    # print(T)
    # print(S)
    # print(R)
    # print(math.sin(theta))
    # print(math.cos(theta))

    command.pose.position.x = s_sol * math.sin(theta)
    command.pose.position.y = s_sol * math.cos(theta)
    command.pose.position.z = z_next
    # publishing the values
    i = 0
    print (command)
    pos_pub.publish(command)
    
if __name__ == '__main__':
    rospy.init_node('PureP')
    pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped,queue_size=10)
    arm_sr = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    mod_sr = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    initial_pos()