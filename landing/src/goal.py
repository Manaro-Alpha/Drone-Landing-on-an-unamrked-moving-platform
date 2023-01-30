#!/usr/bin/env python
from click import command
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

def set_goal():
    command = PoseStamped()
    #command.pose = Pose()
    #command.header = Header()
    #rate = rospy.Rate(5)
    command.pose.position.x = 6
    command.pose.position.y = 0
    command.pose.position.z = 0
    #command.he = rospy.Time.now()
    command.header.frame_id = 'goal'
    command.header.stamp = rospy.Time.now()
    pub.publish(command)

if __name__ == '__main__':
    rospy.init_node('goal_loc',anonymous=True)
    pub = rospy.Publisher('/goal_location',PoseStamped,queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        set_goal()
        #rate.sleep()
        #pub.publish(command)