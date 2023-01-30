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

KpP = 0.25 #Kp Pose
KiP = 0.01 #Ki Pose
KdP = 0.01 #Kd Pose
KpV = 0.25 #Kp Velocity
KiV = 0.01 #Ki Velocity
KdV = 0.01 #Kd Velocity
x = 0 #x pose
y = 0 #y pose
z = 0 #z pose
x_goal = 0 #x goal pose
y_goal = 0 #y goal pose
z_goal = 0 #z goal pose
Vx = 0 #Vx velocity
Vy = 0 #Vy velocity
Vz = 0 #Vz velocity
Vx_goal = 0 #Vx goal velocity
Vy_goal = 0 #Vy goal velocity
Vz_goal = 0 #Vz goal velocity
integral_velocity = np.array([0.,0,0])
integral_pose = np.array([0.,0,0])
prev_err_pose = np.array([0.,0,0])
prev_err_velocity = np.array([0.,0,0])

def PID():
    global KpP
    global KiP
    global KdP
    global KpV
    global KiV
    global KdV
    global x
    global y
    global z
    global x_goal
    global y_goal
    global z_goal
    global Vx
    global Vy
    global Vz
    global Vx_goal
    global Vy_goal
    global Vz_goal
    global integral_velocity
    global integral_pose

    set_mode_client(base_mode=0, custom_mode="OFFBOARD")
    distance = math.sqrt(math.sqrt(((x - x_goal)**2) + ((y - y_goal)**2) + ((z - z_goal)**2)))
    loco = TwistStamped()

    if Vx_goal - Vx > 0.1:
        current_error_velocity = np.array([Vx_goal - Vx,Vy_goal - Vy,Vz_goal - Vz])
        P_v = KpV*current_error_velocity
        integral_velocity += current_error_velocity
        I_v = KiV*integral_velocity
        differential_v = current_error_velocity - prev_err_velocity
        D_v = KdV*differential_v
        prev_err_velocity = current_error_velocity
    else:
        P_v = np.array([0.,0,0])
        I_v = np.array([0.,0,0])
        D_v = np.array([0.,0,0])
    
    loco.header.seq = rospy.Time.now
    loco.header.frame_id = 'vel'
    
    loco.twist.linear.x = P_v[0] + I_v[0] + D_v[0]
    loco.twist.linear.y = P_v[1] + I_v[1] + D_v[1]
    loco.twist.linear.z = P_v[2] + I_v[2] + D_v[2]

    if distance > 0.1:
        current_error_pose = np.array([x_goal - x,y_goal - y,z_goal - z]) + current_error_velocity
        P_p = KpP*current_error_pose
        integral_pose += current_error_pose
        I_p = KiP*integral_pose
        differential_p = current_error_pose - prev_err_pose
        D_p = KdP*differential_p
        prev_err_pose = current_error_pose
    else:
        P_p = np.array([0.,0,0])
        I_p = np.array([0.,0,0])
        D_p = np.array([0.,0,0])

    loco.twist.linear.x = P_p[0] + I_p[0] + D_p[0]
    loco.twist.linear.y = P_p[1] + I_p[1] + D_p[1]
    loco.twist.linear.z = P_p[2] + I_p[2] + D_p[2]

    pub.publish(loco)

def callback_pose(data):
    global x
    global y
    global z

    x = data.pose.position.x
    y = data.pose.position.y
    z = data.pose.position.z

def callback_velocity(data):
    global Vx
    global Vy
    global Vz

    Vx = data.twist.linear.x
    Vy = data.twist.linear.y
    Vz = data.twist.linear.z

def plat_pose_callback(data):
    global x_goal
    global y_goal
    global z_goal
    global Vx_goal
    global Vy_goal
    global Vz_goal

    x_goal = data.pose.pose.position.x
    y_goal = data.pose.pose.position.y
    z_goal = data.pose.pose.position.z

    Vx_goal = data.twist.twist.linear.x
    Vy_goal = data.twist.twist.linear.y
    Vz_goal = data.twist.twist.linear.z

if __name__ == '__main__':
    rospy.init_node('PID_v2')
    pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
    sub_pose = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback_pose)
    sub_velocity = rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, callback_velocity)
    sub_plat_pose = rospy.subscriber('/husky_velocity_controller/odom',Odometry,plat_pose_callback)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        PID()
        rate.sleep()