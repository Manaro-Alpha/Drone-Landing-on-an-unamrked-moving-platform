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

z_lim = 11111111111111111111111
x=1
i = 0
def initial_pos():
    global x
    global i
    global z_lim
    global accumulator
    global prev_er
    global comman
    global pose_details
    global ori_details
    comman = [0, 0, 0, 0, 0, 0]
    global command
    command = TwistStamped()
    command.twist =  Twist()
    command.header = Header()
    command.header.stamp = rospy.Time.now()
    accumulator = [0 ,0 ,0 ,0 ,0 ,0]
    prev_er = [0 ,0 ,0 ,0 ,0 ,0]
    rate = rospy.Rate(10)
    now = rospy.get_rostime()
    
    rate = rospy.Rate(5) # Hz
    rate1 = rospy.Rate(10)
    if x==1:
      print ("Arming")
      result = arm_sr(value=True)
      print (result)
      print ("Setting Offboard Mode")
      result = mod_sr(custom_mode="OFFBOARD")
      print (result)
      x=0
    while i in range(0,200):
        result = mod_sr(custom_mode="OFFBOARD")
        x=0
        pos=set_p()
        pos.header.stamp = rospy.Time.now()
        pos_pub.publish(pos)
        i +=1
        rate.sleep()
    i = 0
    while not rospy.is_shutdown():
        sub = rospy.Subscriber('/mavros/setpoint_position/local', PoseStamped, pid_callback)
        sub1 = rospy.Subscriber("/mavros/local_position/pose", PoseStamped,callback)
        if(z_lim<.05):
            break

        rate.sleep()

def callback(data):
    global z_lim
    global pose_details
    global ori_details
    z_lim = data.pose.position.z
    pose_details = data.pose.position
    ori_details = data.pose.orientation

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
    target_pose = data.pose.position
    target_orientation = data.pose.orientation

    x = round(target_pose.x,4)
    y = round(target_pose.y,4)
    z = round(target_pose.z,4)
    orientation_list = [target_orientation.x - ori_details.x, target_orientation.y - ori_details.y, target_orientation.z - ori_details.z, target_orientation.w - ori_details.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    command = TwistStamped()
    command.header = Header()
    error = [x - pose_details.x,y - pose_details.y,z - pose_details.z,roll,pitch,yaw]
    print(error)
        # calculate PID gains
    i = 0
    while i < 3:
        if i == 0 :
            kp = .25
        elif i == 1:
            kp = .25
        elif i == 2:
            kp = .1
        else:
            pass

        P =  (kp * error[i])
        integral = accumulator[i] + error[i] * .1
        I =  .001 *integral #integral 
        derivative = (error[i] - prev_er[i]) * .1 
        D = .001 * derivative #derivative
        accumulator[i] = integral
        prev_er[i] = error[i] 
        comman[i] =  P + D + I
        i +=1 
    if (target_pose.x - pose_details.x < .03) & (target_pose.x - pose_details.x > -.03) :
        comman[0] = 0
    if (target_pose.y - pose_details.y < .03) & (target_pose.y - pose_details.y > -.03) :
        comman[1] = 0
    if (target_pose.z - pose_details.z < .1) & (target_pose.z - pose_details.z > -.1) :
        comman[2] = 0
    #create command Twist msg for Drone
    command.twist.linear.x = comman[0]
    command.twist.linear.y = comman[1]
    command.twist.linear.z = comman[2]
    command.twist.angular.x = 0
    command.twist.angular.y = 0
    command.twist.angular.z = 0


    pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    print(command)
    pub.publish(command)

if __name__ == '__main__':
    rospy.init_node('PID',anonymous=True)
    pos_pub = rospy.Publisher('/mavros/setpoint_position/local',PoseStamped,queue_size=1)
    arm_sr = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    mod_sr = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    initial_pos()