#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header, String , Int8
import numpy as np

x_screen = 433
y_screen = 344
z_screen = 0
z_world = 1

def getCoords_callback(data):
    global x_screen
    global y_screen
    global z_screen

    P = data.P
    x_world = (x_screen - P[2]) * z_world / P[0]
    y_world = (y_screen - P[6]) * z_world / P[5]

    X = np.array([x_world,y_world])

    print(X)

if __name__ == '__main__':
    rospy.init_node('Coords')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        sub = rospy.Subscriber('/iris/usb_cam/camera_info',CameraInfo,getCoords_callback)
        rate.sleep()

