# import rospy
# from geometry_msgs.msg import TwistStamped,PoseStamped

# rospy.init_node('test_node')
# pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',TwistStamped,queue_size=10)

# command = TwistStamped()

# command.twist.linear.z = -1
# command.header.stamp = rospy.Time.now()
# command.header.frame_id = 'test'
# rate = rospy.Rate(10)
# while not rospy.is_shutdown():
#     pub.publish(command)
#     rate.sleep()

import numpy as np

a = np.array([1,2,3])
print(a+1)