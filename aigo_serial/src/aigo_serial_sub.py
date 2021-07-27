#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

def callback(msg):
    print(msg.data)

rospy.init_node('aigo_serial_sub')
sub = rospy.Subscriber('velocity', Float32MultiArray, callback)

rospy.spin()