#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Int32
import serial, time, re

ser_front = serial.Serial(\
    port = '/dev/ttyUSB0', \
    baudrate = 115200,
)
def callback(msg):
    print(msg.data)

if __name__ == '__main__':
    rospy.init_node('aigo_serial_sub')
    lwheel_desired_rate_sub = rospy.Subscriber('lwheel_desired_rate', Int32, callback)
    rwheel_desired_rate_sub = rospy.Subscriber('rwheel_desired_rate', Int32, callback)
    lwheel_desired_rate = lwheel_desired_rate_sub.data
    rwheel_desired_rate = rwheel_desired_rate_sub.data
    while not rospy.is_shutdown():
        time.sleep(0.1)
        ser_front.write(lwheel_desired_rate, b',', rwheel_desired_rate)
    ser_front.close()