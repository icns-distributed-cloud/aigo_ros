#!/usr/bin/env python

import rospy
import 
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import NavSatFix
import serial, time, re

ser_front = serial.Serial(\
    port = '/dev/ttyAMA2', \
    baudrate = 115200,
)

def gps_callback(msg):
    global lat
    global long


def left_callback(msg):
    lwheel_desired_rate = msg.data

def right_callback(msg):
    rwheel_desired_rate = msg.data

if __name__ == '__main__':
    rospy.init_node('aigo_serial_sub')
   # lwheel_desired_rate = 0
  #  rwheel_desired_rate = 0
    gps_sub = rospy.Subscriber('/fix', NavSatFix, gps_callback)
    with 
    #lwheel_desired_rate_sub = rospy.Subscriber('lwheel_desired_rate', Int32, left_callback)
    #rwheel_desired_rate_sub = rospy.Subscriber('rwheel_desired_rate', Int32, right_callback)
 #   stm32_msg = str(lwheel_desired_rate)+','+str(rwheel_desired_rate)
    
    #while not rospy.is_shutdown():
    #    time.sleep(0.1)
    #    ser_front.write(stm32_msg)
#	ser_front.flushInput()
#    	ser_front.flushOutput()
#ser_front.close()
    
