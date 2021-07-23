#!/usr/bin/env python

import serial, time, rospy, re
from std_msgs.msg import Float32MultiArray

ser_front = serial.Serial( \
    port='/dev/ttyUSB0', \
    baudrate=115200,
)

def is_int(s):
    try:
        int(s)
        return True
    except ValueError:
        return False

def read_sensor(): # receive each side of encoder count from STM32F4 Discovery & convert to velocity
    tick_data = []
    velocity_data = []
    serial_data = ser_front.readline()
    ser_front.flushInput()
    ser_front.flushOutput()
    # string -> integer
    serial_data = serial_data.split(',')
    for i, v in enumerate(serial_data):
        if is_int(v) == True:
            tick_data.append(int(v))
    for i, v in enumerate(tick_data):
        if v > 8191: # reverse
            v -= 16383
        velocity_data.append(float(v) * 3.14 * 0.0085 / 0.1 / 330) # meter per second
    msg.data = velocity_data

if __name__ == '__main__':

    rospy.init_node('aigo_serial_pub', anonymous=False) # initialize node
    pub = rospy.Publisher('velocity', Float32MultiArray, queue_size=1)

    msg = Float32MultiArray() # message type
    while not rospy.is_shutdown():
        read_sensor() 
        pub.publish(msg) # publish a message
        time.sleep(0.1)
    
    ser_front.close()
