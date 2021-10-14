#!/usr/bin/env python

import serial, time, rospy, re, os
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import LaserScan, Imu
ser_front = serial.Serial( \
    port='/dev/ttyAMA2', \
    baudrate=115200,
)
def left_callback(msg):
    global left
    left = msg.data

def right_callback(msg):
    global right
    right = msg.data

def calculate_pose():
    global left_vel
    global right_vel
    #in STM32, MOTOR_SPEED = OUTPUT * 15 (MOTOR_SPEED < 9000)
    #turn right
    if(left > right+50):
        left_vel = 600
        right_vel = -600
    #turn left
    elif(left+50 < right):
        left_vel = -600
        right_vel = 600
    elif(left > 0 and right > 0):
        left_vel = 300
        right_vel = 300
    elif(left < 0 and right < 0):
        left_vel = -300
        right_vel = -300
    else:
        left_vel = 0
        right_vel = 0


def is_int(s):
    try:
        int(s)
        return True
    except ValueError:
        return False

def read_serial(): # read a serial data and determine source of the serial
    serial_data = ser_front.readline()
    ser_front.flushInput()
    ser_front.flushOutput()
    if serial_data[0:1] == "e":
        serial_data = serial_data[1:]
        read_encoder(serial_data)

def read_encoder(serial_data): # receive each side of encoder count from STM32F4 Discovery & convert to velocity
    rate_data = []
    encoder_data = []
    # string -> integer
    encoder_data = serial_data.split(',')
    for i, v in enumerate(encoder_data):
        if is_int(v) == True:
            v = int(v)
            if v > 8191: # reverse
                v -= 16383
            v *= 10
            rate_data.append(v)
        else:
            return
    tick_data[0] += rate_data[0]
    tick_data[1] += rate_data[1]

    lwheel_rate_msg.data = float(rate_data[0])
    rwheel_rate_msg.data = float(rate_data[1])

    lwheel_tick_msg.data = tick_data[0]
    rwheel_tick_msg.data = tick_data[1]

# ticks : cumulative encoder ticks, rate : encoder ticks per second
if __name__ == '__main__':
    rospy.init_node('aigo_serial') # initialize node

    tick_data = [0, 0]
    global left
    global right
    left = 0
    right = 0
    #lidar
    #scan_pub = rospy.Publisher('scan', LaserScan, queue_size = 1)
    #scan_msg = LaserScan()
    
    #encoder_ticks
    lwheel_tick_pub = rospy.Publisher('lwheel_ticks', Int32, queue_size = 1)
    rwheel_tick_pub = rospy.Publisher('rwheel_ticks', Int32, queue_size = 1)

    lwheel_rate_pub = rospy.Publisher('lwheel_rate', Float32, queue_size = 1)
    rwheel_rate_pub = rospy.Publisher('rwheel_rate', Float32, queue_size = 1)
        
    lwheel_tick_msg = Int32() # message type
    rwheel_tick_msg = Int32()
    
    lwheel_rate_msg = Float32()
    rwheel_rate_msg = Float32()
    
    lwheel_desired_rate_sub = rospy.Subscriber('lwheel_desired_rate', Int32, left_callback)
    rwheel_desired_rate_sub = rospy.Subscriber('rwheel_desired_rate', Int32, right_callback)
    
    
    #imu_pub = rospy.Publisher('imu', Imu, queue_size=1)
    #imu_msg = Imu()

    while not rospy.is_shutdown():
        read_serial() 

        #publish encoder
        lwheel_tick_pub.publish(lwheel_tick_msg)
        lwheel_rate_pub.publish(lwheel_rate_msg)

        rwheel_tick_pub.publish(rwheel_tick_msg)
        rwheel_rate_pub.publish(rwheel_rate_msg) 

        #publish lidar
        #scan_pub.publish(scan_msg)

        #publish imu
        #imu_pub.publish(imu_msg)
        calculate_pose()
	mytuple = (str(int(left_vel)), ",", str(int(right_vel)), "/")
        #stm32_msg = str(left)+","+str(right)+"/"
        stm32_msg = "".join(mytuple)
        stm32_msg = stm32_msg.encode('utf-8')
        ser_front.write(stm32_msg)
        time.sleep(0.1)
    left_vel = 0
    right_vel = 0
    mytuple = (str(int(left_vel)), ",", str(int(right_vel)), "/")
    #stm32_msg = str(left)+","+str(right)+"/"
    stm32_msg = "".join(mytuple)
    stm32_msg = stm32_msg.encode('utf-8')
    ser_front.write(stm32_msg)
    ser_front.close()

