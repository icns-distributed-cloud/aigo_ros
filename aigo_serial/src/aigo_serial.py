#!/usr/bin/env python

import serial, time, rospy, re
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import LaserScan, Imu
ser_front = serial.Serial( \
    port='/dev/ttyAMA2', \
    baudrate=115200,
)


def left_callback(msg):
    lwheel_desired_rate = msg.data

def right_callback(msg):
    rwheel_desired_rate = msg.data

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
    '''
    elif serial_data[0:1] == "i":
        serial_data = serial_data[1:]
        #read_imu(serial_data)
    '''

    '''
    elif serial_data[0:1] == "l":
        serial_data = serial_data[1:]
        read_lidar(serial_data)
    '''
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
'''
def read_lidar(serial_data):
    scan_data = []
    #string -> integer
    scan_data = serial_data.split(',')
    for i, v in enumerate(scan_data):
        if is_int(v) == True:
            v = int(v)
            scan_data.append(v)
    time = rospy.Time.now
    scan_msg.header.stamp = time
    scan_msg.header.frame_id = "laser_frame"
    scan_msg.angle_min = -180 * 0.017 # degree to radian
    scan_msg.angle_max = 180 * 0.017 # degree to radian
    scan_msg.angle_increment = 1 * 0.017 # degree to radian
    scan_msg.scan_time = 0.1 # MOTOR_CTR 5V --> works at 10Hz
    scan_msg.time_increment = 0.1 / 360 
    scan_msg.range_min = 0
    scan_msg.range_max = 12
    scan_msg.ranges.resize(361)  # 0 ~ 360 degree !!
    for i in range(0, 359):
        scan_msg.ranges[i] = scan_data[i]

def read_imu(serial_data):
    imu_data = []
    imu_data = serial_data.split(',')
    for i, v in enumerate(imu_data):
        if is_int(v) == True:
            v = int(v)
'''    


# ticks : cumulative encoder ticks, rate : encoder ticks per second
if __name__ == '__main__':
    rospy.init_node('aigo_serial') # initialize node
    lwheel_desired_rate = 0
    rwheel_desired_rate = 0

    tick_data = [0, 0]
    
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
    stm32_msg = str(lwheel_desired_rate)+','+str(rwheel_desired_rate)
    
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
        ser_front.write(stm32_msg)
        time.sleep(0.1)
    
    ser_front.close()
