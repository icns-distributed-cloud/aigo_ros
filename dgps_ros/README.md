# dgps_ros

#### Introduction
ROS Package for differential GPS, with no external dependency.  
It receive RTCM data from the Ntrip(Networked Transport of RTCM via Internet Protocol) server,
and send to the gps module. NMEA data received from gps module is then converted into  navsatfix message.


#### Installation

1.  mkdir -p catkin/src
2.  cd catkin
3.  catkin_make
4.  cd src/
5.  wget https://gitee.com/rivercloud/dgps_ros/repository/archive/master.zip
6.  unzip master
7.  cd ..
8.  catkin_make

#### how to launch

1. roslaunch dgps_ros dgps.launch

### paramaters
~server  
address(domain name or ip address) for Ntrip(Networked Transport of RTCM via Internet Protocol) server

~port  
service port for the ntrip server

~username  
username for ntrip server

~password  
password for username in ntrip server

~serialPort  
serial port device file for the gps device


### published topics
  dgps(sensor_msgs/NavSatFix)  
differential GPS message



