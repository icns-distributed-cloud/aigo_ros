#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <thread>
#include <string>
#include "ntrip.h"


#define STATUS_NO_FIX -1
#define STATUS_FIX 0
#define STATUS_GBAS_FIX 2
#define SERVICE_GPS 1
#define COVARIANCE_TYPE_DIAGONAL_KNOWN  2

double NMEA2float(std::string s)
{
    double d = std::stod(s)/100.0;
    d = floor(d) + (d - floor(d))*10/6;
    return d;
}

boost::array<double,9> covariance={1,0,0,
                     0,1,0,
                     0,0,1 };

void fillSatMessage(sensor_msgs::NavSatFix& sat, Location& loc )
{
    sat.header.frame_id="gps";
    sat.header.stamp = ros::Time::now();

    if( loc.fix == "0" )
        sat.status.status = STATUS_NO_FIX;
    else if( loc.fix == "1" )
        sat.status.status = STATUS_FIX;
    else
        sat.status.status = STATUS_GBAS_FIX;
    sat.status.service = SERVICE_GPS;

    sat.latitude = NMEA2float(loc.lat);
    sat.longitude = NMEA2float(loc.lon);
    sat.altitude = std::stod(loc.alt);
   
    std::copy(covariance.begin(), covariance.end(),sat.position_covariance.begin());
    sat.position_covariance[0]  = std::stod(loc.hdop);
    sat.position_covariance[4]  = std::stod(loc.hdop);
    sat.position_covariance[8]  = 10;


    sat.position_covariance_type = COVARIANCE_TYPE_DIAGONAL_KNOWN;
    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "dgps_node");
    ros::NodeHandle nh("~");


    std::string serverName,userName,password,serverPort,serialPort;
    nh.param<std::string>("server",serverName, "www.gnssdata.or.kr");
    nh.param<std::string>("port",serverPort, "2101");
    nh.param<std::string>("username",userName, "user");
    nh.param<std::string>("password",password, "password");
    nh.param<std::string>("serialPort",serialPort, "");


    struct Args args;
    args.server = serverName.c_str();
    args.port = "2201";
    args.user = userName.c_str();
    args.password = password.c_str();
    args.nmea = "$GNGGA,034458.00,2810.79928,N,11256.54467,E,2,12,0.64,36.0,M,-12.7,M,1.0,0773*7D";
    args.data = "RTCM32_GGB";
    args.bitrate = 0;
    args.proxyhost = 0;
    args.proxyport = "2101";
    args.mode = NTRIP1;
    args.initudp = 0;
    args.udpport = 0;
    args.protocol = SPAPROTOCOL_NONE;
    args.parity = SPAPARITY_NONE;
    args.stopbits = SPASTOPBITS_1;
    args.databits = SPADATABITS_8;
    args.baud = SPABAUD_115200;
    if( serialPort.empty())
        args.serdevice = 0;//"/dev/ttyUSB0";  
    else
        args.serdevice = serialPort.c_str();
    args.serlogfile = 0;
    args.stop = false;

    
    ROS_INFO("Username= %s",args.user); 
    ROS_INFO("password= %s",args.password); 

    const std::string topic = "dgps";
    ros::Publisher pub = nh.advertise<sensor_msgs::NavSatFix>(topic,10);

    
    std::thread ntrip_thread(ntrip_client,&args);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        Location loc = getGNGGA();
        if( loc.lat.empty() )
        {
        }
        else
        {
           // cout<<s;
            sensor_msgs::NavSatFix sat;
            fillSatMessage(sat,loc);
            pub.publish(sat);
            ROS_INFO("Talker_____:GPS:x = %s",loc.nmea.c_str()); 
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    args.stop = true;
    ROS_INFO("Waiting to Quit");
    ntrip_thread.join();
    
}

