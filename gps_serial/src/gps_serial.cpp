#include "gps_serial.h"
#include <nmea_msgs/Gpgga.h>
#include <std_msgs/String.h>
#include <signal.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;
    std::string name = "/dev/ttyUSB1";
    int baudrate = 115200;
    int timeout = 100;
    gps_serial gps(name, baudrate, timeout);
    std_msgs::String str_msg;
    if (gps.Init() != -1)
    {
        ros::Rate loop_rate(10);
        ros::Publisher GPS_pub = nh.advertise<nmea_msgs::Gpgga>("GPS", 1000);
        ros::Publisher gps_string_pub = nh.advertise<std_msgs::String>("GPS_string", 1000);
        //ros::Rate loop_rate(50);
        nmea_msgs::Gpgga gps_data;
        std::string receiver_data;
        while (ros::ok())
        {
            gps.ReadData();
            if(gps.Success())
            {
                str_msg.data = gps.GPS_data();
                gps_string_pub.publish(str_msg);
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    return 0;
}
