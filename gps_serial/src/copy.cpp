#include "gps_serial.h"
#include <nmea_msgs/Gpgga.h>
#include <signal.h>

serial::Serial ser;

void mySigintHandle(int sig)
{
    ros::shutdown();
}
void RecePro(std::string s, double &lat, double &lon, double &high,std::vector<std::string>& v)
{
    //分割有效数据，存入vector中
    //std::vector<std::string> v;
    std::string::size_type pos1, pos2;
    pos2 = s.find(",");
    pos1 = 0;
    while (std::string::npos != pos2)
    {
        v.push_back(s.substr(pos1, pos2 - pos1));
        pos1 = pos2 + 1;
        pos2 = s.find(",", pos1);
    }
    if (pos1 != s.length())
        v.push_back(s.substr(pos1));
    //解析出经纬度
    //字段6：GPS状态，0=未定位，1=非差分定位，2=差分定位，3=无效PPS，6=正在估算
    if (v.max_size() >= 6 && (v[6] == "1" || v[6] == "2" || v[6] == "3" || v[6] == "4" || v[6] == "5" || v[6] == "6" || v[6] == "8" || v[6] == "9"))
    {
        //纬度
        if (v[2] != "")
            lat = std::atof(v[2].c_str()) / 100;
        int ilat = (int)floor(lat) % 100;
        lat = ilat + (lat - ilat) * 100 / 60;
        //经度
        if (v[4] != "")
            lon = std::atof(v[4].c_str()) / 100;
        int ilon = (int)floor(lon) % 1000;
        lon = ilon + (lon - ilon) * 100 / 60;
        //海拔高度
        if (v[9] != "")
            high = std::atof(v[9].c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;
    std::string name = "/dev/ttyUSB0";
    int baudrate = 115200;
    int timeout = 1000;
    gps_serial gps(name, baudrate, timeout);
    gps.Init();
    ros::Rate loop_rate(10);
    std::string gstart = "$GPGGA";
    std::string g_end = "\r\n";
    // std::string receiver_data;
    // ros::init(argc, argv, "serial_node");
    // //声明节点句柄
    // ros::NodeHandle nh;
    // //注册Publisher到话题GPS
    ros::Publisher GPS_pub = nh.advertise<nmea_msgs::Gpgga>("GPS", 1000);
    signal(SIGINT, mySigintHandle);
    try
    {
        //串口设置
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(5000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open Serial Port !");
        return -1;
    }
    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(50);
    nmea_msgs::Gpgga gps_data;
    std::string receiver_data;
    while (ros::ok())
    {

        if (ser.available())
        {
            receiver_data += ser.read(ser.available());
            int i = 0, start = -1, end = -1;
            while (i < receiver_data.size())
            {
                start = receiver_data.find(gstart);
                if (start == -1)
                {
                    if (receiver_data.size() > 5)
                    {
                        receiver_data = receiver_data.substr(receiver_data.size() - 6);
                        break;
                    }
                }
                else
                {
                    end = receiver_data.find(g_end);
                    if (end == -1)
                    {
                        // if(end != 0)
                        receiver_data = receiver_data.substr(start);
                        break;
                    }
                    else
                    {
                        i = end;
                        double sec = ros::Time::now().toSec();
                        double lat, lon, high;
                        std::vector<std::string> data;
                        RecePro(receiver_data.substr(start, end + 2 - start), lat, lon, high, data);
                        // std::cout << "lat: " << lat << std::endl;
                        // std::cout << "lon: " << lon << std::endl;
                        // std::cout << "high: " << high << std::endl;
                        // std::cout<<"size: "<< data.size()<<std::endl;
                        gps_data.lat = lat;
                        gps_data.lon = lon;
                        gps_data.lat_dir = data[3];
                        gps_data.lon_dir = data[5];
                        gps_data.alt = high;
                        gps_data.altitude_units = data[11];
                        //gps_data.utc_seconds = data[1];
                        //gps_data.
                        gps_data.header.stamp = ros::Time::now();
                        gps_data.header.frame_id = "map";
                        //gps_data.
                        GPS_pub.publish(gps_data);
                        receiver_data = receiver_data.substr(end+2);
                        // if ( i+8 < receiver_data.length())
                        //     receiver_data = receiver_data.substr(end+2);
                        // else
                        // {   receiver_data = receiver_data.substr(end+2);
                        //     break;
                        // }
                    }
                }
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}