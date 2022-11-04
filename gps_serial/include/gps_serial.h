#ifndef GPS_SERIAL_H
#define GPS_SERIAL_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <nmea_msgs/Gpgga.h>

class gps_serial
{

public:
    gps_serial(std::string name, int baudrate, int timeout)
        : port_(name), baudrate_(baudrate), timeout_(timeout), gps_start_("$GPGGA"), gps_end_("\r\n"), success_(false)
    {
    }
    ~gps_serial() {}

    int Init()
    {
        try
        {
            //设置串口属性，并打开串口
            ser_.setPort(port_);
            ser_.setParity(serial::parity_none);
            ser_.setBaudrate(baudrate_);
            ser_.setStopbits(serial::stopbits_one);
            ser_.setBytesize(serial::eightbits);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser_.setTimeout(to);
            ser_.open();
        }
        catch (serial::IOException &e)
        {
            ROS_INFO(e.what());
            ROS_ERROR("Unable to open port ");
            return -1;
        }

        //检测串口是否已经打开，并给出提示信息
        if (ser_.isOpen())
        {
            ROS_INFO("Serial Port initialized");
        }
        else
        {
            return -1;
        }
    }

    void ReadData()
    {
        // bool Received = false;
        int start = -1, end = -1;
        std::string s;
        success_ = false;
        while (!success_)
        {
            ser_.waitReadable();
            if (ser_.available())
            {
                ser_.readline(s, 600, gps_end_);
                //printf("\n %s", s.c_str());
            }
            start = s.find(gps_start_);
            if (start != -1)
            {
                
                gps_data_ = s.substr(start);
                printf(" %s", gps_data_.c_str());
                success_ = true;
                if (GpsDataProcess(gps_data_))
                {
                    success_ = true;
                }
            }
        }
    }

    bool GpsDataProcess(std::string &gps_data)
    {
        //分割有效数据，存入vector中
        std::vector<std::string> data;
        int begin = 0, end;
        end = gps_data.find(",");
        while (end != std::string::npos)
        {
            data.push_back(gps_data.substr(begin, end - begin));
            begin = end + 1;
            end = gps_data.find(",", begin);
        }
        if (begin != gps_data.size())
        {
            data.push_back(gps_data.substr(begin));
        }
        printf("%d \n", data.size());
        if (data.size() >= 6 && (data[6] == "1" || data[6] == "2" || data[6] == "3" || data[6] == "4" || data[6] == "5" || data[6] == "6" || data[6] == "8" || data[6] == "9"))
        {
            // printf("%f \n", lat_);
            // printf("%f \n", lon_);
            if (!data[2].empty())
            {
                lat_ = std::atof(data[2].c_str()) / 100.0;
                
            }
            if (!data[4].empty())
            {
                lon_ = std::atof(data[4].c_str()) / 100.0;
                
                return true;
            }
            return false;
        }
        return false;
    }

    double Longitude() { return lon_; }
    double Latitude() { return lat_; }
    std::string GPS_data() {return gps_data_;}
    bool Success(){return success_;}
private:
    /* data */
    serial::Serial ser_;
    std::string port_;
    int baudrate_;
    int timeout_;
    std::string gps_data_;
    std::string gps_start_;
    std::string gps_end_;
    double lat_;
    double lon_;
    bool success_;
};

#endif // GPS_SERIAL_H