#ifndef GPS_SERIAL_H
#define GPS_SERIAL_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

class gps_serial
{

public:
    gps_serial(std::string name, int baudrate)
    {
        port_ = name;
        baudrate_ = baudrate;
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


    void ReadData(std::string& s)
    {
        ser_.waitReadable();
        if(ser_.available())
        {
            s = ser_.read(ser_.available());
        }

    }

private:
    /* data */
    serial::Serial ser_;
    std::string port_;
    int baudrate_;
};

#endif // GPS_SERIAL_H