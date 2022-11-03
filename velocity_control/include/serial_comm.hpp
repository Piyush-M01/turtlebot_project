#pragma once

#include <stdio.h>
#include <string.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

class Serial
{
    public:
        
        Serial();
        void callback(const geometry_msgs::Twist::ConstPtr& msg);
        int serial_port;
    private:
        ros::Subscriber sub;
        ros::NodeHandle n;  

};
