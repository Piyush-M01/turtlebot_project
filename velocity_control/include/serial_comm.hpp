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
        
        Serial(); //constructor
        void callback(const geometry_msgs::Twist::ConstPtr& msg); //velocity topic callback
        int serial_port;    //serial port
        struct termios tty; // Create new termios struct, we call it 'tty' for convention
    private:
        ros::Subscriber sub; //ros subscriber
        ros::NodeHandle n;  //ros node handler
        std::string x,z;  //string for linear and angular velocity

};
