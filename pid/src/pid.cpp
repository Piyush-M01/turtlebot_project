#include "../include/pid/pid.hpp"
#include "../include/pid/callback.hpp"

PID::pid::pid(ros::NodeHandle &nh)
{
    //to intialize the parameters
    this->angle.kp = 0.01;
    this->angle.kd = 0.005;
    this->angle.ki = 0.0002;

    this->distance.kp = 0.01;
    this->distance.kd = 0.005;
    this->distance.ki = 0.0002;


    this->angle.previous_state = 0.0;
    this->distance.previous_state=0.0;

    this->dist_err.previous_error = 0.0;
    this->dist_err.integral_error = 0.0;

    this->angle_err.previous_error = 0.0;
    this->angle_err.integral_error = 0.0;


    this->pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    this->sub = nh.subscribe<nav_msgs::Odometry>("/odom", 100, &PID::pid::callbacks, this);
}