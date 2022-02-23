#include<iostream>
#include "../include/pid.hpp"
#include "../include/callbacks.hpp"

using namespace std;

Pidcontrol::Pidcontrol(ros::NodeHandle &nh)
{
    this->pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    this->sub = nh.subscribe<nav_msgs::Odometry>("/odom", 10, &Pidcontrol::OdomCB, this);
    this->prev_error=0;
    this->prev_theta_err=0;
    this->integral_dist=0;
    this->integral_angle=0;
}