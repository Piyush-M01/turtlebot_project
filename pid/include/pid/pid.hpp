#ifndef _PID_HPP_
#define _PID_HPP_
#include <iostream>
#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <math.h>
#include<vector>
#include<bits/stdc++.h>

typedef struct PARAMETERS
{
    double kp;
    double kd;
    double ki;
    double velocity_min;
    double velocity_max;
    double acceleration_max;

    double previous_state;
}parameters;

typedef struct ERROR
{
    double previous_error;
    double integral_error;
}error;



namespace PID
{
    class pid
    {
        public:
            bool goal;
            double x,y;
            void callbacks(const nav_msgs::Odometry::ConstPtr& msg);
            void getParameters();
            void compute(double theta_error,double distance_error, double linear_velocity, double angular_velocity);
            pid(ros::NodeHandle &nh);
            
            
            geometry_msgs::Twist speed;
            ros::Subscriber sub;
            ros::Publisher pub;
            parameters angle;
            error angle_err;
            parameters distance;
            error dist_err;
            
    };
}


#endif