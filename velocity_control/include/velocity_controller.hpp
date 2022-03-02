#ifndef _VELOCITY_CONTROLLER_HPP
#define _VELOCITY_CONTROLLER_HPP
#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
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



namespace velocityController
{
    class velocity_state_controller
    {
        public:
            bool goal;

            double measured_linear_velocity, measured_angular_velocity;
            void odom(const nav_msgs::Odometry::ConstPtr& msg);
            void getParameters();
            void compute(double refrence_linear_velocity, double refrence_angular_velocity);
            velocity_state_controller(ros::NodeHandle &nh);
            
            
            geometry_msgs::Twist speed;
            ros::Subscriber sub;
            ros::Publisher pub;
            parameters angular_velocity;
            error angular;
            parameters velocity;
            error velocity_error;
            
    };
}

#endif