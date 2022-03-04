#ifndef _VELOCITY_CONTROLLER_HPP
#define _VELOCITY_CONTROLLER_HPP

#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <bits/stdc++.h>
#include "realtime_tools/realtime_publisher.h"
#include <dynamic_reconfigure/server.h>
#include <velocity_control/dynamicConfig.h>

typedef struct PARAMETERS
{
    double kp;
    double kd;
    double ki;

    double velocity_max;
    double acceleration_max;

    double previous_state;

    double diff;
    double diff_filtered;

    double integral_max;
}parameters;

typedef struct ERROR
{
    double previous_error;
    double integral_error;
    double velocity_error;

}error;

namespace velocityController
{
    class velocity_state_controller
    {
        public:

            void odom(const nav_msgs::Odometry::ConstPtr& msg);
            void goal(const geometry_msgs::Twist::ConstPtr& msg);
            void getParameters();
            void compute(double refrence_linear_velocity, double refrence_angular_velocity);
            velocity_state_controller(const ros::NodeHandlePtr &nh);
            void controlLoop(const ros::TimerEvent& event);

        private:

            double linear_ref, angular_ref;
            double measured_linear_velocity, measured_angular_velocity;
            geometry_msgs::Twist speed;

            ros::Subscriber sub;
            ros::Subscriber goalSub;
            // ros::Publisher pub; //convert to realtime publisher

            realtime_tools::RealtimePublisherSharedPtr<geometry_msgs::Twist> pub;

            ros::Timer control_timer;
            double control_rate;

            parameters angular_velocity;
            error angular;

            parameters linear_velocity;
            error linear;


    };
}

#endif