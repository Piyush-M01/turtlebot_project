#include "../include/velocity_controller.hpp"
#include "../include/callback.hpp"

velocityController::velocity_state_controller::velocity_state_controller(ros::NodeHandle &nh)
{
    this->angular_velocity.previous_state=0;
    this->angular.previous_error=0;
    this->angular.integral_error=0;

    this->velocity.previous_state=0;
    this->velocity_error.previous_error=0;
    this->velocity_error.integral_error=0;

    this->pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    this->sub = nh.subscribe<nav_msgs::Odometry>("/odom",100, &velocityController::velocity_state_controller::odom, this);
}