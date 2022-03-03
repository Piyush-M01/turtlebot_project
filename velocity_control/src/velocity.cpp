#include "../include/velocity_controller.hpp"
#include "../include/callback.hpp"

using namespace velocityController;

velocity_state_controller::velocity_state_controller(const ros::NodeHandlePtr &_nh)
{
    angular_velocity.previous_state=0;
    angular.previous_error=0;
    angular.integral_error=0;

    linear_velocity.previous_state=0;
    linear.previous_error=0;
    linear.integral_error=0;

    // pub = _nh->advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    pub = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::Twist>>(*_nh.get(),"/cmd_vel", 10);
    sub = _nh->subscribe<nav_msgs::Odometry>("/odom", 100, &velocity_state_controller::odom,this);
    goalSub = _nh->subscribe<geometry_msgs::Twist>("/cmd_vel_filtered", 100, &velocity_state_controller::goal, this);

    _nh->getParam("/velocity/control_rate", control_rate);

    control_timer = _nh->createTimer(ros::Duration(control_rate), &velocity_state_controller::controlLoop, this);

    _nh->getParam("/velocity/linear/kp",linear_velocity.kp);
    _nh->getParam("/velocity/linear/ki",linear_velocity.ki);
    _nh->getParam("/velocity/linear/kd",linear_velocity.kd);
    _nh->getParam("/velocity/linear/max",linear_velocity.velocity_max);
    
    _nh->getParam("/velocity/angular/kp",angular_velocity.kp);
    _nh->getParam("/velocity/angular/ki",angular_velocity.ki);
    _nh->getParam("/velocity/angular/kd",angular_velocity.kd);
    _nh->getParam("/velocity/angular/max",angular_velocity.velocity_max);
}