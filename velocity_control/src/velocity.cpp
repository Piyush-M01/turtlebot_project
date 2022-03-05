#include "../include/velocity_controller.hpp"
#include "../include/callback.hpp"
#include <dynamic_reconfigure/server.h>
#include <velocity_control/dynamicConfig.h>

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

    _nh->getParam("/velocity_controller/kp_linear",linear_velocity.kp);
    _nh->getParam("/velocity_controller/ki_linear",linear_velocity.ki);
    _nh->getParam("/velocity_controller/kd_linear",linear_velocity.kd);
    _nh->getParam("/velocity/linear/max",linear_velocity.velocity_max);
    
    _nh->getParam("/velocity_controller/kp_angular",angular_velocity.kp);
    _nh->getParam("/velocity_controller/ki_angular",angular_velocity.ki);
    _nh->getParam("/velocity_controller/kd_angular",angular_velocity.kd);
    _nh->getParam("/velocity/angular/max",angular_velocity.velocity_max);
}


void callback(velocity_control::dynamicConfig &config, uint32_t level) {
//   ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
//             config.int_param, config.double_param, 
//             config.str_param.c_str(), 
//             config.bool_param?"True":"False", 
//             config.size);
}