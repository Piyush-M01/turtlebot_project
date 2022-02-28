#ifndef _CALLBACK_HPP
#define _CALLBACK_HPP
#include "velocity_controller.hpp"


void velocityController::velocity_state_controller::compute(double refrence_linear_velocity, double refrence_angular_velocity)
{
    double angular_velocity_error = refrence_angular_velocity - this->measured_angular_velocity;
    double linear_velocity_error = refrence_linear_velocity - this->measured_linear_velocity;
    double pub_angular_velocity, pub_linear_velocity;
    
    pub_angular_velocity=this->angular_velocity.kp*angular_velocity_error + this->angular_velocity.ki*this->angular_velocity.integral_error + this->angular_velocity.kd*(angular_velocity_error-this.angular.previous_error);
   
    pub_linear_velocity=this->linear_velocity.kp*linear_velocity_error + this->linear_velocity.ki*this->linear_velocity.integral_error + this->linear_velocity.kd*(linear_velocity_error-this->velocity_error.previous_error);

    

    this->angular.integral_error += angular_velocity_error;
    this->linear_velocity.integral_error += linear_velocity_error;
    
    this->velocity_error.previous_error = linear_velocity_error;
    this->angular.previous_error = angular_velocity_error;
  
}

void velocityController::velocity_state_controller::odom(const nav_msgs::Odometry::ConstPtr& msg)
{
    ros::NodeHandle nh;
    
    double linear_ref,angular_ref;
    
    this->measured_linear_velocity=msg->twist.twist.linear.x;
    this->measured_angular_velocity=msg->twist.twist.angular.z;

    nh.getParam("/velocity/linear_velocity",linear_ref);
    ng.getParam("/velocity/angular_velocity",angular_ref);
    
    compute(linear_ref,angular_ref);

}
#endif