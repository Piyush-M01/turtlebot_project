#pragma once

#include "velocity_controller.hpp"

using namespace velocityController;

void velocity_state_controller::compute(double refrence_linear_velocity, double refrence_angular_velocity)
{

    angular.velocity_error = refrence_angular_velocity - measured_angular_velocity;
    linear.velocity_error = refrence_linear_velocity - measured_linear_velocity;
        
    angular_velocity.diff = angular.velocity_error - angular.previous_error;
    linear_velocity.diff = linear.velocity_error - linear.previous_error;

    angular_velocity.diff_filtered=pow((1+(1/angular_velocity.diff)),2)/((2+sqrt(2))+((2-sqrt(2))*pow((1/angular_velocity.diff),2)));
    linear_velocity.diff_filtered=pow((1+(1/linear_velocity.diff)),2)/((2+sqrt(2))+((2-sqrt(2))*pow((1/linear_velocity.diff),2)));


    angular_velocity.integral_max = (angular_velocity.velocity_max - angular_velocity.kp*angular.velocity_error - angular_velocity.kd*angular_velocity.diff_filtered)/angular_velocity.ki;

    
    if(angular.integral_error>angular_velocity.integral_max)
    {
        angular.integral_error = angular_velocity.integral_max;
    }

    linear_velocity.integral_max = (linear_velocity.velocity_max - linear_velocity.kp*linear.velocity_error - linear_velocity.kd*linear_velocity.diff_filtered)/linear_velocity.ki;

    if(linear.integral_error>linear_velocity.integral_max)
    {
        linear.integral_error = linear_velocity.integral_max;
    }

    
    speed.angular.z=angular_velocity.kp*angular.velocity_error + (angular_velocity.ki*angular.integral_error) + (angular_velocity.kd*(angular_velocity.diff_filtered));
    speed.linear.x=linear_velocity.kp*linear.velocity_error + (linear_velocity.ki*linear.integral_error) + (linear_velocity.kd*(linear_velocity.diff_filtered));


    angular.integral_error += angular.velocity_error;
    linear.integral_error += linear.velocity_error;

    linear.previous_error = linear.velocity_error;
    angular.previous_error = angular.velocity_error;

    pub.publish(speed);
  
}

void velocity_state_controller::odom(const nav_msgs::Odometry::ConstPtr& msg)
{    
    this->measured_linear_velocity=msg->twist.twist.linear.x;
    this->measured_angular_velocity=msg->twist.twist.angular.z;

    compute(linear_ref,angular_ref);
}

void velocity_state_controller::goal(const geometry_msgs::Twist::ConstPtr& msg)
{
    this->linear_ref = msg->linear.x;
    this->angular_ref = msg->angular.z;
}