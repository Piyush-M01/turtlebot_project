#pragma once

#include "velocity_controller.hpp"

using namespace velocityController;

void velocity_state_controller::compute(double refrence_linear_velocity, double refrence_angular_velocity)
{

    // nh->getParam("/velocity_controller/kp_linear",linear_velocity.kp);
    // nh->getParam("/velocity_controller/ki_linear",linear_velocity.ki);
    // nh->getParam("/velocity_controller/kd_linear",linear_velocity.kd);

    // nh->getParam("/velocity_controller/kp_angular",angular_velocity.kp);
    // nh->getParam("/velocity_controller/ki_angular",angular_velocity.ki);
    // nh->getParam("/velocity_controller/kd_angular",angular_velocity.kd);

    angular.velocity_error = refrence_angular_velocity - measured_angular_velocity;
    linear.velocity_error = refrence_linear_velocity - measured_linear_velocity;
        
    angular_velocity.diff = angular.velocity_error - angular.previous_error;
    linear_velocity.diff = linear.velocity_error - linear.previous_error;

    // if(angular_velocity.diff!=0)
    // {   
    //     angular_velocity.diff_filtered=pow((1+(1/angular_velocity.diff)),2)/((2+sqrt(2))+((2-sqrt(2))*pow((1/angular_velocity.diff),2)));
    // }
    // if(linear_velocity.diff!=0)
    //     linear_velocity.diff_filtered=pow((1+(1/linear_velocity.diff)),2)/((2+sqrt(2))+((2-sqrt(2))*pow((1/linear_velocity.diff),2)));

    // std::cout<<"idk"<<linear_velocity.kp<<"\n";
    angular.integral_error += angular.velocity_error;
    angular_velocity.integral_max =angular_velocity.integral_max+(angular_velocity.velocity_max - angular_velocity.kp*angular.velocity_error - angular_velocity.kd*angular_velocity.diff_filtered)/angular_velocity.ki;

    
    linear.integral_error += linear.velocity_error;

    // if(angular.integral_error>angular_velocity.integral_max)
    // {
    //     angular.integral_error = angular_velocity.integral_max;
    // }

    linear_velocity.integral_max = linear_velocity.integral_max+(linear_velocity.velocity_max - linear_velocity.kp*linear.velocity_error - linear_velocity.kd*linear_velocity.diff_filtered)/linear_velocity.ki;
    std::cout<<"max_integral: "<<linear_velocity.integral_max<<"\n";


    // if(linear.integral_error>linear_velocity.integral_max)
    // {
    //     linear.integral_error = linear_velocity.integral_max;
    // }
    
    //speed.angular.z=angular_velocity.kp*angular.velocity_error + (angular_velocity.ki*angular.integral_error) + (angular_velocity.kd*(angular_velocity.diff_filtered));
    //speed.linear.x=linear_velocity.kp*linear.velocity_error + (linear_velocity.ki*linear.integral_error) + (linear_velocity.kd*(linear_velocity.diff_filtered));

    if(pub->trylock())
    {
        pub->msg_.angular.z=(measured_angular_velocity+(angular_velocity.kp*angular.velocity_error + (angular_velocity.ki*angular.integral_error) + (angular_velocity.kd*(angular_velocity.diff_filtered))));
        pub->msg_.linear.x=measured_linear_velocity+(linear_velocity.kp*linear.velocity_error + (linear_velocity.ki*linear.integral_error) + (linear_velocity.kd*(linear_velocity.diff_filtered)));
        pub->unlockAndPublish();
    }


    linear.previous_error = linear.velocity_error;
    angular.previous_error = angular.velocity_error;


}

void velocity_state_controller::odom(const nav_msgs::Odometry::ConstPtr& msg)
{    
    this->measured_linear_velocity=msg->twist.twist.linear.x;
    this->measured_angular_velocity=msg->twist.twist.angular.z;

    //dynamic reconfigure for kp, ki, kd

}

void velocity_state_controller::goal(const geometry_msgs::Twist::ConstPtr& msg)
{
    this->linear_ref = msg->linear.x;
    this->angular_ref = msg->angular.z;
}

void velocity_state_controller::controlLoop(const ros::TimerEvent& event)
{
    compute(this->linear_ref, this->angular_ref);
}
