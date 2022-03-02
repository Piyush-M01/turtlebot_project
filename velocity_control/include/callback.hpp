#ifndef _CALLBACK_HPP
#define _CALLBACK_HPP
#include "velocity_controller.hpp"


void velocityController::velocity_state_controller::compute(double refrence_linear_velocity, double refrence_angular_velocity)
{
    double angular_velocity_error = refrence_angular_velocity - this->measured_angular_velocity;
    double linear_velocity_error = refrence_linear_velocity - this->measured_linear_velocity;
    double pub_angular_velocity, pub_linear_velocity;
    
    double angular_diff=angular_velocity_error-this->angular.previous_error;
    double linear_diff=linear_velocity_error-this->velocity_error.previous_error;

    double angular_diff_filter=pow((1+(1/angular_diff)),2)/((2+sqrt(2))+((2-sqrt(2))*pow((1/angular_diff),2)));
    double linear_diff_filter=pow((1+(1/linear_diff)),2)/((2+sqrt(2))+((2-sqrt(2))*pow((1/linear_diff),2)));

    //VSPID integral windup fix
    //a = tanh(time constant * b(t)-e) where e is a constant and a (- [0,1] and e (- (0,1)
    //b=abs(error)-e if abs(error)>=e and b=0 if abs(error)<e
    //a*derivative_error + (1-a)*integral_error + proportional_error
    //http://www.iust.ac.ir/files/ee/farrokhi_0a5f0/conference_1_50/c46.pdf

    // double e_angular=0.5;
    // double b_angular= (abs(angular_velocity_error)-e_angular)>=e_angular?(abs(angular_velocity_error)-e_angular):0;
    // double a_angular=tanh(0.05*b_angular-e_angular);

    // pub_angular_velocity=this->angular_velocity.kp*angular_velocity_error + (1-a_angular)*(this->angular_velocity.ki*this->angular.integral_error) + a_angular*(this->angular_velocity.kd*(angular_diff_filter));
    
    // double e_linear=0.5;
    // double b_linear= (abs(linear_velocity_error)-e_linear)>=e_linear?(abs(linear_velocity_error)-e_linear):0;
    // double a_linear=tanh(0.05*b_linear-e_linear);

    // pub_linear_velocity=this->velocity.kp*linear_velocity_error + (1-a_linear)*(this->velocity.ki*this->velocity_error.integral_error) + a_linear*(this->velocity.kd*(linear_diff_filter));
    
    ros::NodeHandle nh;
    nh.getParam("/velocity/kp_angle",this->angular_velocity.kp);
    nh.getParam("/velocity/ki_angle",this->angular_velocity.ki);
    nh.getParam("/velocity/kd_angle",this->angular_velocity.kd);

    nh.getParam("/velocity/kp_distance",this->velocity.kp);
    nh.getParam("/velocity/ki_distance",this->velocity.ki);
    nh.getParam("/velocity/kd_distance",this->velocity.kd);
    
    double integeral_max;
    integeral_max = (this->angular_velocity.velocity_max - this->angular_velocity.kp*angular_velocity_error - this->angular_velocity.kd*angular_diff)/this->angular_velocity.ki;

    if(this->angular.integral_error>integeral_max)
    {
        this->angular.integral_error = integeral_max;
    }
    integeral_max = (this->velocity.velocity_max - this->velocity.kp*linear_velocity_error - this->velocity.kd*linear_diff)/this->velocity.ki;

    if(this->velocity_error.integral_error>integeral_max)
    {
        this->velocity_error.integral_error = integeral_max;
    }
    pub_angular_velocity=this->angular_velocity.kp*angular_velocity_error + (this->angular_velocity.ki*this->angular.integral_error) + (this->angular_velocity.kd*(angular_diff_filter));
    pub_linear_velocity=this->velocity.kp*linear_velocity_error + (this->velocity.ki*this->velocity_error.integral_error) + (this->velocity.kd*(linear_diff_filter));

    this->angular.integral_error += angular_velocity_error;
    this->velocity_error.integral_error += linear_velocity_error;

    this->velocity_error.previous_error = linear_velocity_error;
    this->angular.previous_error = angular_velocity_error;

    this->speed.linear.x=pub_linear_velocity;
    this->speed.angular.z=pub_angular_velocity;

    this->pub.publish(this->speed);
  
}

void velocityController::velocity_state_controller::odom(const nav_msgs::Odometry::ConstPtr& msg)
{
    ros::NodeHandle nh;

    double linear_ref,angular_ref;
    
    this->measured_linear_velocity=msg->twist.twist.linear.x;
    this->measured_angular_velocity=msg->twist.twist.angular.z;

    nh.getParam("/velocity/linear_velocity",linear_ref);
    nh.getParam("/velocity/angular_velocity",angular_ref);
    
    compute(linear_ref,angular_ref);

}
#endif