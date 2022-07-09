#ifndef _CALLBACKS_HPP_
#define _CALLBACKS_HPP_

#include "pid.hpp"
#define angle_wrap M_PI

void PID::pid::getParameters()
{
    ros::NodeHandle nh;
    nh.getParam("parameters/kp_angle", this->angle.kp);
    nh.getParam("parameters/ki_angle", this->angle.ki);
    nh.getParam("parameters/kd_angle", this->angle.kd);

    nh.getParam("parameters/kp_distance",this->distance.kp);
    nh.getParam("parameters/ki_distance",this->distance.ki);
    nh.getParam("parameters/kd_distance",this->distance.kd);

    nh.getParam("parameters/kp_angle", this->angle.kp);
    nh.getParam("parameters/ki_angle", this->angle.ki);
    nh.getParam("parameters/kd_angle",this->angle.kd);
    nh.getParam("parameters/angle_velocity_max",this->angle.velocity_max);
    nh.getParam("parameters/angle_acceleration_max",this->angle.acceleration_max);

    nh.getParam("parameters/kp_distance",this->distance.kp);
    nh.getParam("parameters/ki_distance", this->distance.ki);
    nh.getParam("parameters/kd_distance", this->distance.kd);
    nh.getParam("parameters/distance_velocity_min",this->distance.velocity_max);
    nh.getParam("parameters/distance_velocity_max",this->distance.velocity_min);
    nh.getParam("parameters/distance_acceleration_max",this->distance.acceleration_max);


    nh.getParam("setpoint_x",this->x);
    nh.getParam("setpoint_y",this->y);

}

void PID::pid::compute(double theta_error,double distance_error, double linear_velocity, double angular_velocity)
{
    if(abs(theta_error) > 0.1)
    {
        if(angular_velocity > this->angle.velocity_max)
            this->speed.angular.z = this->angle.velocity_max;
        else
            this->speed.angular.z = angular_velocity;   
    }
    else
    {
        std::cout<<"here\n";
        this->speed.angular.z = 0;
        if(distance_error>0.1)
        {
            this->speed.linear.x = linear_velocity > this->distance.velocity_max ? this->distance.velocity_max : linear_velocity;
        }
        else
        {
            this->dist_err.integral_error = 0;
            this->dist_err.previous_error = 0;
            this->speed.linear.x = 0;
        }
        this->angle_err.integral_error = 0;
        this->angle_err.previous_error = 0;
    }
    
    std::cout<<"distance_error: "<<distance_error<<std::endl;
    if(distance_error<0.1 && theta_error<0.1)
    {
        this->speed.linear.x=0;
        this->speed.angular.z=0;
        this->pub.publish(this->speed);
        ros::shutdown();
    }
    this->pub.publish(this->speed);
}

void PID::pid::callbacks(const nav_msgs::Odometry::ConstPtr& msg)
{
    getParameters();

    double roll;
    double pitch;
    double yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation,quat);
    tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);

    double distance_error;
    double theta;
    double theta_error;

    distance_error = sqrt(pow(msg->pose.pose.position.x - this->x,2) + pow(msg->pose.pose.position.y - this->y,2));
    theta =atan2(this->y-msg->pose.pose.position.y,this->x-msg->pose.pose.position.x);
    theta_error = theta - yaw;

    this->dist_err.integral_error = this->dist_err.integral_error + distance_error;
    this->angle_err.integral_error = this->angle_err.integral_error + theta_error;

    double linear_velocity;
    double angular_velocity;


    linear_velocity = this->distance.kp * distance_error + this->distance.ki * this->dist_err.integral_error + this->distance.kd * (distance_error - this->dist_err.previous_error);
    angular_velocity = this->angle.kp * theta_error + this->angle.ki * this->angle_err.integral_error + this->angle.kd * (theta_error - this->angle_err.previous_error);

    this->dist_err.previous_error = distance_error;
    this->angle_err.previous_error = theta_error;


    std::cout<<"theta: "<<theta<<std::endl;
    std::cout<<"theta_error: "<<theta_error<<std::endl;
    std::cout<<"yaw: "<<yaw<<std::endl;
    
    compute(theta_error,distance_error,linear_velocity,angular_velocity);

}
#endif
