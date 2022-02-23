#ifndef _CALLBACKS_HPP_
#define _CALLBACKS_HPP_
#include "pid.hpp"

double Pidcontrol::angle_PID(double theta_err)
{
    this->integral_angle=this->integral_angle+theta_err;
    double p_angle=kp_angle*theta_err;
    double d_angle=kd_angle*(abs(theta_err-this->prev_theta_err));
    double i_angle=ki_angle*this->integral_angle;
    double gain=p_angle+i_angle+d_angle;
    return gain;
}

double Pidcontrol::distance_PID(double distance_err)
{
    this->integral_dist=this->integral_dist+distance_err;
    double p_dist=kp*distance_err;
    double d_dist=kd*(distance_err-this->prev_error);
    double i_dist=ki*this->integral_dist;
    double gain=p_dist+i_dist+d_dist;
    return gain;
}

void Pidcontrol::fix_angle(double gain)
{
    if (abs(gain)>0.08)
    this->speed.angular.z=0.08;
    else
    this->speed.angular.z=gain;
}

void Pidcontrol::fix_distance(double pid_dist)
{
    if(pid_dist-this->prev_vel<this->acc_max)
    {
        if(pid_dist<this->vel_max && pid_dist>this->vel_min)
        {
            this->speed.linear.x=pid_dist;
        }
        else if(pid_dist>this->vel_max)
        {
            this->speed.linear.x = this->vel_max;
        }
        else
        {
            this->speed.linear.x  = this->vel_min;
        }
    }
    else
    {
        this->speed.linear.x=this->acc_max;
    }
}

void Pidcontrol::OdomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
    ros::NodeHandle nh;
    //this->kp=0.4;
    //nh.getParam("/my_robot/p_gain",this->kp);
    //nh.getParam("/my_robot/i_gain",this->ki);
    //nh.getParam("/my_robot/d_gain",this->kd);

    double roll,pitch,yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation,quat);
    tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);

    double distance_err= sqrt(pow(this->x-msg->pose.pose.position.x,2)+pow(this->y-msg->pose.pose.position.y,2));
    this->theta=atan2(this->y-msg->pose.pose.position.y,this->x-msg->pose.pose.position.x);

    double theta_err=this->theta-yaw;
    printf("theta_err: %f \n",theta_err);

    printf("distance error %f \n",distance_err);

    if(abs(theta_err)>0.05 && !is_fixed)
    {
        fix_angle(angle_PID(theta_err));
    }
    else
    {
        if(abs(theta_err)>0.06)
        is_fixed=false;
        else
        is_fixed=true;
        this->speed.angular.z=0;

        if(distance_err>0.1)
        { 
           double pid_dist=distance_PID(distance_err);
           fix_distance(pid_dist);
        }
        else
        {
            speed.linear.x=0;
        }
    }

    if(distance_err<0.1 && abs(theta_err)<0.05)
    {
        speed.linear.x=0.0;
        speed.angular.z=0.0;
        this->pub.publish(speed);
        ros::shutdown();
    }

    this->pub.publish(speed);  
    this->prev_error=distance_err;
    this->prev_theta_err=theta_err;
}

#endif