#include<iostream>
#include "../include/pid.hpp"
using namespace std;

Pidcontrol::Pidcontrol(ros::NodeHandle &nh)
{
    this->pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    this->sub = nh.subscribe<nav_msgs::Odometry>("/odom", 100, &Pidcontrol::OdomCB, this);
    this->prev_error=0;
    this->prev_theta_err=0;
    //this->prev_time=0;
    this->integral_dist=0;
    this->integral_angle=0;
}
void Pidcontrol::OdomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
    ros::NodeHandle nh;
    //this->kp=0.4;
    //nh.getParam("/my_robot/p_gain",this->kp);
    //nh.getParam("/my_robot/i_gain",this->ki);
    nh.getParam("/my_robot/d_gain",this->kd);

    double roll,pitch,yaw;
    geometry_msgs::Twist speed;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation,quat);
    tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);

    double distance_err= sqrt(pow(this->x-msg->pose.pose.position.x,2)+pow(this->y-msg->pose.pose.position.y,2));
    this->theta=atan2(this->y-msg->pose.pose.position.y,this->x-msg->pose.pose.position.x);

    //fixing distance error first

    this->integral_dist=this->integral_dist+distance_err;

    double p_dist=kp*distance_err;
    double d_dist=kd*(distance_err-this->prev_error);
    double i_dist=ki*this->integral_dist;

    double theta_err=this->theta-yaw;
    printf("theta_err: %f \n",theta_err);

    //fixing angular error
    this->integral_angle=this->integral_angle+theta_err;
    double p_angle=kp*theta_err;
    double d_angle=kd*((theta_err-this->prev_theta_err));
    double i_angle=ki*this->integral_angle;

    printf("distance error %f \n",distance_err);

    if(abs(theta_err)>0.1)
        {
            speed.angular.z=0.08;
        }
        //speed.angular.z=(p_angle+i_angle+d_angle);
    else
    {
        speed.angular.z = 0;
        this->theta=0;
        if(distance_err>0.1)
        { 
           if(p_dist+i_dist+d_dist-this->prev_vel<acc_max)
            {
                speed.angular.y=0;
                if(p_dist+ i_dist+ d_dist<vel_max && p_dist+i_dist+d_dist>vel_min)
                    {
                        speed.linear.x=p_dist+i_dist+d_dist;
                        cout<<"between"<<speed.linear.x<<"\n";
                    }
                else if(p_dist+ i_dist+ d_dist>vel_max)
                {
                    speed.linear.x = vel_max;
                    cout<<"greater"<<speed.linear.x<<"\n";
                }
                else
                {
                    speed.linear.x  =  vel_min;
                    cout<<"smaller"<<speed.linear.x<<"\n";
                }
            }
            else
            {
                speed.angular.y=0;
                speed.linear.x=acc_max;
                cout<<"greater than amax"<<speed.linear.x<<"\n";
            }
        }
        else
        {
            speed.linear.x=0;
        }
    }   
    this->prev_error=distance_err;
    //this->prev_time=time;
    this->prev_theta_err=theta_err;
    this->pub.publish(speed); 
}

