#ifndef _CALLBACKS_HPP_
#define _CALLBACKS_HPP_

#include "pid.hpp"
#define angle_wrap M_PI
void PID::pid::callbacks(const nav_msgs::Odometry::ConstPtr& msg)
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

    std::cout << "x: " << this->x << std::endl;
    std::cout << "y: " << this->y << std::endl;

    double roll;
    double pitch;
    double yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation,quat);
    tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);

    double distance_err= sqrt(pow(this->x-msg->pose.pose.position.x,2)+pow(this->y-msg->pose.pose.position.y,2));
    double theta=atan2(this->y-msg->pose.pose.position.y,this->x-msg->pose.pose.position.x);

    double theta_err=theta-yaw;
    printf("theta_err: %f \n",theta_err);

    printf("distance error %f \n",distance_err);

    while (theta < -1.0 * angle_wrap / 2.0)
        theta_err += angle_wrap;
    while (theta > angle_wrap/ 2.0)
        theta_err -= angle_wrap;
    

    angle_err.integral_error=angle_err.integral_error+theta_err;
    double p_angle=this->angle.kp*theta_err;
    double d_angle=this->angle.kd*(theta_err-angle_err.previous_error);
    double i_angle=this->angle.ki*angle_err.integral_error;
    double gain=p_angle+i_angle+d_angle;

    dist_err.integral_error=dist_err.integral_error+distance_err;
    double p_dist=this->distance.kp*distance_err;
    double d_dist=this->distance.kd*(distance_err-dist_err.previous_error);
    double i_dist=this->distance.ki*dist_err.integral_error;
    double pid_dist=p_dist+i_dist+d_dist;

    if(abs(theta_err)>0.05)
    {
        // if (abs(gain)>this->angle.velocity_max)
            
        //     if(gain>0)
        //         gain=this->angle.velocity_max;
        //     else
        //         gain=-this->angle.velocity_max;
            
        // else
        //     this->speed.angular.z=gain;
         this->speed.angular.z=0.08;
    }
    else
    {
        this->speed.angular.z=0;
        //if(distance_err>0.5)
        //speed.linear.x=0.3;
        //else
        //speed.linear.x=0;

        //if(pid_dist-this->distance.previous_state<this->distance.acceleration_max)
        //{

        //     if(pid_dist<this->distance.velocity_max && pid_dist>this->distance.velocity_min)
        //         this->speed.linear.x=pid_dist;
        //     else if(pid_dist>this->distance.velocity_max)
        //         this->speed.linear.x = this->distance.velocity_max;
        //     else
        //         this->speed.linear.x  = this->distance.velocity_min;

            
        // }
        // else
        //     this->speed.linear.x=this->distance.acceleration_max;

        //}
    }

    pub.publish(this->speed);  
    this->dist_err.previous_error=distance_err;
    this->angle_err.previous_error=theta_err;
}
#endif