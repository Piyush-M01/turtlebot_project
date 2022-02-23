#ifndef _PID_H_
#define _PID_H_
#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <math.h>
class Pidcontrol
{
    public:
    Pidcontrol(ros::NodeHandle &nh);
    void OdomCB(const nav_msgs::Odometry::ConstPtr& ptr);
    double angle_PID(double theta_err);
    double distance_PID(double distance_err);
    void angular_constraint(double gain);
    void fix_distance(double pid_dist);


    protected:
    double kp=0.5,kd=0.02,ki=0.05,x=1,y=1,integral_dist;
    double theta,integral_angle,kp_angle=0.016,kd_angle=0.005,ki_angle=0.003;
    double prev_error,prev_vel=0,prev_theta_err;
    double vel_max = 0.1,acc_max = 0.1,vel_min = 0.05;
    bool is_fixed=false;
    geometry_msgs::Twist speed;

    private:
    ros::Publisher pub;
    ros::Subscriber sub;
    
};
#endif