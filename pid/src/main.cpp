#include "../include/pid/pid.hpp"
#include "pid.cpp"
using namespace std;
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "control");
    ros::NodeHandle nh;    

    double max_vel,min_vel,max_angular_acc,max_distance_acc;
    string pub_topic, sub_topic;

    double kp_angle, ki_angle, kd_angle, kp_distance, ki_distance, kd_distance;

    double max_angle;
    
    std::cout<<" maximum_velocity \n minimum_velocity \n maximum_linear_acceleration \n maximum_angular_acceleration \n";
    std::cin>>max_vel>>min_vel>>max_distance_acc>>max_angular_acc;

    std::cout<<" publisher_topic \n subscriber_topic\n";
    std::cin>>pub_topic>>sub_topic;
    
    std::cout<<" kp_angle \n ki_angle \n kd_angle \n kp_distance \n ki_distance \n kd_distance\n";
    std::cin>>kp_angle>>ki_angle>>kd_angle>>kp_distance>>ki_distance>>kd_distance;

    std::cout<<" maximum angle \n";
    std::cin>>max_angle;
    
    PID::pid ob(nh);

  
    double x,y;
    nh.setParam("parameters/kp_angle", kp_angle);
    nh.setParam("parameters/ki_angle", ki_angle);
    nh.setParam("parameters/kd_angle",kd_angle);
    nh.setParam("parameters/angle_velocity_max",max_angle);
    nh.setParam("parameters/angle_acceleration_max",max_angular_acc);

    nh.setParam("parameters/kp_distance",kp_distance);
    nh.setParam("parameters/ki_distance", ki_distance);
    nh.setParam("parameters/kd_distance", kd_distance);
    nh.setParam("parameters/distance_velocity_min",min_vel);
    nh.setParam("parameters/distance_velocity_max",max_vel);
    nh.setParam("parameters/distance_acceleration_max",max_distance_acc);

    nh.setParam("parameters/publisher",pub_topic);
    nh.setParam("parameters/subscriber",sub_topic);

    cout<<"Enter setpoint X \n setpoint Y \n";
    cin>>x>>y;

    nh.setParam("setpoint_x",x);
    nh.setParam("setpoint_y",y);
    
    ros::Rate loop_rate(100);
    ros::spin();
}
