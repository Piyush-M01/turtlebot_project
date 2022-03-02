#include<iostream>
#include "../include/velocity_controller.hpp"
#include "velocity.cpp"
using namespace std;
int main(int argc, char* argv[])
{
    ros::init(argc,argv,"velocity_controller");
    ros::NodeHandle nh;
    velocityController::velocity_state_controller ob(nh);
    double linear_velocity,angular_velocity;
    cout<<"Enter: \nLinear velocity: \nAngular velocity\n";
    cin>>linear_velocity>>angular_velocity;

    cout<<"Enter: \nMaximum linear velocity: \nMaximum angular velocity\n";
    cin>>ob.velocity.velocity_max>>ob.angular_velocity.velocity_max;

    cout<<"Enter: \nMinimum linear velocity: \nMinimum angular velocity\n";
    cin>>ob.velocity.velocity_min>>ob.angular_velocity.velocity_min;

    nh.setParam("/velocity/linear_velocity",linear_velocity);
    nh.setParam("/velocity/angular_velocity",angular_velocity);

    ros::Rate r(1.0);
    ros::spin();
}