#include<iostream>
#include "../include/velocity_controller.hpp"
using namespace std;
int main(int argc, char* argv[])
{
    ros::init(argc,argv,"velocity_controller");
    double linear_velocity,angular_velocity;
    cout<<"Enter: \nLinear velocity: \nAngular velocity\n";
    cin>>linear_velocity>>angular_velocity;

    ros::NodeHandle nh;
    nh.setParam("/velocity/linear_velocity",linear_velocity);
    nh.setParam("/velocity/angular_velocity",angular_velocity);
    ros::spin();
}