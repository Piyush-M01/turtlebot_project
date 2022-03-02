#include<iostream>
#include "../include/velocity_controller.hpp"
#include "velocity.cpp"
using namespace std;
int main(int argc, char* argv[])
{
    ros::init(argc,argv,"velocity_controller");
    
    ros::NodeHandlePtr nh;

    nh = boost::make_shared<ros::NodeHandle>();

    velocityController::velocity_state_controller ob(nh);

    ros::spin();
}