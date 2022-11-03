#include<iostream>
#include "../include/velocity_controller.hpp"
#include "velocity.cpp"
using namespace std;
int main(int argc, char* argv[])
{
    ros::init(argc,argv,"velocity_controller");
    
    ros::NodeHandlePtr nh;

    nh = boost::make_shared<ros::NodeHandle>();

    dynamic_reconfigure::Server<velocity_control::dynamicConfig> server;
    dynamic_reconfigure::Server<velocity_control::dynamicConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    velocityController::velocity_state_controller ob(nh);

    ros::spin();
}
