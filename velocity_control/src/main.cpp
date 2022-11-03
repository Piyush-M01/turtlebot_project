//inorder to run pass data using serial communication, connect usb port to STM and start roscore. Then run command: rosrun velocity_control velocity_serial


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
