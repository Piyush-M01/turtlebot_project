#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <turtlebot_controller/configConfig.h>

void callback(turtlebot_controller::configConfig &config, uint32_t level) 
{
    //ROS_INFO("Reconfigure Request: %f %f %f ",config.p_gain, config.i_gain, config.d_gain); 
}
