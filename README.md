# turtlebot_project
   
# Run the serial interfacing with ros by virtual com port using the following commands:   
1) roslaunch velocity_control launch.xml   
2) rosrun velocity_control velocity_controller   
3) rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel_filtered   
4) rosrun velocity_control velocity_serial   
