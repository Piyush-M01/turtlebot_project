// C library headers
#include "serial_init.cpp"
int main(int argc, char **argv) {
  ros::init(argc, argv, "serial_node");
  Serial ob;
  ros::spin();
  return 0; // success
};