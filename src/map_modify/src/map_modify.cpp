#include "map_modify/map_modify.hpp"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_modify");
  MODMAP modmap;
  ros::spin();
  return 0;
}