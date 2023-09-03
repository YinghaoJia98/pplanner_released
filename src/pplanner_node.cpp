#include <ros/ros.h>
#include "pplanner/pplanner.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pplanner_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  PPlanner pplanner(nh, nh_private);
  ROS_INFO("Hello world, the pplanner is starting.");
  int spinner_thread;
  nh.param<int>("pplanner_settings/spinner_thread", spinner_thread, 1);
  ros::AsyncSpinner spinner(spinner_thread); // Use n threads
  spinner.start();
  ros::waitForShutdown();

  ROS_INFO("Hello world, the pplanner is closing.");
  return 0;
}