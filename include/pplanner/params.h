
#ifndef PARAMS_H_
#define PARAMS_H_

#include <stdio.h>

#include <iostream>
#include <unordered_map>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

typedef ros::Time TIMER;
#define START_TIMER(x) (x = ros::Time::now())
#define GET_ELAPSED_TIME(x) (float)((ros::Time::now() - x).toSec())

enum Verbosity
{
  SILENT = 0,
  PLANNER_STATUS = 1,
  ERROR = 2,
  WARN = 3,
  INFO = 4,
  DEBUG = 5
};

#define global_verbosity Verbosity::ERROR
#define param_verbosity Verbosity::SILENT

typedef Eigen::Vector4d StateVec;
typedef Eigen::Matrix<double, 5, 1> ExplorationGainVec;

typedef Eigen::Vector4d PointAttributeMembershipDegreeVec;

#endif
