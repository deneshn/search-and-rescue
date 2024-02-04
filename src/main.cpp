/**
 * @file main.cpp
 * @author WeiLun Hsu Denesh Nallur Narasimman Pranav Prakash Mare
 * @brief 
 * start the project
 * @version 0.1
 * @date 2021-12-15
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <ros/ros.h>
#include "explorer_robot.h"
#include "follower_robot.h"
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;

  ExplorerRobot explorer(nh);
  FollowerRobot follower;

  explorer.run();
  follower.run();

  ROS_INFO("Congratulations! Task finished!");

  ros::shutdown();
}