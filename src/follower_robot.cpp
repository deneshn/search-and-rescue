/**
 * @file follower_robot.cpp
 * @author WeiLun Hsu Denesh Nallur Narasimman Pranav Prakash Mare
 * @brief 
 * @version 0.1
 * @date 2021-12-15
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "follower_robot.h"

extern std::array<std::pair<double, double>, num_goals> marker_locations; // locations of 4 detected markers

FollowerRobot::FollowerRobot():
    tolerance(0.4),
    start_x(-4.0), 
    start_y(3.5), 
    follower_client("/follower/move_base", true)
{
    // wait for the action server to come up
    ROS_INFO("Follower waits for move base action server to start.");
    follower_client.waitForServer();
    ROS_INFO("Move_base action server started");
}

void FollowerRobot::run()
{
    visit_markers();
    return_back();
}



/**
 * @brief 
 * Visit all markers
 */
void FollowerRobot::visit_markers()
{
    move_base_msgs::MoveBaseGoal follower_goal;
    for(int i = 0; i < num_goals; i++)
    {
        //Build goal for follower
        follower_goal.target_pose.header.frame_id = "map";
        follower_goal.target_pose.header.stamp = ros::Time::now();
        
        if(marker_locations[i].first > 0)
        {
            follower_goal.target_pose.pose.position.x = marker_locations[i].first - tolerance;
        }
        else
        {
            follower_goal.target_pose.pose.position.x = marker_locations[i].first + tolerance;
        }

        if(marker_locations[i].second > 0)
        {
            follower_goal.target_pose.pose.position.y = marker_locations[i].second - tolerance;
        }
        else
        {
            follower_goal.target_pose.pose.position.y = marker_locations[i].second + tolerance;
        }

        follower_goal.target_pose.pose.orientation.w = 1.0;

        ROS_INFO("Sending goal %d for follower", i);
        follower_client.sendGoal(follower_goal);

        while(ros::ok()) 
        {
            if (follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
            {
                ROS_INFO("Follower robot reached goal %d", i);
                break;
            }
        }
    }
}



/**
 * @brief 
 * Follower returns back to start position
 */
void FollowerRobot::return_back()
{   
    move_base_msgs::MoveBaseGoal follower_goal;

    //Build goal for follower
    follower_goal.target_pose.header.frame_id = "map";
    follower_goal.target_pose.header.stamp = ros::Time::now();
    follower_goal.target_pose.pose.position.x = start_x;
    follower_goal.target_pose.pose.position.y = start_y;
    follower_goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal (start position) for follower");
    follower_client.sendGoal(follower_goal);//this should be sent only once

    while(ros::ok()) 
    {
        if (follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Follower robot reached start position");
            break;
        }
    }
}