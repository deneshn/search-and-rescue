/**

#ifndef FOLLOWER_ROBOT_H
#define FOLLOWER_ROBOT_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#ifndef NUM_GOALS
#define num_goals 4
#endif

class FollowerRobot
{
public:
    FollowerRobot();

    void run();

private:
    // Visit all markers
    void visit_markers();

    // Follower returns back to start position
    void return_back();

    //// private variables
    double tolerance;  // tolerance for reaching markers
    double start_x, start_y; // start position

    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    MoveBaseClient follower_client;
};
#endif // FOLLOWER_ROBOT_H
