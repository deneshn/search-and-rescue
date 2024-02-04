/**
 * @file explorer_robot.cpp
 * @author WeiLun Hsu Denesh Nallur Narasimman Pranav Prakash Mare
 * @brief 
 * @version 0.1
 * @date 2021-12-15
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "explorer_robot.h"

#include <geometry_msgs/Twist.h>
#include <XmlRpcValue.h>
#include <string>

std::array<std::pair<double, double>, num_goals> marker_locations;
ros::Time detect_start_time;
bool wait_for_valid = true;

/**
 * @brief Construct a new Explorer Robot:: Explorer Robot object
 * 
 * @param nodehandle 
 */
ExplorerRobot::ExplorerRobot(ros::NodeHandle &nodehandle):
    m_nh(nodehandle),
    start_x(-4.0), start_y(2.5),
    explorer_client("/explorer/move_base", true),
    marker_detected(false), 
    target_reached(false),
    target_count(0)
{
    // wait for the action server to come up
    ROS_INFO("Explorer waits for move base action server to start.");
    explorer_client.waitForServer();
    ROS_INFO("Move_base action server started");

    // initialize pub and sub
    fiducial_sub = m_nh.subscribe("/fiducial_transforms", 1000, &ExplorerRobot::fiducial_callback, this);
    rotation_pub = m_nh.advertise<geometry_msgs::Twist>("explorer/cmd_vel", 1000);

    tfListener = new tf2_ros::TransformListener(tfBuffer);
}

/**
 * @brief 
 * 5.1Read target locations from the Parameter Server
 * 5.2 & 5.3 Move to a target and detect marker
 * 5.4 Explorer returns back to start position
 */
void ExplorerRobot::run()
{
    // 5.1 Read target locations from the Parameter Server
    read_target_locations();

    // 5.2 & 5.3 Move to a target and detect marker
    move_and_detect();

    // 5.4 Explorer returns back to start position
    return_back();
}

/**
 * @brief 
 * callback
 * @param msg 
 */
void ExplorerRobot::fiducial_callback(
    const fiducial_msgs::FiducialTransformArray::ConstPtr& msg)
{
    if(wait_for_valid)
        return;

    if (target_reached && !marker_detected && !msg->transforms.empty())  //check marker is detected
    {
        double msg_timestamp = msg->header.stamp.toSec();

        if(msg_timestamp > detect_start_time.toSec())
        {        
            // 5.3.1 Broadcaster && 5.3.2 Listener
            broadcast(msg);
            listen(msg->transforms[0].fiducial_id);
            ros::Duration(1.0).sleep();
            return;
        }
    }
}

/**
 * @brief 
 * broadcast
 * @param msg 
 */
void ExplorerRobot::broadcast(
    const fiducial_msgs::FiducialTransformArray::ConstPtr& msg)
{
    //broadcaster object
    static tf2_ros::TransformBroadcaster br; 
    geometry_msgs::TransformStamped transformStamped;  
    
    //broadcast the new frame to /tf Topic
    transformStamped.header.stamp = ros::Time::now(); 
    transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame"; 
    transformStamped.child_frame_id = "marker_frame"; //name of the frame 
    transformStamped.transform.translation.x = msg->transforms[0].transform.translation.x;
    transformStamped.transform.translation.y = msg->transforms[0].transform.translation.y;
    transformStamped.transform.translation.z = msg->transforms[0].transform.translation.z;
    transformStamped.transform.rotation.x = msg->transforms[0].transform.rotation.x;
    transformStamped.transform.rotation.y = msg->transforms[0].transform.rotation.y;
    transformStamped.transform.rotation.z = msg->transforms[0].transform.rotation.z;
    transformStamped.transform.rotation.w = msg->transforms[0].transform.rotation.w;
    ROS_INFO("Broadcasting");
    br.sendTransform(transformStamped); //broadcast the transform on /tf Topic
}

void ExplorerRobot::listen(int32_t id)
{
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer.lookupTransform("map", "marker_frame", ros::Time(0));
        auto trans_x = transformStamped.transform.translation.x;
        auto trans_y = transformStamped.transform.translation.y;
        auto trans_z = transformStamped.transform.translation.z;

        ROS_INFO_STREAM("marker in /map frame: ["
        << trans_x << ","
        << trans_y << ","
        << trans_z << "]"
        );

        // Save detected marker location [x, y]
        if(trans_x - target_locations[target_count].first > 2.0 || trans_y - target_locations[target_count].second > 2.0)
        {
            marker_detected = false;
        }
        else
        {
            marker_locations[id] = std::make_pair(trans_x, trans_y);
            marker_detected = true;
        }
    }
    catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}


void ExplorerRobot::read_target_locations()
{
    XmlRpc::XmlRpcValue array;

    for(int32_t i = 1; i <= num_goals; i++)
    {
        m_nh.getParam("/aruco_lookup_locations/target_" + std::to_string(i), array);

        ROS_ASSERT(array.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(array[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(array[1].getType() == XmlRpc::XmlRpcValue::TypeDouble);

        target_locations[i - 1] = std::make_pair(static_cast<double>(array[0]), static_cast<double>(array[1]));
    }
}

/**
 * @brief 
 * 5.2 Move to a target
 * 5.3 Rotate to detect a marker
 */
void ExplorerRobot::move_and_detect()
{   
    move_base_msgs::MoveBaseGoal explorer_goal;

    for(int i = 0; i < num_goals; i++)
    {
        // 5.2 Move to a target
        // 1) Build goal for explorer
        explorer_goal.target_pose.header.frame_id = "map";
        explorer_goal.target_pose.header.stamp = ros::Time::now();
        explorer_goal.target_pose.pose.position.x = target_locations[i].first;
        explorer_goal.target_pose.pose.position.y = target_locations[i].second;
        explorer_goal.target_pose.pose.orientation.w = 1.0;

        // 2) Send a goal for explorer
        ROS_INFO("Sending goal %d for explorer", i);
        explorer_client.sendGoal(explorer_goal);

        while(ros::ok()) 
        {
            if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
            {
                ROS_INFO("Explorer robot reached goal %d", i);
                target_reached = true;
                break;
            }
        }

        // Stop moving
        geometry_msgs::Twist stop_msg;
        stop_msg.linear.x = 0;
        stop_msg.angular.z = 0;
        rotation_pub.publish(stop_msg);
        ros::Duration(1.0).sleep();
        ROS_INFO("Robot stops");

        detect_start_time = ros::Time::now();

        ros::spinOnce();

        wait_for_valid = false;

        // 5.3 Rotate to detect a marker
        while(target_reached && !marker_detected)
        {
            ROS_INFO("Marker detecting!");
            geometry_msgs::Twist rotation_msg;
            rotation_msg.linear.x = 0;
            rotation_msg.angular.z = 0.1;

            rotation_pub.publish(rotation_msg);
            ros::spinOnce();
            ros::Duration(1.0).sleep();
        }
        ROS_INFO("Marker %d detected!", i);

        marker_detected = false;
        target_reached = false;
        wait_for_valid = true;

        target_count++;
    }
}



/**
 * @brief 
 * 5.4 Explorer returns back to start position
 */
void ExplorerRobot::return_back()
{   
    move_base_msgs::MoveBaseGoal explorer_goal;

    //Build goal for explorer
    explorer_goal.target_pose.header.frame_id = "map";
    explorer_goal.target_pose.header.stamp = ros::Time::now();
    explorer_goal.target_pose.pose.position.x = start_x;
    explorer_goal.target_pose.pose.position.y = start_y;
    explorer_goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal (start position) for explorer");
    explorer_client.sendGoal(explorer_goal);//this should be sent only once

    while(ros::ok()) 
    {
        if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
        {
            ROS_INFO("Explorer robot reached start position");
            break;
        }
    }
}