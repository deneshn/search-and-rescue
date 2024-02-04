/**

#ifndef Explorer_ROBOT_H
#define Explorer_ROBOT_H

#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>

#ifndef NUM_GOALS
#define num_goals 4
#endif

extern std::array<std::pair<double, double>, num_goals> marker_locations; // locations of 4 detected markers

class ExplorerRobot
{
public:
    ExplorerRobot(ros::NodeHandle &nodehandle);

    void run();

private:
    void fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg);

    void broadcast(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg);
    void listen(int32_t id);
    
    void read_target_locations(); // 5.1 Read target locations from the PS
    void move_and_detect(); // 5.2 & 5.3 Move to a target and detect marker
    void return_back(); // 5.4 Explorer returns back to start position

    ///// private variables
    ros::NodeHandle m_nh;
    ros::Subscriber fiducial_sub;
    ros::Publisher rotation_pub;
    
    double start_x, start_y; // start position

    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    MoveBaseClient explorer_client;

    bool marker_detected;// one marker has been detected or not
    bool target_reached; // explorer have arrived at the target location or not
    std::array<std::pair<double, double>, num_goals> target_locations; // target locations read from the Parameter Server

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *tfListener; // these two parameters should be class member variables, otherwise error occurs if set as local parameters

    int target_count; // count for target id we are detecting
};
#endif // Explorer_ROBOT_H
