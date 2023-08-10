#ifndef NAVEGATION_CONTROL_NODE
#define NAVEGATION_CONTROL_NODE

// Include custom libaries
#include "navegation/utils.h"
#include "navegation/ros_utils.h"

// Include for shutdown
#include <signal.h>

// Include msgs
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"

namespace control {

    // Variables for Control usage
    static ros::Time t;
    static Eigen::Matrix<double, 2, 1> qd;

    // ROS publishers and messages
    static ros::Publisher left_pub;
    static ros::Publisher right_pub;
    static std_msgs::Float32 left_msg;
    static std_msgs::Float32 right_msg;
    
    // Control handler
    static Control* ctr_h;
}

void control_shutdown(int sig);

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose);

void goal_callback(const geometry_msgs::Pose2D::ConstPtr& goal);

int main(int argc, char** argv);

#endif
