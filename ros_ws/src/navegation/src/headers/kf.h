#ifndef NAVEGATION_KALMAN_NODE
#define NAVEGATION_KALMAN_NODE

// Include custom libaries
#include "navegation/utils.h"
#include "navegation/ros_utils.h"

// Include for shutdown
#include <signal.h>

// Include msgs
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"

namespace kf {

    // Variables for Kalman usage
    static ros::Time t;
    static Eigen::Matrix<double, 2, 1> u;

    // ROS publishers and messages
    static ros::Publisher pose_2d_pub;
    static ros::Publisher pose_pub;
    static geometry_msgs::Pose2D pose_2d_msg;
    static geometry_msgs::PoseStamped pose_msg;    

    // Kalman handler
    static Kalman* kh;
}

void pub_pose(Eigen::Matrix<double, 3, 1>& x_hat);

void kf_shutdown(int sig);

void lidar_odom_callback(const geometry_msgs::PoseStamped::ConstPtr& pose);

void left_wheel_callback(const std_msgs::Float32::ConstPtr& vel);

void right_wheel_callback(const std_msgs::Float32::ConstPtr& vel);

int main(int argc, char** argv);

#endif
