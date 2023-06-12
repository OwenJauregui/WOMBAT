#ifndef SLAM_NODE_H
#define SLAM_NODE_H

// Include ICP library
#include "slam/icp.h"

// Include ROS libaries
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>

#include <signal.h>

namespace slam {
    // Odometry messages
    static geometry_msgs::Pose2D icp_odom;
    static geometry_msgs::PoseStamped icp_pose;

    // Odometry publishers
    static ros::Publisher icp_pub;
    static ros::Publisher pose_pub;

    // ICP handler
    static ICP* icp_h;

    // Previous transform to origin
    static Eigen::Matrix3d origin_tf; 
}

void slam_shutdown(int sig);

void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& laser);

int main(int argc, char** argv);

#endif

