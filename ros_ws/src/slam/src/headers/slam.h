#ifndef SLAM_NODE_H
#define SLAM_NODE_H

// Include ICP library
#include "slam/icp.h"

// Include ROS libaries
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>

#include <signal.h>

namespace slam {
    // Odometry message
    static geometry_msgs::Pose2D icp_odom;

    // Odometry publisher
    static ros::Publisher icp_pub;

    // ICP handler
    static ICP* icp_h; 
}

void slam_shutdown(int sig);

void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& laser);

int main(int argc, char** argv);

#endif

