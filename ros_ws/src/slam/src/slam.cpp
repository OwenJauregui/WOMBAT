#include "headers/slam.h"
#include <iostream>

void slam_shutdown(int sig)
{
    delete slam::icp_h;

    ros::shutdown(); 
}

void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& laser)
{   
    // Check number of meassurements
    int point_cloud_size = sizeof(laser->ranges)/sizeof(float);
    
    // Extract laser polar meassurements
    Eigen::MatrixX2d polar_coords;
    
    Eigen::Map<const Eigen::VectorXf> aux_vec(laser->ranges.data(), laser->ranges.size());
    polar_coords.col(0) = aux_vec.cast<double>();
    polar_coords.col(1) = utils::linspace(laser->angle_min, laser->angle_max, point_cloud_size);
    
    // Convert to cartesian points
    Eigen::MatrixX2d points = utils::polar_to_cartesian(polar_coords);
    
    // Estimate transform using ICP
    Eigen::Matrix3d odom_tf = slam::icp_h->icp(points);

    // Decompose tf into x, y, theta
    slam::icp_odom.x     = odom_tf(0, 2);
    slam::icp_odom.y     = odom_tf(1, 2);
    slam::icp_odom.theta = acos(odom_tf(0, 0));
}

int main(int argc, char** argv)
{
    // Start ROS node and node handle
    ros::init(argc, argv, "slam");
    ros::NodeHandle nh;

    // Set shutdown function
    signal(SIGINT, slam_shutdown);

    slam::icp_h = new ICP();

    // Create ROS subscribers
    ros::Subscriber lidar_sub = nh.subscribe("/scan", 10, lidar_callback);

    // Create ROS publishers
    slam::icp_pub = nh.advertise<geometry_msgs::Pose2D>("/WOMBAT/SLAM/odometry", 10);

    // Call ROS spin
    ros::spin();

    return 0;

}

