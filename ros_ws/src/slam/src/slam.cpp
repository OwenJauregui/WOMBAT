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
    int point_cloud_size = laser->ranges.size();
    
    // Extract laser polar meassurements
    Eigen::MatrixX2d polar_coords(point_cloud_size, 2);
    
    Eigen::Map<const Eigen::VectorXf> aux_vec(laser->ranges.data(), laser->ranges.size());
    polar_coords.col(0) = aux_vec.cast<double>();
    polar_coords.col(1) = utils::linspace(laser->angle_min, laser->angle_max, point_cloud_size);
    
    Eigen::VectorXi finite = (polar_coords.col(0).array().isFinite() && !(polar_coords.col(0).array().isNaN())).cast<int>();

    // Select only valid points
    Eigen::MatrixX2d polar_valid(finite.sum(), 2);
    int new_row = 0;
    for(int i=0; i < polar_valid.rows(); i++) {
    	if(finite[i]) {
	        polar_valid.row(new_row) = polar_coords.row(i);
	        new_row++;
	    }
    }

    std::cout <<"Infinite: "<<polar_valid.array().isInf().any()<<" NAN: "<<polar_valid.array().isNaN().any()<<std::endl;


    // Convert to cartesian points
    Eigen::MatrixX2d points = utils::polar_to_cartesian(polar_valid);
    
    // Estimate transform using ICP
    Eigen::Matrix3d odom_tf = slam::icp_h->icp(points);

    // Compute the transform from origin
    slam::origin_tf *= odom_tf;

    // Decompose tf into x, y, theta
    slam::icp_odom.x     = slam::origin_tf(0, 2);
    slam::icp_odom.y     = slam::origin_tf(1, 2);
    slam::icp_odom.theta = acos(slam::origin_tf(0, 0));

    slam::icp_pub.publish(slam::icp_odom);

    // Convert to pose
    slam::icp_pose.header.stamp = ros::Time::now();
    slam::icp_pose.pose.position.x = slam::icp_odom.x; 
    slam::icp_pose.pose.position.y = slam::icp_odom.y;

    double* quat = utils::euler_to_quat(0, 0, slam::icp_odom.theta);

    slam::icp_pose.pose.orientation.w = quat[0];
    slam::icp_pose.pose.orientation.z = quat[3];

    delete quat;

    slam::pose_pub.publish(slam::icp_pose);
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

    // Initialize first transform to origin
    slam::origin_tf << 1, 0, 0,
                       0, 1, 0,
                       0, 0, 1;

    // Create ROS publishers
    slam::icp_pub = nh.advertise<geometry_msgs::Pose2D>("/WOMBAT/SLAM/odometry", 10);
    slam::pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/WOMBAT/SLAM/pose", 10);

    // Set frame ID
    slam::icp_pose.header.frame_id = "world";

    // Call ROS spin
    ros::spin();

    return 0;

}

