// Include header
#include "headers/odom.h"

// Function for handling ROS shutdown
void odom_shutdown(int sig)
{
    ros::shutdown();
}

void left_wheel_callback(const std_msgs::Float32::ConstPtr& vel)
{
    odom::u(1, 0) = vel->data;
}

void right_wheel_callback(const std_msgs::Float32::ConstPtr& vel)
{
    odom::u(0, 0) = vel->data;
}

int main(int argc, char** argv)
{
    // Start ROS node, handle and rate
    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh;
    ros::Rate rate(100);
    
    // Set shutdown function
    signal(SIGINT, odom_shutdown);

    // Get ROS params
    std::string left_topic;
    nh.param<std::string>("/navigation/topics/vel_l", left_topic, "/WOMBAT/navegation/leftSpeed");

    std::string right_topic;
    nh.param<std::string>("/navigation/topics/vel_r", right_topic, "/WOMBAT/navegation/rightSpeed");

    double r, d;

    nh.param<double>("/navigation/model_args/radius", r, 0.05);

    nh.param<double>("/navigation/model_args/distance", d, 0.08);

    // Create odometry handler

    Odometry odomh = Odometry(r, d);

    // Create ROS subscribers
    ros::Subscriber left_sub = nh.subscribe(left_topic, 10, left_wheel_callback);
    ros::Subscriber right_sub = nh.subscribe(right_topic, 10, right_wheel_callback);

    // Create ROS publisher and message
    ros::Publisher pose_2d_pub = nh.advertise<geometry_msgs::Pose2D>("/WOMBAT/navegation/odometry", 10);
    geometry_msgs::Pose2D pose_2d_msg;

    // Initialize t and u
    odom::u << 0,
               0;
    ros::Time t = ros::Time::now();
    
    // Define time differential and odometry matrix
    double dt;
    Eigen::Matrix<double, 3, 1> q;

    // Call ROS spin once
    while(ros::ok()) {
        
        // Check for topic messages
        ros::spinOnce();

        // Get the time difference
        dt = ros_utils::dt_and_swp(t, ros::Time::now());

        // Compute odometry
        q = odomh.compute_odom(odom::u, dt); 

        // Set values to message and publish it
        pose_2d_msg.x     = q(0, 0);
        pose_2d_msg.y     = q(1, 0);
        pose_2d_msg.theta = q(2, 0);

        pose_2d_pub.publish(pose_2d_msg);
        
        // Sleep
        rate.sleep();
    }

    return 0;
}

    
