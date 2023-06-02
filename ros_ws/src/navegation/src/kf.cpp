#include "headers/kf.h"

void kf_shutdown(int sig)
{
    delete kf::kh;
    ros::shutdown();
}

void pose_callback(const geometry_msgs::Pose2D::ConstPtr& pose)
{
    // Read values from Pose2D message
    Eigen::Matrix<double, 3, 1> q; 
    q << pose->x,
         pose->y,
         pose->theta;

    // Get the time difference
    double dt = ros_utils::dt_and_swp(kf::t, ros::Time::now());

    // Make Kalman Filter estimation
    Eigen::Matrix<double, 3, 1> x_hat = kf::kh->estimate(q, kf::u, dt);

    // Set values to messages and publish them
    kf::pose_msg.header.stamp = kf::t;

    kf::pose_msg.pose.position.x = kf::pose_2d_msg.x = x_hat(0, 0);
    kf::pose_msg.pose.position.y = kf::pose_2d_msg.y = x_hat(1, 0);
    
    kf::pose_2d_msg.theta = x_hat(2, 0);
    
    double* quat = utils::euler_to_quat(0, 0, x_hat(2, 0));
    
    kf::pose_msg.pose.orientation.w = quat[0];
    kf::pose_msg.pose.orientation.z = quat[3];

    delete quat;

    kf::pose_2d_pub.publish(kf::pose_2d_msg);
    kf::pose_pub.publish(kf::pose_msg);
}

void left_wheel_callback(const std_msgs::Float32::ConstPtr& vel)
{
    kf::u(1, 0) = vel->data;
}

void right_wheel_callback(const std_msgs::Float32::ConstPtr& vel)
{
    kf::u(0, 0) = vel->data;
}

int main(int argc, char** argv)
{
    // Start ROS node and node handle
    ros::init(argc, argv, "kalman_filter");
    ros::NodeHandle nh;
    
    // Set shutdown function
    signal(SIGINT, kf_shutdown);

    // Get ROS params
    std::string left_topic;
    nh.param<std::string>("/navigation/topics/vel_l", left_topic, "/WOMBAT/navegation/leftSpeed");

    std::string right_topic;
    nh.param<std::string>("/navigation/topics/vel_r", right_topic, "/WOMBAT/navegation/rightSpeed");

    double kns[3];

    nh.param<double>("/navigation/model_args/radius", kns[0], 1);

    nh.param<double>("/navigation/model_args/distance", kns[1], 1);

    nh.param<double>("/navigation/model_args/displacement", kns[2], 1);

    // Create kalman filter handler

    Eigen::Matrix<double, 3, 3> Q; 
    Q << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;

    Eigen::Matrix<double, 3, 3> R;
    R << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;

    Eigen::Matrix<double, 3, 3> H;
    H << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;

    kf::kh = new Kalman(Q, R, H, kns);

    // Create ROS subscribers
    ros::Subscriber pose_sub = nh.subscribe("/WOMBAT/navegation/odometry", 10, pose_callback);
    ros::Subscriber left_sub = nh.subscribe(left_topic, 10, left_wheel_callback);
    ros::Subscriber right_sub = nh.subscribe(right_topic, 10, right_wheel_callback);

    // Create ROS publishers
    kf::pose_2d_pub = nh.advertise<geometry_msgs::Pose2D>("/WOMBAT/navegation/pose2d", 10);

    kf::pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/WOMBAT/navegation/pose", 10);

    // Initialize t and u
    kf::u << 0,
             0;
    kf::t = ros::Time::now();
    
    // Initialize header values for msg
    kf::pose_msg.header.frame_id = "world";

    // Call ROS spin once
    ros::spin();

    return 0;
}

    
