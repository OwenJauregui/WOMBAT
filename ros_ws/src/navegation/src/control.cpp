#include "headers/control.h"

void control_shutdown(int sig)
{
    delete control::ctr_h;
    
    // Stop the motors
    control::right_msg.data = 0;
    control::left_msg.data  = 0;

    control::right_pub.publish(control::right_msg);
    control::left_pub.publish(control::left_msg);

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
    double dt = ros_utils::dt_and_swp(control::t, ros::Time::now());

    // Make control calculations
    Eigen::Matrix<double, 2, 1> u = control::ctr_h->control_position(q, control::qd, dt);

    // Set values to messages and publish them
    control::right_msg.data = u(0,0);
    control::left_msg.data  = u(1,0);

    control::right_pub.publish(control::right_msg);
    control::left_pub.publish(control::left_msg);
}

void goal_callback(const geometry_msgs::Pose2D::ConstPtr& goal)
{
    control::qd << goal->x,
                   goal->y;
}

int main(int argc, char** argv)
{
    // Start ROS node and node handle
    ros::init(argc, argv, "control");
    ros::NodeHandle nh;
    
    // Set shutdown function
    signal(SIGINT, control_shutdown);

    // Get ROS params
    std::string left_topic;
    nh.param<std::string>("/navigation/topics/vel_l", left_topic, "/WOMBAT/navegation/leftSpeed");

    std::string right_topic;
    nh.param<std::string>("/navigation/topics/vel_r", right_topic, "/WOMBAT/navegation/rightSpeed");

    double r, d, h;

    nh.param<double>("/navigation/model_args/radius", r, 0.05);

    nh.param<double>("/navigation/model_args/distance", d, 0.08);

    nh.param<double>("/navigation/model_args/displacement", h, 0.02);
    
    // Asign constant control values
    double k1, k2;
    k1 = 0.1;
    k2 = 0.2;

    control::ctr_h = new Control(k1, k2, r, d, h);

    // Create ROS subscribers
    
    //WOMBAT/navegation/pose2d
    ros::Subscriber pose_sub = nh.subscribe("/slam_out_pose", 10, pose_callback);
    ros::Subscriber goal_sub = nh.subscribe("/WOMBAT/navegation/goal", 10, goal_callback);

    // Create ROS publishers
    control::left_pub = nh.advertise<std_msgs::Float32>(left_topic, 10);
    control::right_pub = nh.advertise<std_msgs::Float32>(right_topic, 10);

    // Initialize t and qd
    control::qd << 0,
                   0;
    control::t = ros::Time::now();

    // Call ROS spin
    ros::spin();

    return 0;
}

    
