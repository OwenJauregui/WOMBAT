#include "odom_tf.h"

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // Start tf broadcaster for odom to base_link and tf message
    static tf2_ros::StaticTransformBroadcaster odom_to_base;

    geometry_msgs::TransformStamped tf_msg;

    // Read values from pose and set them as tf

    tf_msg.header.stamp = msg->header.stamp;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_link";
    tf_msg.transform.translation.x = msg->pose.position.x;
    tf_msg.transform.translation.y = msg->pose.position.y;
    tf_msg.transform.translation.z = msg->pose.position.z;
    tf_msg.transform.rotation.x = msg->pose.orientation.x;
    tf_msg.transform.rotation.y = msg->pose.orientation.y;
    tf_msg.transform.rotation.z = msg->pose.orientation.z;
    tf_msg.transform.rotation.w = msg->pose.orientation.w;
    odom_to_base.sendTransform(tf_msg);
}

int main(int argc, char **argv)
{
    // Initialize node and node handle
    ros::init(argc, argv, "odom_tf");
    ros::NodeHandle nh;

    // Create static transform from world to odom
    static tf2_ros::StaticTransformBroadcaster static_tf;
   
    geometry_msgs::TransformStamped tf_msg;
    
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.header.frame_id = "world";
    tf_msg.child_frame_id = "odom";
    tf_msg.transform.translation.x = 0;
    tf_msg.transform.translation.y = 0;
    tf_msg.transform.translation.z = 0;
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, 0.0);
    tf_msg.transform.rotation.x = quat.x();
    tf_msg.transform.rotation.y = quat.y();
    tf_msg.transform.rotation.z = quat.z();
    tf_msg.transform.rotation.w = quat.w();
    static_tf.sendTransform(tf_msg); 
    
    // Set the initial relation between odom and base_link
    
    static tf2_ros::TransformBroadcaster odom_to_base;
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_link";    

    // Broadcast the transform from odom to base_link
    odom_to_base.sendTransform(tf_msg); 

    // Create ROS subscriber for WOMBAT odometry pose
    ros::Subscriber pose_sub = nh.subscribe("/WOMBAT/navegation/pose", 10, pose_callback);

    ros::spin();    

    return 0;
}
