#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>

int main(int argc, char **argv);

void pose_callback(const geometry_msgs::PoseStamped& msg);
void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
