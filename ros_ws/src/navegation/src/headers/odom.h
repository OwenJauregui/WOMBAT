#ifndef NAVEGATION_ODOM_NODE
#define NAVEGATION_ODOM_NODE

// Include custom libaries
#include "navegation/utils.h"
#include "navegation/ros_utils.h"

// Include for shutdown
#include <signal.h>

// Include msgs
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose2D.h"

namespace odom {

    // Variables for Odometry usage
    static Eigen::Matrix<double, 2, 1> u;

}

void odom_shutdown(int sig);

void left_wheel_callback(const std_msgs::Float32::ConstPtr& vel);

void right_wheel_callback(const std_msgs::Float32::ConstPtr& vel);

int main(int argc, char** argv);

#endif
