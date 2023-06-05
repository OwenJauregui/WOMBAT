#ifndef NAVEGATION_ROS_UTILS_H
#define NAVEGATION_ROS_UTILS_H

#include <ros/ros.h>

namespace ros_utils {

    double get_dt(ros::Time& start, ros::Time& end);
    double dt_and_swp(ros::Time& start, ros::Time end);
        
}

#endif
