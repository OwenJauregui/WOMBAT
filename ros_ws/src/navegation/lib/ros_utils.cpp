#include "navegation/ros_utils.h"

double ros_utils::get_dt(ros::Time& start, ros::Time& end)
{   
    // Compute time difference
    ros::Duration diff = end - start;
    
    // Convert time into seconds
    double dt = diff.toSec();

    // Check if dt is a valid difference
//    if(dt < 0) {
  //      dt += 1;
    //}

    return dt;
}

double ros_utils::dt_and_swp(ros::Time& start, ros::Time end)
{
    // Calculate dt
    double dt = ros_utils::get_dt(start, end);
    
    // Swap end time into start time reference
    start = end;

    return dt;
}
