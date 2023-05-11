cmake_minimum_required(VERSION 3.0.2)
project(custom_msgs)

## Compile as c++11, supported in ROS Kinetic and newer 
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
)

#Messages to compile
add_message_files(
    FILES
    vel_cmd.msg
)

#Messages dependencies
generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
# INCLUDE_DIRS include
# LIBRARIES oswaldo
# CATKIN_DEPENDS roscpp rospy std_msgs
# DEPENDS system_lib
)
