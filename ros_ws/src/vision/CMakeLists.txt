cmake_minimum_required(VERSION 3.0.2)
project(vision)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  image_transport
  std_msgs
  geometry_msgs
  custom_msgs
)

#Catkin configuration
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES face_det
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

#Headers
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

#Python libs
catkin_python_setup()

#Nodos de python
catkin_install_python(PROGRAMS src/point_cloud.py src/laser_map.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
