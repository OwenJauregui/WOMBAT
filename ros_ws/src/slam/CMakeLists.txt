cmake_minimum_required(VERSION 3.0.2)
project(slam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES slam
#  CATKIN_DEPENDS cv_bridge geometry_msgs roscpp rospy sensor_msgs std_msgs visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

###############################################################
## Add custom libraries to the package

## Find and include Eigen library
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(LIBRARY_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/include)

include_directories(
    ${LIBRARY_HEADER}
)

# Utils library
add_library(slam_utils SHARED
    ${LIBRARY_DIR}/utils.cpp)
target_link_libraries(slam_utils
    Eigen3::Eigen
)

# KD-Tree library
add_library(kd_tree SHARED
    ${LIBRARY_DIR}/kd_tree.cpp)
target_link_libraries(kd_tree
    Eigen3::Eigen
    slam_utils
)


# ICP library
add_library(slam_icp SHARED
    ${LIBRARY_DIR}/icp.cpp)
target_link_libraries(slam_icp
    Eigen3::Eigen
    slam_utils
    kd_tree
)

###############################################################
## Add executables

add_executable(slam
    src/slam.cpp
)
    
target_link_libraries(slam
    ${catkin_LIBRARIES}
    Eigen3::Eigen
    slam_icp
    kd_tree
    slam_utils
)

add_executable(test_kd
    src/test.cpp
)

target_link_libraries(test_kd
    Eigen3::Eigen
    kd_tree
    slam_utils
)

