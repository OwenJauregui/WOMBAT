cmake_minimum_required(VERSION 3.0.2)
project(navegation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  nav_msgs
  custom_msgs
)

## Find and include Eigen library
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

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
#  LIBRARIES navegation
#  CATKIN_DEPENDS geometry_msgs roscpp std_msgs navegation_ros_utils
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


install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

###############################################################
## Add custom libraries to the package 

set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(LIBRARY_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/include)

include_directories(
    ${LIBRARY_HEADER}
)

# Navegation utils
add_library(navegation_utils SHARED
    ${LIBRARY_DIR}/utils.cpp)
target_link_libraries(navegation_utils
    Eigen3::Eigen
)

# Navegation ROS utils
add_library(navegation_ros_utils SHARED
    ${LIBRARY_DIR}/ros_utils.cpp)
target_link_libraries(navegation_ros_utils
    ${catkin_LIBRARIES}    
)

###############################################################
## Add executables

add_executable(kf
    src/kf.cpp
)
    
target_link_libraries(kf
    Eigen3::Eigen
    ${catkin_LIBRARIES}
    navegation_utils
    navegation_ros_utils
)

add_executable(odom
    src/odom.cpp
)

target_link_libraries(odom
    Eigen3::Eigen
    ${catkin_LIBRARIES}
    navegation_utils
    navegation_ros_utils
)

add_executable(control
    src/control.cpp
)

target_link_libraries(control
    Eigen3::Eigen
    ${catkin_LIBRARIES}
    navegation_utils
    navegation_ros_utils
)

