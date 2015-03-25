cmake_minimum_required(VERSION 2.8.3)
project(spin_motors)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services

## Generate added messages and services
# generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(roboteq_device src/RoboteqDevice.cpp)

add_executable(spin_motors src/spin_motors.cpp)
target_link_libraries(spin_motors roboteq_device ${catkin_LIBRARIES})
add_dependencies(spin_motors ${catkin_EXPORTED_TARGETS})