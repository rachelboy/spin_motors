cmake_minimum_required(VERSION 2.8.3)
project(spin_motors)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    rospy 
    std_msgs 
    genmsg
    message_generation)

## Declare ROS messages and services
add_message_files(
  FILES
  Encoder.msg
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(roboteq_device src/RoboteqDevice.cpp)

add_executable(spin_motors src/spin_motors.cpp)
target_link_libraries(spin_motors roboteq_device ${catkin_LIBRARIES})
add_dependencies(spin_motors ${catkin_EXPORTED_TARGETS})

add_executable(velocities_to_controller src/velocities_to_controller.cpp)
target_link_libraries(velocities_to_controller roboteq_device ${catkin_LIBRARIES})
add_dependencies(velocities_to_controller ${catkin_EXPORTED_TARGETS})

add_executable(adjustable_PID_controller src/adjustable_PID_controller.cpp)
target_link_libraries(adjustable_PID_controller roboteq_device ${catkin_LIBRARIES})
add_dependencies(adjustable_PID_controller ${catkin_EXPORTED_TARGETS})