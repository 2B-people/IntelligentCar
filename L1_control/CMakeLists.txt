cmake_minimum_required(VERSION 2.8.3)
project(hypha_racecar)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  move_base
  tf
  roscpp
  rospy
  std_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS visualization_msgs
#  INCLUDE_DIRS include
#  LIBRARIES hypha_racecar
#  CATKIN_DEPENDS geometry_msgs move_base roscp rospy std_msgs visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  src/pid.h
)


add_executable(L1_controller_v2 src/L1_controller_v2.cpp src/pid.cpp )
target_link_libraries(L1_controller_v2 ${catkin_LIBRARIES})
add_dependencies(L1_controller_v2 hypha_racecar_generate_messages_cpp)
