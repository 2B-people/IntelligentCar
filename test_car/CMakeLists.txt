cmake_minimum_required(VERSION 2.8.3)
project(test_car)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  joy
  move_base
  tf
  roborts_msgs
  roscpp
  rospy
  std_msgs
  visualization_msgs
  dynamic_reconfigure
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES art_racecar
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
# generate_dynamic_reconfigure_options(cfg/imu.cfg)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(car_controller 
	src/car_controller.cpp 
	src/car_controller.hpp
	src/PID.h
)
target_link_libraries(car_controller 
	${catkin_LIBRARIES}
	${PROJECT_SOURCE_DIR}/src/lib/libart_controller.a
)
# add_dependencies(car_controller art_car_generate_messages_cpp)








