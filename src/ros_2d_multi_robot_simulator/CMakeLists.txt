cmake_minimum_required(VERSION 3.0.2)
project(ros_2d_multi_robot_simulator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf2
  tf2_ros
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package()

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/src/libs
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
