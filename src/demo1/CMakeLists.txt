cmake_minimum_required(VERSION 2.8.3)
project(demo1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  kobuki_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/lineColor.cfg
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
