cmake_minimum_required(VERSION 2.8.3)
project(racer)


find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
)

generate_dynamic_reconfigure_options(
  cfg/lineColor.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES racer
#  CATKIN_DEPENDS dynamic_reconfigure roscpp rospy
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

