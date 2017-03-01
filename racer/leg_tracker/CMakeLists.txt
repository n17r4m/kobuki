cmake_minimum_required(VERSION 2.8.3)
project(leg_tracker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  interactive_markers
  rosbag
  image_geometry
)


catkin_package(
)


include_directories(
	INCLUDE include
	${catkin_INCLUDE_DIRS}
)



add_executable(
	detect_leg_clusters 
	src/detect_leg_clusters.cpp
	src/laser_processor.cpp
	src/calc_leg_features.cpp
)
target_link_libraries(
	detect_leg_clusters 
	${catkin_LIBRARIES}
)


install(
  PROGRAMS scripts/multi_person_kalman_tracker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



install(
  PROGRAMS scripts/single_person_tracker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)








