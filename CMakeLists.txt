# Project's name
PROJECT (lunabotics2020)

# Specify the minimum version for CMake
cmake_minimum_required(VERSION 3.1)

add_definitions(-std=c++11)

set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(catkin REQUIRED)
find_package (Threads REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	sensor_msgs
	laser_geometry
	tf2
	tf2_ros
	geometry_msgs
)

catkin_package()

include_directories (src include ${catkin_INCLUDE_DIRS})

add_executable(Localization ${PROJECT_SOURCE_DIR}/src/Localization.cpp)
add_executable(DriveController ${PROJECT_SOURCE_DIR}/src/DriveController.cpp)

target_link_libraries(Localization ${catkin_LIBRARIES})
target_link_libraries(DriveController ${catkin_LIBRARIES})

add_library(global_planner_lib ${PROJECT_SOURCE_DIR}/src/GlobalPlanner.cpp)