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
	tf
	nav_msgs
)

catkin_package()

include_directories (src include ${catkin_INCLUDE_DIRS})

#add_library(global_planner_lib ${PROJECT_SOURCE_DIR}/src/GlobalPlanner.cpp)

add_executable(FlagLocalization ${PROJECT_SOURCE_DIR}/src/FlagLocalization.cpp)
add_executable(DriveController ${PROJECT_SOURCE_DIR}/src/DriveController.cpp)
add_executable(PerceptionTF ${PROJECT_SOURCE_DIR}/src/PerceptionTF.cpp)
add_executable(Odometry ${PROJECT_SOURCE_DIR}/src/Odometry.cpp)
add_executable(BaseController ${PROJECT_SOURCE_DIR}/src/BaseController.cpp)
add_executable(DugHW ${PROJECT_SOURCE_DIR}/src/DugHW.cpp)

target_link_libraries(FlagLocalization ${catkin_LIBRARIES})
target_link_libraries(DriveController ${catkin_LIBRARIES})
target_link_libraries(PerceptionTF ${catkin_LIBRARIES})
target_link_libraries(Odometry ${catkin_LIBRARIES})
target_link_libraries(BaseController ${catkin_LIBRARIES})
target_link_libraries(DugHW ${catkin_LIBRARIES})
