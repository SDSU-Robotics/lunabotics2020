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

add_executable(FlagLocalization ${PROJECT_SOURCE_DIR}/src/FlagLocalization.cpp)
add_executable(DriveController ${PROJECT_SOURCE_DIR}/src/DriveController.cpp)
add_executable(SLS ${PROJECT_SOURCE_DIR}/src/SLS.cpp)
add_executable(Odometry ${PROJECT_SOURCE_DIR}/src/Odometry.cpp)
#add_executable(GlobalPlanner ${PROJECT_SOURCE_DIR}/src/GlobalPlanner.cpp)

target_link_libraries(FlagLocalization ${catkin_LIBRARIES})
target_link_libraries(DriveController ${catkin_LIBRARIES})
target_link_libraries(SLS ${catkin_LIBRARIES} Threads::Threads)
target_link_libraries(Odometry ${catkin_LIBRARIES})
#target_link_libraries(GlobalPlanner ${catkin_LIBRARIES})

# Phoenix includes
if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1 / Pi

	target_link_libraries(SLS /home/$ENV{USERNAME}/catkin_ws/src/lunabotics2020/lib/raspberry/libCTRE_Phoenix.a)
	target_link_libraries(SLS /home/$ENV{USERNAME}/catkin_ws/src/lunabotics2020/lib/raspberry/libCTRE_PhoenixCCI.a)
	target_link_libraries(SLS /home/$ENV{USERNAME}/catkin_ws/src/lunabotics2020/lib/raspberry/libCTRE_PhoenixPlatformLinuxSocketCan.a)
	target_link_libraries(SLS /home/$ENV{USERNAME}/catkin_ws/src/lunabotics2020/lib/raspberry/libCTRE_PhoenixCanutils.a)

else()

	target_link_libraries(SLS /home/$ENV{USERNAME}/catkin_ws/src/lunabotics2020/lib/x86-64/libCTRE_Phoenix.a)
	target_link_libraries(SLS /home/$ENV{USERNAME}/catkin_ws/src/lunabotics2020/lib/x86-64/libCTRE_PhoenixCCI.a)
	target_link_libraries(SLS /home/$ENV{USERNAME}/catkin_ws/src/lunabotics2020/lib/x86-64/libCTRE_PhoenixPlatformLinuxSocketCan.a)
	target_link_libraries(SLS /home/$ENV{USERNAME}/catkin_ws/src/lunabotics2020/lib/x86-64/libCTRE_PhoenixCanutils.a)

endif()
