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
)

catkin_package()

include_directories (src include ${catkin_INCLUDE_DIRS})

add_executable(HardwareInterface ${PROJECT_SOURCE_DIR}/src/HardwareInterface.cpp)
add_executable(DiffDriveController ${PROJECT_SOURCE_DIR}/src/DiffDriveController.cpp)
add_executable(DriveController ${PROJECT_SOURCE_DIR}/src/DriveController.cpp)


target_link_libraries(HardwareInterface ${catkin_LIBRARIES} Threads::Threads)
target_link_libraries(DiffDriveController ${catkin_LIBRARIES} Threads::Threads)
target_link_libraries(DriveController ${catkin_LIBRARIES} Threads::Threads)

# Phoenix includes
if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1 / Pi

	target_link_libraries(HardwareInterface /home/$ENV{USERNAME}/dev/catkin_ws/src/lunabotics2020/lib/raspberry/libCTRE_Phoenix.a)
	target_link_libraries(HardwareInterface /home/$ENV{USERNAME}/dev/catkin_ws/src/lunabotics2020/lib/raspberry/libCTRE_PhoenixCanutils.a)
	target_link_libraries(HardwareInterface /home/$ENV{USERNAME}/dev/catkin_ws/src/lunabotics2020/lib/raspberry/libCTRE_PhoenixCCI.a)
	target_link_libraries(HardwareInterface /home/$ENV{USERNAME}/dev/catkin_ws/src/lunabotics2020/lib/raspberry/libCTRE_PhoenixPlatformLinuxSocketCAN.a)


else()

	target_link_libraries(HardwareInterface /home/$ENV{USERNAME}/dev/catkin_ws/src/lunabotics2020/lib/x86-64/libCTRE_Phoenix.a)
	target_link_libraries(HardwareInterface /home/$ENV{USERNAME}/dev/catkin_ws/src/lunabotics2020/lib/x86-64/libCTRE_PhoenixCanutils.a)
	target_link_libraries(HardwareInterface /home/$ENV{USERNAME}/dev/catkin_ws/src/lunabotics2020/lib/x86-64/libCTRE_PhoenixCCI.a)
	target_link_libraries(HardwareInterface /home/$ENV{USERNAME}/dev/catkin_ws/src/lunabotics2020/lib/x86-64/libCTRE_PhoenixPlatformLinuxSocketCAN.a)
endif()
