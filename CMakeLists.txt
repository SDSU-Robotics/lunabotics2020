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
  geometry_msgs
  tf2
  tf2_ros
)

catkin_package()

include_directories (src include ${catkin_INCLUDE_DIRS})

add_executable(Localization ${PROJECT_SOURCE_DIR}/src/Localization.cpp)
add_executable(DriveController ${PROJECT_SOURCE_DIR}/src/DriveController.cpp)
add_executable(Perception ${PROJECT_SOURCE_DIR}/src/Perception.cpp)
add_executable(PerceptionTF ${PROJECT_SOURCE_DIR}/src/PerceptionTF.cpp)

target_link_libraries(Localization ${catkin_LIBRARIES})
target_link_libraries(DriveController ${catkin_LIBRARIES})
target_link_libraries(Perception ${catkin_LIBRARIES})
target_link_libraries(PerceptionTF ${catkin_LIBRARIES})