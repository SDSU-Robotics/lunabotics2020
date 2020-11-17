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

add_executable(ExcvDriveBaseHW ${PROJECT_SOURCE_DIR}/src/ExcvDriveBaseHW.cpp)
add_executable(ExcvController ${PROJECT_SOURCE_DIR}/src/ExcvController.cpp)
add_executable(DriveController ${PROJECT_SOURCE_DIR}/src/DriveController.cpp)
add_executable(ExcvTrencherHW ${PROJECT_SOURCE_DIR}/src/ExcvTrencherHW.cpp)
add_executable(ExcvConveyorHW ${PROJECT_SOURCE_DIR}/src/ExcvConveyorHW.cpp)
add_executable(TPortConveyorHW ${PROJECT_SOURCE_DIR}/src/TPortConveyorHW.cpp)
add_executable(TPortJoy ${PROJECT_SOURCE_DIR}/src/TPortJoy.cpp)

target_link_libraries(ExcvDriveBaseHW ${catkin_LIBRARIES} Threads::Threads)
target_link_libraries(ExcvController ${catkin_LIBRARIES} Threads::Threads)
target_link_libraries(DriveController ${catkin_LIBRARIES} Threads::Threads)
target_link_libraries(ExcvTrencherHW ${catkin_LIBRARIES} Threads::Threads)
target_link_libraries(ExcvConveyorHW ${catkin_LIBRARIES} Threads::Threads)
target_link_libraries(TPortConveyorHW ${catkin_LIBRARIES} Threads::Threads)
target_link_libraries(TPortJoy ${catkin_LIBRARIES} Threads::Threads)

# Phoenix includes
if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1 / Pi

	target_link_libraries(ExcvConveyorHW ${PROJECT_SOURCE_DIR}/lib/raspberry/libCTRE_Phoenix.a)
	target_link_libraries(ExcvConveyorHW ${PROJECT_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCCI.a)
	target_link_libraries(ExcvConveyorHW ${PROJECT_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixPlatform_socketcan.a)
	target_link_libraries(ExcvConveyorHW ${PROJECT_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCanutils.a)
	target_link_libraries(ExcvConveyorHW ${PROJECT_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCore.a)
	target_link_libraries(ExcvConveyorHW ${PROJECT_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixDiagnostics.a)
	
	target_link_libraries(ExcvDriveBaseHW ${PROJECT_SOURCE_DIR}/lib/raspberry/libCTRE_Phoenix.a)
	target_link_libraries(ExcvDriveBaseHW ${PROJECT_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCCI.a)
	target_link_libraries(ExcvDriveBaseHW ${PROJECT_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixPlatform_socketcan.a)
	target_link_libraries(ExcvDriveBaseHW ${PROJECT_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCanutils.a)
	target_link_libraries(ExcvDriveBaseHW ${PROJECT_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCore.a)
	target_link_libraries(ExcvDriveBaseHW ${PROJECT_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixDiagnostics.a)

	target_link_libraries(ExcvTrencherHW ${PROJECT_SOURCE_DIR}/lib/raspberry/libCTRE_Phoenix.a)
	target_link_libraries(ExcvTrencherHW ${PROJECT_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCCI.a)
	target_link_libraries(ExcvTrencherHW ${PROJECT_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixPlatform_socketcan.a)
	target_link_libraries(ExcvTrencherHW ${PROJECT_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCanutils.a)
	target_link_libraries(ExcvTrencherHW ${PROJECT_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCore.a)
	target_link_libraries(ExcvTrencherHW ${PROJECT_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixDiagnostics.a)

	target_link_libraries(TPortConveyorHW ${PROJECT_SOURCE_DIR}/lib/raspberry/libCTRE_Phoenix.a)
	target_link_libraries(TPortConveyorHW ${PROJECT_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCCI.a)
	target_link_libraries(TPortConveyorHW ${PROJECT_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixPlatform_socketcan.a)
	target_link_libraries(TPortConveyorHW ${PROJECT_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCanutils.a)
	target_link_libraries(ExcvTrencherHW ${PROJECT_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCore.a)
	target_link_libraries(ExcvTrencherHW ${PROJECT_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixDiagnostics.a)

else()

	target_link_libraries(ExcvConveyorHW ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_Phoenix.a)
	target_link_libraries(ExcvConveyorHW ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCCI.a)
	target_link_libraries(ExcvConveyorHW ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixPlatform_socketcan.a)
	target_link_libraries(ExcvConveyorHW ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCanutils.a)
	target_link_libraries(ExcvConveyorHW ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCore.a)
	target_link_libraries(ExcvConveyorHW ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixDiagnostics.a)
	
	target_link_libraries(ExcvDriveBaseHW ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_Phoenix.a)
	target_link_libraries(ExcvDriveBaseHW ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCCI.a)
	target_link_libraries(ExcvDriveBaseHW ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixPlatform_socketcan.a)
	target_link_libraries(ExcvDriveBaseHW ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCanutils.a)
	target_link_libraries(ExcvDriveBaseHW ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCore.a)
	target_link_libraries(ExcvDriveBaseHW ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixDiagnostics.a)

	target_link_libraries(ExcvTrencherHW ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_Phoenix.a)
	target_link_libraries(ExcvTrencherHW ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCCI.a)
	target_link_libraries(ExcvTrencherHW ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixPlatform_socketcan.a)
	target_link_libraries(ExcvTrencherHW ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCanutils.a)
	target_link_libraries(ExcvTrencherHW ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCore.a)
	target_link_libraries(ExcvTrencherHW ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixDiagnostics.a)

	target_link_libraries(TPortConveyorHW ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_Phoenix.a)
	target_link_libraries(TPortConveyorHW ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCCI.a)
	target_link_libraries(TPortConveyorHW ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixPlatform_socketcan.a)
	target_link_libraries(TPortConveyorHW ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCanutils.a)
	target_link_libraries(TPortConveyorHW ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCore.a)
	target_link_libraries(TPortConveyorHW ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixDiagnostics.a)

	target_link_libraries(TPortJoy ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_Phoenix.a)
	target_link_libraries(TPortJoy ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCCI.a)
	target_link_libraries(TPortJoy ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixPlatform_socketcan.a)
	target_link_libraries(TPortJoy ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCanutils.a)
	target_link_libraries(TPortJoy ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCore.a)
	target_link_libraries(TPortJoy ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixDiagnostics.a)

endif()
