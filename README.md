# lunabotics2020

## Prerequisites

 1. Install necessary tools and utilities
     - `sudo apt install git`
 2. Install ROS Melodic
     - http://wiki.ros.org/melodic/Installation/Ubuntu
     - Perform full desktop installation
 3. Install ROS packages
     - `sudo apt install ros-melodic-joy`
     - `sudo apt install ros-melodic-laser-geometry`
     - `sudo apt install ros-melodic-rplidar`
 
## Setup
 1. Setup catkin workspace
     - `mkdir -p ~/catkin_ws/src`
     - `cd ~/catkin_ws/src`
     - `echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`
     - `source ~/.bashrc`
 2. Clone latest rplidar package from github `git clone https://github.com/Slamtec/rplidar_ros.git` .
 3. Clone repo into user directory `git clone https://github.com/SDSU-Robotics/lunabotics2020.git`.
 4. Navigate into repo `cd lunabotics2020`.

## Building
 1. `cd ~/catkin_ws`
 2. `catkin_make`
