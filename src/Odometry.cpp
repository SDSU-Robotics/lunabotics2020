#include <string>
#include <unistd.h>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Float32.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/PointStamped.h"
#include "laser_geometry/laser_geometry.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>


int main (int argc, char **argv)
{
	ros::init(argc, argv, "Odometry");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	double x = 0;
  	double y = 0;
  	double th = 0.0;

  	double vx = 0;
  	double vy = 0;
  	double vth = 0;

	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

	while (ros::ok())
	{
		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(vth);

		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = ros::Time::now();
		odom.header.frame_id = "transportation"; //Center frame of odometry message

		//set the position
		odom.pose.pose.position.x = 0.0;
		odom.pose.pose.position.y = 0.0;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
		odom.child_frame_id = "map";
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;

		//publish the message
		odom_pub.publish(odom);

		vth += .01;

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
