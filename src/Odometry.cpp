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
#include <tf2_ros/transform_listener.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

class Listener
{
    public:
        void getFlagPoints(const geometry_msgs::PointStamped flag); //get points for flag on transport bot
		float flagX, flagY, flagZ;
};

int main (int argc, char **argv)
{
	ros::init(argc, argv, "Odometry");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	Listener listener;
	tf2_ros::Buffer tfBuffer;
 	tf2_ros::TransformListener tfListener(tfBuffer);

	double x = 0;
  	double y = 0;
  	double th = 0.0;

	double lastFlagX =0;
	double lastFlagY = 0;
	double lastFlagZ = 0;

  	double vx = 0;
  	double vy = 0;
  	double vth = 0;

	double roll = 0;
	double pitch = 0;
	double yaw = 0;
	double lastYaw = 0;

	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

	ros::Subscriber flag_point_sub = n.subscribe("point1", 100, &Listener::getFlagPoints, &listener);

	ros::Time current_time, last_time;
  	current_time = ros::Time::now();
  	last_time = ros::Time::now();

	while (ros::ok())
	{
		geometry_msgs::TransformStamped transformStamped;
		 try{
   			transformStamped = tfBuffer.lookupTransform("flag", "beacon_frame", ros::Time(0));
 			}
		 catch (tf2::TransformException &ex) {
  				ROS_WARN("%s",ex.what());
   				ros::Duration(1.0).sleep();
   				continue;
 			}
		tf::Quaternion q;

	 	quaternionMsgToTF(transformStamped.transform.rotation , q);
		tf::Matrix3x3 m(q);
//		tf::Matrix3x3 m(transformStamped.transform.rotation);
    	
    	m.getRPY(roll, pitch, yaw);



		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(vth);

		current_time = ros::Time::now();

		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = ros::Time::now();
		odom.header.frame_id = "map"; //Center frame of odometry message

		//set the position
		odom.pose.pose.position.x = 0.0;
		odom.pose.pose.position.y = 0.0;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		double dt = (current_time - last_time).toSec();

		vx = (listener.flagX - lastFlagX)/dt;
		vy = (listener.flagY - lastFlagY)/dt;
		vth = (yaw - lastYaw)/dt;
		

		//compute odometry in a typical way given the velocities of the robot
    	
    	double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    	double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    	double delta_th = vth * dt;

    	x += delta_x;
    	y += delta_y;
    	th += delta_th;


		//set the velocity
		odom.child_frame_id = "map";
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;

		//publish the message
		odom_pub.publish(odom);

		vth = 1.57;

		lastFlagX = listener.flagX;
		lastFlagY = listener.flagY;
		lastFlagZ = listener.flagZ;
		last_time = current_time;
		lastYaw = yaw;


		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void Listener::getFlagPoints(const geometry_msgs::PointStamped flag){
	flagX = flag.point.x;
	flagY = flag.point.y;
	flagZ = flag.point.z;
}