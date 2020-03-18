#include <iostream>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <std_msgs/Float32.h>
#include <math.h>

#define PI 3.141592653589

#define SLS_HEIGHT 0.098 // meters
#define LEVER_LENGTH 0.005 // meters

using namespace std;

void rpy_cb(const geometry_msgs::Vector3Stamped rpy_in){
	
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;

	float pitch =  rpy_in.vector.y + PI / 2.0; // radians

	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "map";
	transformStamped.child_frame_id = "front_lidar_frame";
	transformStamped.transform.translation.x = 0.0;
	transformStamped.transform.translation.y = LEVER_LENGTH * cos(pitch);
	transformStamped.transform.translation.z = SLS_HEIGHT + LEVER_LENGTH * sin(pitch);

	tf2::Quaternion q;
	q.setRPY(0.0, -1 * pitch, 0.0);

	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "SLS");
	
	ros::NodeHandle n;

	string topic;
	n.param<std::string>("rpy_topic", topic, "/imu/rpy");

	ros::Subscriber rpy_sub = n.subscribe(topic, 10, &rpy_cb);

	ros::spin();

	return 0;
};
