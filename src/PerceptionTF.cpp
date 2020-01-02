#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32.h>
#include <math.h>

const float leverLength = 0.0572; // meters

void angle_cb(const std_msgs::Float32 msg){
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;

	float pitch =  msg.data * -0.0174533; // radians

	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "map";
	transformStamped.child_frame_id = "front_lidar_frame";
	transformStamped.transform.translation.x = 0.0;
	transformStamped.transform.translation.y = leverLength * cos(pitch);
	transformStamped.transform.translation.z = .1575 + leverLength * sin(pitch);

	tf2::Quaternion q;
	q.setRPY(0.0, pitch, 0.0);

	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "PerceptionTF");
	
	ros::NodeHandle nh;
	ros::Subscriber lidar_angle_sub = nh.subscribe("lidar_angle", 10, &angle_cb);

	ros::spin();
	return 0;
};
