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
#include <geometry_msgs/Vector3Stamped.h>
#include <math.h>

#define PI 3.14159265

using namespace std;

class Listener
{
public:
	
	void rpyCB(const geometry_msgs::Vector3Stamped rpy_in);

private:
	
};

void Listener::rpyCB (const geometry_msgs::Vector3Stamped rpy_in)
{	
float roll = rpy_in.vector.x*(180.0/PI);
float pitch = rpy_in.vector.y*(180.0/PI);
float yaw = rpy_in.vector.z*(180.0/PI);
	cout<<"Roll: "<<roll<<"  Pitch:"<<pitch<<"  Yaw:"<<yaw<<endl;
}



int main (int argc, char **argv)
{
    ros::init(argc, argv, "Read_IMU");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	Listener listener;

	

	ros::Subscriber rpy_sub = n.subscribe("/imu/rpy",10, &Listener::rpyCB,&listener);

	//ros::Publisher point_pub1 = n.advertise<geometry_msgs::PointStamped>("point1", 1000);
	//ros::Publisher point_pub2= n.advertise<geometry_msgs::PointStamped>("point2", 1000);


	while (ros::ok())
	{
    // point_pub1.publish(listener.getPoint1());
	//	point_pub2.publish(listener.getPoint2());

		

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
