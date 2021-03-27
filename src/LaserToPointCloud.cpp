#include <unistd.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "laser_geometry/laser_geometry.h"

laser_geometry::LaserProjection projector_;

class Listener
{
public:
	void scanCB(const sensor_msgs::LaserScan::ConstPtr& scan_in);
    sensor_msgs::PointCloud2 cloud;

private:
	laser_geometry::LaserProjection projector_;
};

void Listener::scanCB (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	projector_.projectLaser(*scan_in, cloud);
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "LaserToPointCloud");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	//Creating instance of class Listener named "listener"
	Listener listener;

	//Subcribing to filtered lidar data "perception_scan_filtered"
	ros::Subscriber scan_sub = n.subscribe("perception_scan", 100, &Listener::scanCB, &listener);

	//Publishing Pointcloud2
	ros::Publisher perception_cloud = n.advertise<sensor_msgs::PointCloud2>("perception_cloud", 1000);

	while (ros::ok())
	{
		perception_cloud.publish(listener.cloud);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}