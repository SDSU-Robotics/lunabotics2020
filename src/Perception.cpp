#include <string>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/PointStamped.h"
#include "laser_geometry/laser_geometry.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>

using namespace std;

class Listener
{
public:
	void scanCB(const sensor_msgs::LaserScan::ConstPtr& scan_in);
	geometry_msgs::PointStamped getPoint() { return avgPt_; }

private:
	laser_geometry::LaserProjection projector_;
	geometry_msgs::PointStamped avgPt_;
};

void Listener::scanCB (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	sensor_msgs::PointCloud cloud;
	projector_.projectLaser(*scan_in, cloud);

	int n = cloud.points.size();  // Number of points total
	
	float xsum = 0;
	float ysum = 0;
	float zsum = 0;

	
	std::cout << n << std::endl; //cloud_.points.size() is number of points scanned after filters
	
	// Summing all points on x and y axes
	for (int i = 0; i <= n; i++)
	{
		xsum += cloud.points[i].x;
		ysum += cloud.points[i].y;
		zsum += cloud.points[i].z;
	}

	// Averaging x and y values
	avgPt_.point.x = xsum / n;
	avgPt_.point.y = ysum / n;
	avgPt_.point.z = zsum / n;
	

	avgPt_.header.frame_id = "front_lidar_frame";	//Specifying what frame in header 
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "Perception");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	Listener listener;

	ros::Subscriber scan_sub = n.subscribe("front_lidar_scan_filtered", 100, &Listener::scanCB, &listener);

	ros::Publisher point_pub = n.advertise<geometry_msgs::PointStamped>("point", 1000);

	while (ros::ok())
	{
        point_pub.publish(listener.getPoint());

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
