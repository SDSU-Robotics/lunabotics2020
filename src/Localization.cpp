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

#define PI 3.14159265

using namespace std;

class Listener
{
public:
	void scanCB(const sensor_msgs::LaserScan::ConstPtr& scan_in);
	geometry_msgs::PointStamped getPoint1() { return avgPt_1; }
	geometry_msgs::PointStamped getPoint2() { return avgPt_2; }

private:
	laser_geometry::LaserProjection projector_;
	geometry_msgs::PointStamped avgPt_1;
	geometry_msgs::PointStamped avgPt_2;
};

float pnt1x, pnt2x, pnt1y, pnt2y;

void Listener::scanCB (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	sensor_msgs::PointCloud cloud;
	projector_.projectLaser(*scan_in, cloud);

	int n = cloud.points.size();  // Number of points total
	
	float xsum1 = 0;
	float ysum1 = 0;

	float xsum2 = 0;
	float ysum2 = 0;

	float theta1;
	float theta2;

	int obj = 1;

	int pts1 = 0;
	int pts2 = 0;

	for (int i = 1; i<n; i++)
	{
		if(sqrt((cloud.points[i].x - cloud.points[i-1].x)*(cloud.points[i].x - cloud.points[i-1].x) + (cloud.points[i].y - cloud.points[i-1].y)*(cloud.points[i].y - cloud.points[i-1].y)) < .3048)
		{
			if(obj == 1)
			{
				xsum1 += cloud.points[i-1].x;
				ysum1 += cloud.points[i-1].y;
				pts1 ++;
			}
			else
			{
				xsum2 += cloud.points[i-1].x;
				ysum2 += cloud.points[i-1].y;
				pts2 ++;
			}
		}
		else
		{
			if(obj == 1)
			{
				xsum1 += cloud.points[i-1].x;
				ysum1 += cloud.points[i-1].y;
				pts1 ++;
			}
			else
			{
				xsum2 += cloud.points[i-1].x;
				ysum2 += cloud.points[i-1].y;
				pts2 ++;
			}

			if(obj == 1)
			{
				obj = 2;
			}
			else
			{
				obj = 1;
			}
			
		}
	}
	
	avgPt_1.point.x = xsum1 / pts1;  
	avgPt_1.point.y = ysum1 / pts1;
	avgPt_1.point.z = 0;

	theta1 = acos( avgPt_1.point.x / sqrt(avgPt_1.point.x*avgPt_1.point.x + avgPt_1.point.y*avgPt_1.point.y));

	avgPt_1.point.x = ((sqrt(avgPt_1.point.x*avgPt_1.point.x + avgPt_1.point.y*avgPt_1.point.y) + (4*.18)/(3*3.14159))*cos(theta1));
	avgPt_1.point.y = ((sqrt(avgPt_1.point.x*avgPt_1.point.x + avgPt_1.point.y*avgPt_1.point.y) + (4*.18)/(3*3.14159))*sin(theta1));

	avgPt_1.header.frame_id = "beacon_frame";	//Specifying what frame in header 

	avgPt_2.point.x = xsum2 / pts2;  
	avgPt_2.point.y = ysum2 / pts2;
	avgPt_2.point.z = 0;

	theta2 = acos( avgPt_2.point.x / sqrt(avgPt_2.point.x*avgPt_2.point.x + avgPt_2.point.y*avgPt_2.point.y));

	avgPt_2.point.x = ((sqrt(avgPt_2.point.x*avgPt_2.point.x + avgPt_2.point.y*avgPt_2.point.y) + (4*.0508)/(3*3.14159))*cos(theta2));
	avgPt_2.point.y = ((sqrt(avgPt_2.point.x*avgPt_2.point.x + avgPt_2.point.y*avgPt_2.point.y) + (4*.0508)/(3*3.14159))*sin(theta2));

	pnt1x = avgPt_1.point.x;
	pnt2x = avgPt_2.point.x;

	pnt1y = avgPt_1.point.y;
	pnt2y = avgPt_2.point.y;

	avgPt_2.header.frame_id = "beacon_frame";

	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;

	float yaw =  atan((pnt1y-pnt2y)/(pnt1x-pnt2x)); // radians

	std::cout << yaw << std::endl;

	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "beacon_frame";
	transformStamped.child_frame_id = "robot_frame";
	transformStamped.transform.translation.x = pnt1x;
	transformStamped.transform.translation.y = pnt1y;
	transformStamped.transform.translation.z = 0.0;

	tf2::Quaternion q;
	q.setRPY(0.0, 0.0, yaw + 3.14159/2.0);

	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	br.sendTransform(transformStamped);

}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "Localization");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	Listener listener;

	ros::Subscriber scan_sub = n.subscribe("beacon_scan_filtered", 100, &Listener::scanCB, &listener);

	ros::Publisher point_pub1 = n.advertise<geometry_msgs::PointStamped>("point1", 1000);
	ros::Publisher point_pub2= n.advertise<geometry_msgs::PointStamped>("point2", 1000);


	while (ros::ok())
	{
        point_pub1.publish(listener.getPoint1());
		point_pub2.publish(listener.getPoint2());

		

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
