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
#include <tf/transform_broadcaster.h>

#define PI 3.14159265

using namespace std;

class Listener
{
public:
	void scanCB(const sensor_msgs::LaserScan::ConstPtr& scan_in);
	geometry_msgs::PointStamped getPoint1() { return _avgPt_1; }			//Average points found by adding all x and y values for each flag then dividing
	geometry_msgs::PointStamped getPoint2() { return _avgPt_2; }			//by the # of points.  These happen to be geometric centroids of semicircles seen
																		//by the lidar.

private:
	laser_geometry::LaserProjection projector_;
	geometry_msgs::PointStamped _avgPt_1;
	geometry_msgs::PointStamped _avgPt_2;

};

float pnt1x, pnt2x, pnt1y, pnt2y;

void Listener::scanCB (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	sensor_msgs::PointCloud cloud;
	projector_.projectLaser(*scan_in, cloud);

	//Radius of flags in meters
	float rad1 = .1;
	float rad2 = .04;
	//Value to determine tolerance between the two flags, 
	//should be a little less than half of the distance between flags,
	//but greater than the diameter of large flag
	float gap = .05;

	//Variables used by code
		int n = cloud.points.size();// Number of points total
		
		float xsum1 = 0;			//Used to find an average point for the first cluster of points(flag 1)
		float ysum1 = 0;

		float xsum2 = 0;			//Used to find an average point for the first cluster of points(flag 2)
		float ysum2 = 0;

		int pts1 = 0;				//pts1 and 2 are keeping track of the number of points collected for each flag
		int pts2 = 0;				//These are used in calculating the avg points

		float theta1;				//Angles from beacon to average points.  By extending along this line,
		float theta2;				//a center of each flag can be calculated.

		int obj = 1;				//The variable obj is used to distinguish whether a point belongs to the first flag or second

		float yaw;					//This is the yaw of the output frame "flag" and used creating the transform between "beacon_frame" and "flag"


	//Loop through every point
	for (int i = 1; i<n; i++)
	{
		//Check if the current point is part of the same flag as the previous one by seeing how close the current and previous points are
		if(sqrt((cloud.points[i].x - cloud.points[i-1].x)*(cloud.points[i].x - cloud.points[i-1].x) + (cloud.points[i].y - cloud.points[i-1].y)*(cloud.points[i].y - cloud.points[i-1].y)) < (rad1+gap))
		{
			//If object 1, add the last point to the sum for flag 1.  Vice versa for flag 2.
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
			//If points do not share an object, add the last point to the sum of its respective flag,
			//but then switch the obj variable for the next pass of the loop
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

			//Switching objects
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
	
	//Averaging the points
	_avgPt_1.point.x = xsum1 / pts1;  
	_avgPt_1.point.y = ysum1 / pts1;
	_avgPt_1.point.z = 0;

	//Calulating theta1 using trig.
	theta1 = acos(_avgPt_1.point.x / sqrt(_avgPt_1.point.x*_avgPt_1.point.x + _avgPt_1.point.y*_avgPt_1.point.y));

	//This overwrites _avgPt_1 with the true center of the flag by extending along the line formed by theta1 and the line from beacon to avg point
	_avgPt_1.point.x = ((sqrt(_avgPt_1.point.x*_avgPt_1.point.x + _avgPt_1.point.y*_avgPt_1.point.y) + (4*rad1)/(3*3.14159))*cos(theta1)); //4r/3pi is distance of centroid of semicircle from the center
	_avgPt_1.point.y = ((sqrt(_avgPt_1.point.x*_avgPt_1.point.x + _avgPt_1.point.y*_avgPt_1.point.y) + (4*rad1)/(3*3.14159))*sin(theta1));

	_avgPt_1.header.frame_id = "beacon_frame";	//Specifying what frame in header 

	//The process is repeated for flag 2
	_avgPt_2.point.x = xsum2 / pts2;  
	_avgPt_2.point.y = ysum2 / pts2;
	_avgPt_2.point.z = 0;

	theta2 = acos(_avgPt_2.point.x / sqrt(_avgPt_2.point.x*_avgPt_2.point.x + _avgPt_2.point.y*_avgPt_2.point.y));

	_avgPt_2.point.x = ((sqrt(_avgPt_2.point.x*_avgPt_2.point.x + _avgPt_2.point.y*_avgPt_2.point.y) + (4*rad2)/(3*3.14159))*cos(theta2));
	_avgPt_2.point.y = ((sqrt(_avgPt_2.point.x*_avgPt_2.point.x + _avgPt_2.point.y*_avgPt_2.point.y) + (4*rad2)/(3*3.14159))*sin(theta2));

	_avgPt_2.header.frame_id = "beacon_frame";

	//Assigns the new center points to new variables used in the transform
	pnt1x = _avgPt_1.point.x;
	pnt2x = _avgPt_2.point.x;

	pnt1y = _avgPt_1.point.y;
	pnt2y = _avgPt_2.point.y;

	//Calculates yaw of the robot
	yaw =  atan((pnt1y-pnt2y)/(pnt1x-pnt2x)); // radians

	//Creates the transform between "beacon_frame" and "flag"
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;

	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "beacon_frame";
	transformStamped.child_frame_id = "flag";

	//Uses the center of flag 1 as the xyz coordinates for frame "flag"
	transformStamped.transform.translation.x = pnt1x;
	transformStamped.transform.translation.y = pnt1y;
	transformStamped.transform.translation.z = 0.0;

	//Generates a quaternion value for the orientation/rotation of "flag" based on RPY values (0, 0, yaw)
	tf2::Quaternion q;
	q.setRPY(0, 0, yaw);

	//Setting the quaternion orientation of frame "flag"
	transformStamped.transform.rotation = q;

	br.sendTransform(transformStamped);
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "FlagLocalization");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	//Creating instance of class Listener named "listener"
	Listener listener;

	//Subcribing to filtered lidar data "beacon_scan_filtered"
	ros::Subscriber scan_sub = n.subscribe("beacon_scan_filtered", 100, &Listener::scanCB, &listener);

	//Publishing 2 points that are centers of flags
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
