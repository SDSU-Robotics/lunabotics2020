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

#define PI 3.14159265

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
	// Values just used to calculate the difference in x and y for different cases
	float xdiff, ydiff, side1x, side2x, side1y, side2y;
	// Values for side length of 2 side cases
	float side1, side2;
	// Used for keeping track of the max and min x and y coordinates
	float maxX = -10.0;
	float minX = 10;
	float maxY = -10;
	float minY = 10;
	double angle;
	// Used for keeping track of which point is max and min
	int minXloc, maxXloc, minYloc, maxYloc, sides;

	const int side_long = 0.142875;
	const int side_mid = 0.1143;
	const int side_short = 0.085725;

	int sideTol = (side_long - side_mid) / 2;


	
	
	//std::cout << n << std::endl; //cloud.points.size() is number of points scanned after filters
	
	
	for (int i = 0; i < n; i++)
	{
		// Summing all points on x and y axes
		xsum += cloud.points[i].x;
		ysum += cloud.points[i].y;

		// Assigns max and min values and 
		if(cloud.points[i].x > maxX)
		{
			maxX = cloud.points[i].x;
			maxXloc = i;
		}
		if(cloud.points[i].x < minX)
		{
			minX = cloud.points[i].x;
			minXloc = i;
		}
		if(cloud.points[i].y > maxY)
		{	maxY = cloud.points[i].y;
			maxYloc = i;}
		if(cloud.points[i].y < minY)
		{	minY = cloud.points[i].y;
			minYloc = i;}
	}
	
	if(((minXloc == maxYloc) && (maxXloc == minYloc)) || ((maxXloc == maxYloc) && (minXloc == minYloc)))
	{
		sides = 1;

		xdiff = maxX - minX;
		ydiff = maxY - minY;
		

		std::cout << "\n1 side \n\n";
		std::cout << sqrt((xdiff*xdiff) + (ydiff*ydiff)) * 39.3701 << std::endl;
		side1 = sqrt((xdiff*xdiff) + (ydiff*ydiff));
	}
	else
	{
		// Case 1
		if(minXloc == minYloc)
		{
			side1x = cloud.points[maxYloc].x - maxX;
			side1y = maxY - cloud.points[maxXloc].y;
			side1 = sqrt((side1x*side1x) + (side1y*side1y)) * 39.3701;
			side2x = maxX - minX;
			side2y = cloud.points[maxXloc].y - minY;
			side2 = sqrt((side2x*side2x) + (side2y*side2y)) * 39.3701;
			
			// Eliminates false two sides
			if(side1 < 1)
			{
				std::cout << "\n1 side\n\n";
				std::cout << side2 << std::endl;
				side1 = side2;
				sides = 1;
			}
			else if(side2 < 1)
			{
				std::cout << "\n1 side\n\n";
				std::cout << side1 << std::endl;
				sides = 1;
			}
			else
			{
				std::cout << "\n2 sides (1)\n\n";
				std::cout << side1;
				std::cout << " and ";
				std::cout << side2 << std::endl;
				sides = 2;
			}
		}
		// Case 2
		else if(minXloc == maxYloc)
		{
			side1x = minX - maxX;
			side1y = maxY - cloud.points[maxXloc].y;
			side1 = sqrt((side1x*side1x) + (side1y*side1y)) * 39.3701;
			side2x = maxX - cloud.points[minYloc].x;
			side2y = cloud.points[maxXloc].y - minY;
			side2 = sqrt((side2x*side2x) + (side2y*side2y)) * 39.3701;

			// Eliminates false two sides
			if(side1 < 1)
			{
				std::cout << "\n1 side\n\n";
				std::cout << side2 << std::endl;
				side1 = side2;
				sides = 1;
			}
			else if(side2 < 1)
			{
				std::cout << "\n1 side\n\n";
				std::cout << side1 << std::endl;
				sides = 1;
			}
			else
			{
				std::cout << "\n2 sides (1)\n\n";
				std::cout << side1;
				std::cout << " and ";
				std::cout << side2 << std::endl;
				sides = 2;
			}
			
			
			if(side1 <= 6 && side1 > 4.5 && side2 > 2 && side2 <= 4)
			{
//				std::cout << "\n 5 over 3 case \n\n";
			}
			else if (side1 > 2.5 && side1 < 4)
			{
//				std::cout << "\n  over 4 case \n\n";
			}
		}
		// Case 3
		else if(maxXloc == maxYloc)
		{
			side1x = minX - cloud.points[minYloc].x;
			side1y = minY - cloud.points[minXloc].y;
			side1 = sqrt((side1x*side1x) + (side1y*side1y)) * 39.3701;
			side2x = maxX - cloud.points[minYloc].x;
			side2y = maxY - minY;
			side2 = sqrt((side2x*side2x) + (side2y*side2y)) * 39.3701;

			// Eliminates false two sides
			if(side1 < 1)
			{
				std::cout << "\n1 side\n\n";
				std::cout << side2 << std::endl;
				side1 = side2;
				sides = 1;
			}
			else if(side2 < 1)
			{
				std::cout << "\n1 side\n\n";
				std::cout << side1 << std::endl;
				sides = 1;
			}
			else
			{
				std::cout << "\n2 sides (1)\n\n";
				std::cout << side1;
				std::cout << " and ";
				std::cout << side2 << std::endl;
				sides = 2;
			}
			
			
			if(side1 <= 4 && side1 > 2 && side2 > 5 && side2 <= 6)
			{
//				std::cout << "\n 3 left of 5 case \n\n";
			}
		}
		// Case 4
		else
		{
			side1x = minX - cloud.points[minYloc].x;
			side1y = minY - maxY;
			side1 = sqrt((side1x*side1x) + (side1y*side1y)) * 39.3701;
			side2x = maxX - cloud.points[minYloc].x;
			side2y = cloud.points[maxXloc].y - minY;
			side2 = sqrt((side2x*side2x) + (side2y*side2y)) * 39.3701;

			// Eliminates false two sides
			if(side1 < 1)
			{
				std::cout << "\n1 side\n\n";
				std::cout << side2 << std::endl;
				side1 = side2;
				sides = 1;
			}
			else if(side2 < 1)
			{
				std::cout << "\n1 side\n\n";
				std::cout << side1 << std::endl;
				sides = 1;
			}
			else
			{
				std::cout << "\n2 sides (1)\n\n";
				std::cout << side1;
				std::cout << " and ";
				std::cout << side2 << std::endl;
				sides = 2;
			}
			
			
			if(side1 <= 5 && side1 > 4 && side2 > 2 && side2 <= 4)
			{
//				std::cout << "\n 4 left of 3 case \n\n";
			}
			else if(side1 <= 6 && side1 > 5 && side2 > 4 && side2 <= 5)
			{
//				std::cout << "\n 5 left of 4 case \n\n";
			}
		}
		
	}

if (sides == 1)
{
	if (side1 > (side_long - sideTol)) // 5 side case
	{
		if((minXloc == maxYloc) && (maxXloc == minYloc))
		{
			// Infer 3rd point (cloud.points[maxXloc].x, cloud.points[maxYloc].y)
			angle = (2 * PI) - asin((cloud.points[maxXloc].x - cloud.points[minXloc].x) / side1);
		}
		else if((minXloc == minYloc) && (maxXloc == maxYloc))
		{
			// Infer 3rd point (cloud.points[minXloc].x, cloud.points[maxYloc].y)
			angle = asin((cloud.points[maxXloc].x - cloud.points[minXloc].x) / side1);
		}
	}
	else if (side1 > (side_mid - sideTol) && side1 < (side_mid + sideTol)) // 4 side case
	{
		if((minXloc == maxYloc) && (maxXloc == minYloc))
		{
			angle = ((PI / 2) - asin((cloud.points[maxXloc].x - cloud.points[minXloc].x) / side1)) + 0.9272952;
		}
		else if((minXloc == minYloc) && (maxXloc == maxYloc))
		{
			angle = (PI / 2) + asin((cloud.points[maxXloc].x - cloud.points[minXloc].x) / side1) + 0.9272952;
		}
	}
	else // 3 side case
	{
		if((minXloc == maxYloc) && (maxXloc == minYloc))
		{
			angle = (PI - asin((cloud.points[maxXloc].x - cloud.points[minXloc].x) / side1)) + 0.9272952;
		}
		else if((minXloc == minYloc) && (maxXloc == maxYloc))
		{
			angle = (PI + asin((cloud.points[maxXloc].x - cloud.points[minXloc].x) / side1)) + 0.9272952;
		}
	}
	std::cout << (angle / PI) * 180.0 << std::endl;
}
	// Averaging x and y values
	avgPt_.point.x = xsum / n;  
	avgPt_.point.y = ysum / n;
	avgPt_.point.z = 0;

	

	avgPt_.header.frame_id = "beacon_frame";	//Specifying what frame in header 
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "Localization");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	Listener listener;

	ros::Subscriber scan_sub = n.subscribe("beacon_scan_filtered", 100, &Listener::scanCB, &listener);

	ros::Publisher point_pub = n.advertise<geometry_msgs::PointStamped>("point", 1000);

	while (ros::ok())
	{
        point_pub.publish(listener.getPoint());

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
