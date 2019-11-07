#include <string>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/LaserScan.h"

using namespace std;

class Listener
{
public:
	void scanCB(const sensor_msgs::LaserScan msg);

private:
	
};

void Listener::scanCB(const sensor_msgs::LaserScan msg)
{
	cout << msg.angle_min << endl;
	cout << "Msg received" << endl;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "Localization");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	Listener listener;

	ros::Subscriber scan_sub = n.subscribe("scan", 100, &Listener::scanCB, &listener);
	//ros::Subscriber rpm_sub = n.subscribe("calculated_rpm", 100, &Listener::rpmListener, &listener);

	ros::Publisher x_pub = n.advertise<std_msgs::Float64>("x", 1000);
	//ros::Publisher rpm_pub = n.advertise<std_msgs::Float64>("set_RPM", 1000);

    std_msgs::Float64 x_msg;

	x_msg.data = 5.0;

	while (ros::ok())
	{
        x_pub.publish(x_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
