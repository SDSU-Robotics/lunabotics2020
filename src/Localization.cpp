#include <string>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/Float64.h"

using namespace std;

int main (int argc, char **argv)
{
    ros::init(argc, argv, "Localization");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	//Listener listener;

	//ros::Subscriber joySub = n.subscribe("joy", 100, &Listener::joyListener, &listener);
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
