#include <string>
#include <unistd.h>
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <sensor_msgs/Joy.h>

using namespace std;

class Listener
{
public:
	void joyListener(const sensor_msgs::Joy::ConstPtr& Joy);
	void getJoyVals(bool buttons[], double axes[]) const;
	void toggle(const bool keys[], bool &t, std_msgs::Float32 &message);


private:
    bool _buttons[12] = { 0 };
	double _axes[6] = { 0 };
};


void Listener::joyListener(const sensor_msgs::Joy::ConstPtr& Joy)
{
	for (int i = 0 ; i < 12; i++)
		_buttons[i] = Joy->buttons[i];

    for (int i = 0; i < 6; i++)
        _axes[i] = Joy->axes[i];
}

void Listener::getJoyVals(bool buttons[], double axes[]) const
{
    for (int i = 0; i < 12; i++)
        buttons[i] = _buttons[i];

    for (int i = 0; i < 6; i++)
        axes[i] = _axes[i];
}

void Listener::toggle(const bool keys[], bool &t, std_msgs::Float32 &message)
{
	// get button state
	if (keys[0] && !t)
	{
		message.data = 1;
		// publish button state
		ROS_INFO("A Button on");
		//toggle button off
		t = !t;
	}
	else if (!keys[0] && t)
	{
		message.data = 0;
		//publish button state
		ROS_INFO("A Button Off");
		//toggle button on
		t = !t;
	}
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "DriveController");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	Listener listener;

	ros::Subscriber joySub = n.subscribe("joy", 100, &Listener::joyListener, &listener);
	
	bool buttons[12];
	double axes[6];
	bool t = false;


	ros::Publisher l_speed_pub = n.advertise<std_msgs::Float32>("ExcvLDrvPwr", 100);
    ros::Publisher r_speed_pub = n.advertise<std_msgs::Float32>("ExcvRDrvPwr", 100);
	ros::Publisher conveyor_pwr_pub = n.advertise<std_msgs::Float32>("ExcvConveyorDrvPwr", 100);

    std_msgs::Float32 l_speed_msg;
    std_msgs::Float32 r_speed_msg;
	std_msgs::Float32 conveyor_pwr_msg;
	
	while (ros::ok())
	{
        listener.getJoyVals(buttons, axes);
		listener.toggle(buttons, t, conveyor_pwr_msg);

		l_speed_msg.data = axes[1]; // left Y
		r_speed_msg.data = axes[3]; // right Y
		
		l_speed_pub.publish(l_speed_msg);
		r_speed_pub.publish(r_speed_msg);
		conveyor_pwr_pub.publish(conveyor_pwr_msg);
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}


