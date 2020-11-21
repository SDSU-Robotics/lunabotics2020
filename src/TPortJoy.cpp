#include <string>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <sensor_msgs/Joy.h>

using namespace std;

class Listener
{
public:
	void joyListener(const sensor_msgs::Joy::ConstPtr& Joy);
	void getJoyVals(bool buttons[], double axes[]) const;
	void getButtonState(bool buttons[]);
	void toggle(const bool keys, bool &currentButton, bool &on, std_msgs::Float32 &message);

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

int main (int argc, char **argv)
{
    ros::init(argc, argv, "DriveController");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	Listener listener;

	ros::Subscriber joySub = n.subscribe("TPort/joy", 100, &Listener::joyListener, &listener);
	
	bool buttons[12];
	double axes[6];

	bool currentButton = false;
	bool on = false;

	ros::Publisher l_speed_pub = n.advertise<std_msgs::Float32>("TPortLDrvPwr", 100);
    ros::Publisher r_speed_pub = n.advertise<std_msgs::Float32>("TPortRDrvPwr", 100);
	ros::Publisher conveyor_pub = n.advertise<std_msgs::Float32>("ConveyorDrvPwr", 100);
	
    std_msgs::Float32 l_speed_msg;
    std_msgs::Float32 r_speed_msg;
	std_msgs::Float32 conveyor_pwr;
	
	while (ros::ok())
	{
        listener.getJoyVals(buttons, axes);
		// listener.getButtonState(buttons);
		listener.toggle(buttons[0], currentButton, on, conveyor_pwr);


		l_speed_msg.data = axes[1]; // left Y
		r_speed_msg.data = axes[3]; // right Y
		
		l_speed_pub.publish(l_speed_msg);
		r_speed_pub.publish(r_speed_msg);
		conveyor_pub.publish(conveyor_pwr); // conveyor power
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

//bool pressButton(bool button, Float32)

void Listener::toggle(const bool keys, bool &currentButton, bool &on, std_msgs::Float32 &message)
{
	bool lastButton;
	lastButton = currentButton;
	currentButton = keys;

	if (lastButton && !currentButton)
	{
		on = !on;
		ROS_INFO("A button released");
	}
		
	if (on)
	{
		ROS_INFO("A button on");
		message.data = 1;
	}
	else
	{
		ROS_INFO("A button off");
		message.data = 0;
	}
}
