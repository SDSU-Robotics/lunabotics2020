#include <string>
#include <unistd.h>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include <sensor_msgs/Joy.h>
#include "JoyMap.h"

using namespace std;

/****************************************************************************
****     This node interprets joystick values and publishes values for   ****
****         hardware on the Transportation robot                        ****
****     Subscribers:                                                    ****
****          sensor_msgs/Joy /TPort/joy - tport values from joy node    ****
****     Publishers:                                                     ****
****          std_msgs/Float32 TPortRDrvPwr - tport right motor power    ****
****          std_msgs/Float32 TPortLDrvPwr - tport left motor power     ****
****          std_msgs/Float32 TPortConveyorDrvPwr - conveyor motor power****
****          std_msgs/Int8 TportExtendPwr - extender true/false value   ****
****************************************************************************/

#define DRIVE_SCALE 0.5

class Listener
{
public:
	void joyListener(const sensor_msgs::Joy::ConstPtr& Joy);
	void getJoyVals(bool buttons[], double axes[]) const;
	void toggle(const bool keys, bool &currentButton, bool &on, std_msgs::Float32 &message);
	void toggleInt(const bool keys, bool &currentButton, bool &on, std_msgs::Int8 &message);
	void toggleSaveOdomData(const bool keys, bool &currentButton, bool &on, std_msgs::Bool &message);
	void whileHeld(bool button, std_msgs::Int8 & msg, double value);

private:
    bool _buttons[12] = { 0 };
	double _axes[7] = { 0 };
};


void Listener::joyListener(const sensor_msgs::Joy::ConstPtr& Joy)
{
	for (int i = 0 ; i < 12; i++)
		_buttons[i] = Joy->buttons[i];

    for (int i = 0; i < 7; i++)
        _axes[i] = Joy->axes[i];
}

void Listener::getJoyVals(bool buttons[], double axes[]) const
{
    for (int i = 0; i < 12; i++)
        buttons[i] = _buttons[i];

    for (int i = 0; i < 7; i++)
        axes[i] = _axes[i];
}
void Listener::whileHeld(bool button, std_msgs::Int8 & msg, double value)
{
	if (button)
	{
		msg.data = value;
	}
	else
	{
		msg.data = 1;
	}
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "TPortController");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	Listener listener;

	ros::Subscriber joySub = n.subscribe("TPort/joy", 100, &Listener::joyListener, &listener);
	
	bool buttons[12];
	double axes[7];
	
	//axes
	int ForwardAxis = {JoyMap::TPortForwardAxis};
    int TurnAxis = {JoyMap::TPortTurnAxis};
	int SaveData = {JoyMap::SaveOdomData};

	//buttons
	int ConveyorToggle = {JoyMap::TPortConveyorToggle};
	int ToggleExtension = {JoyMap::TPortToggleExtension};

	bool currentButton = false;
	bool on = false;

	bool currentButtonExtend = false;
	bool onExtend = false;

	bool odomButton = false;
	bool onOdomButton = false;

	bool odomButtonValue = false;

	ros::Publisher l_speed_pub = n.advertise<std_msgs::Float32>("TPortRDrvPwr", 100);
    ros::Publisher r_speed_pub = n.advertise<std_msgs::Float32>("TPortLDrvPwr", 100);
	ros::Publisher conveyor_pub = n.advertise<std_msgs::Float32>("TPortConveyorDrvPwr", 100);
	ros::Publisher extend_pub = n.advertise<std_msgs::Int8>("TPortExtendPwr", 100);
	ros::Publisher save_data_pub = n.advertise<std_msgs::Bool>("SaveData", 100);
	ros::Publisher trencher_drive_toggle_pub = n.advertise<std_msgs::Bool>("ExcvTrencherDriveToggle", 100);

	std_msgs::Bool save_data_msg;
    std_msgs::Float32 l_speed_msg;
    std_msgs::Float32 r_speed_msg;
	std_msgs::Float32 conveyor_pwr;
	std_msgs::Int8 extend_pwr;
	
	while (ros::ok())
	{
        listener.getJoyVals(buttons, axes);
		listener.toggle(buttons[ConveyorToggle], currentButton, on, conveyor_pwr);
		listener.toggleInt(buttons[ToggleExtension], currentButtonExtend, onExtend, extend_pwr);

		//gets d-pad axis value as 1 or 0.
		odomButtonValue = axes[SaveData] == -1? 1 : 0;
		//toggle message using odomButtonValue
		listener.toggleSaveOdomData(odomButtonValue, odomButton, onOdomButton, save_data_msg);

		
		l_speed_msg.data = pow(axes[ForwardAxis], 3.0) * DRIVE_SCALE; // left Y
		r_speed_msg.data = pow(axes[TurnAxis], 3.0) * DRIVE_SCALE; // right Y

		l_speed_pub.publish(l_speed_msg);
		r_speed_pub.publish(r_speed_msg);
		conveyor_pub.publish(conveyor_pwr); // conveyor power
		save_data_pub.publish(save_data_msg);

		extend_pub.publish(extend_pwr);
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

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

void Listener::toggleInt(const bool keys, bool &currentButton, bool &on, std_msgs::Int8 &message)
{
	bool lastButton;
	lastButton = currentButton;
	currentButton = keys;

	if (lastButton && !currentButton)
	{
		on = !on;
		ROS_INFO("Extend button released");
	}
		
	if (on)
	{
		ROS_INFO("Extend button on");
		message.data = 1;
	}
	else
	{
		ROS_INFO("Extend button off");
		message.data = 0;
	}
}

void Listener::toggleSaveOdomData(const bool keys, bool &currentButton, bool &on, std_msgs::Bool &message)
{
	bool lastButton;
	lastButton = currentButton;
	currentButton = keys;

	if (lastButton && !currentButton)
	{
		on = !on;
		ROS_INFO("Odom save button released");
	}
		
	if (on)
	{
		ROS_INFO("Odom save button on");
		message.data = 1;
	}
	else
	{
		ROS_INFO("Odom save button off");
		message.data = 0;
	}

	

}
