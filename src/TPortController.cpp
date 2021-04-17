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
****          std_msgs/Bool TPortConveyorDrvPwr - conveyor motor power****
****          std_msgs/Int8 TportExtendPwr - extender true/false value   ****
****************************************************************************/

#define DRIVE_SCALE 1

class Listener
{
public:
	void joyListener(const sensor_msgs::Joy::ConstPtr& Joy);
	void getJoyVals(bool buttons[], double axes[]) const;
	void toggle(const bool keys, bool &currentButton, bool &on, std_msgs::Bool &message);
	//void whileHeld(bool button, std_msgs::Int8 & msg, double value);
	

	void toggleIntExtend(const bool keys, bool &currentButton, bool &on, std_msgs::UInt16 &message, ros::Publisher extend_pub);
	void toggleIntFlag(const bool keys, bool &currentButton, bool &on, std_msgs::UInt16 &message, ros::Publisher flag_pub);

private:
    bool _buttons[12] = { 0 };
	double _axes[8] = { 0 };
};


void Listener::joyListener(const sensor_msgs::Joy::ConstPtr& Joy)
{
	for (int i = 0 ; i < 12; i++)
		_buttons[i] = Joy->buttons[i];

    for (int i = 0; i < 8; i++)
        _axes[i] = Joy->axes[i];
}

void Listener::getJoyVals(bool buttons[], double axes[]) const
{
    for (int i = 0; i < 12; i++)
        buttons[i] = _buttons[i];

    for (int i = 0; i < 8; i++)
        axes[i] = _axes[i];
}
/*
void Listener::whileHeld(bool button, std_msgs::UInt16 & msg, double value)
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
*/
int main (int argc, char **argv)
{
    ros::init(argc, argv, "TPortController");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	Listener listener;

	ros::Subscriber joySub = n.subscribe("TPort/joy", 100, &Listener::joyListener, &listener);
	
	bool buttons[12];
	double axes[8];
	
	//axes
	int ForwardAxis = {JoyMap::TPortForwardAxis};
    int TurnAxis = {JoyMap::TPortTurnAxis};
	int SaveOdomData = {JoyMap::SaveOdomData};
	int SaveDigCollectData = {JoyMap::SaveDigCollectData};	

	//buttons
	int ConveyorToggle = {JoyMap::TPortConveyorToggle};
	int ToggleExtension = {JoyMap::TPortToggleExtension};

	bool currentButton = false;
	bool on = false;

	bool currentButtonExtend = false;
	bool onExtend = false;


	bool odomButton = false;
	bool onOdomButton = false;

	bool digButton = false;
	bool onDigButton = false;
  
  bool currentButtonFlag = false;
	bool onFlag = false;

	bool collectButton = false;
	bool onCollectButton = false;

	bool dpadOdomValue = false;
	bool dpadDigValue = false;
	bool dpadCollectValue = false;
	
	ros::Publisher l_speed_pub = n.advertise<std_msgs::Float32>("TPortRDrvPwr", 100);
    ros::Publisher r_speed_pub = n.advertise<std_msgs::Float32>("TPortLDrvPwr", 100);
	
	
	ros::Publisher conveyor_pub = n.advertise<std_msgs::Bool>("TPortConveyorDrvPwr", 100);
	ros::Publisher trencher_drive_toggle_pub = n.advertise<std_msgs::Bool>("ExcvTrencherDriveToggle", 100);
	ros::Publisher save_odomData_pub = n.advertise<std_msgs::Bool>("SaveOdomData", 100);
	ros::Publisher digData_pub = n.advertise<std_msgs::Bool>("DigData", 100);
	ros::Publisher collectData_pub = n.advertise<std_msgs::Bool>("CollectData", 100);
	ros::Publisher extend_pub = n.advertise<std_msgs::UInt16>("TPortExtendPos", 100);
	ros::Publisher flag_pub = n.advertise<std_msgs::UInt16>("TPortFlagPos", 100);
  
	std_msgs::Bool conveyor_pwr;
	std_msgs::Bool save_odomData_msg;
	std_msgs::Bool digData_msg;
	std_msgs::Bool collectData_msg;

    std_msgs::Float32 l_speed_msg;
    std_msgs::Float32 r_speed_msg;
		std_msgs::UInt16 extend_pwr;
	  std_msgs::UInt16 flag_pwr;
	
	while (ros::ok())
	{
        listener.getJoyVals(buttons, axes);
		listener.toggle(buttons[ConveyorToggle], currentButton, on, conveyor_pwr);

		//toggle message using odomButtonValue
		dpadOdomValue = axes[SaveOdomData] == 1? 1 : 0;
		listener.toggle(dpadOdomValue, odomButton, onOdomButton, save_odomData_msg);

		//toggle message using odomButtonValue
		dpadDigValue = axes[SaveDigCollectData] == 1? 1 : 0;
		listener.toggle(dpadDigValue, digButton, onDigButton, digData_msg);
		
		//toggle message using odomButtonValue
		dpadCollectValue = axes[SaveDigCollectData] == -1? 1 : 0;
		listener.toggle(dpadCollectValue, collectButton, onCollectButton, collectData_msg);

		listener.toggleIntExtend(buttons[ToggleExtension], currentButtonExtend, onExtend, extend_pwr, extend_pub);
		listener.toggleIntFlag(buttons[JoyMap::TPortToggleFlags], currentButtonFlag, onFlag, flag_pwr, flag_pub);

		

		l_speed_msg.data = pow(axes[ForwardAxis], 3.0) * DRIVE_SCALE; // left Y
		r_speed_msg.data = pow(axes[TurnAxis], 3.0) * DRIVE_SCALE; // right Y

		l_speed_pub.publish(l_speed_msg);
		r_speed_pub.publish(r_speed_msg);
		conveyor_pub.publish(conveyor_pwr); // conveyor power

		save_odomData_pub.publish(save_odomData_msg);
		digData_pub.publish(digData_msg);
		collectData_pub.publish(collectData_msg);
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void Listener::toggle(const bool keys, bool &currentButton, bool &on, std_msgs::Bool &message)
{
	bool lastButton;
	lastButton = currentButton;
	currentButton = keys;

	if (lastButton && !currentButton)
	{
		on = !on;
		//ROS_INFO(" button released");
	}
		
	if (on)
	{
		//ROS_INFO(" button on");
		message.data = 1;
	}
	else
	{
		//ROS_INFO(" button off");
		message.data = 0;
	}
}


void Listener::toggleIntFlag(const bool keys, bool &currentButton, bool &on, std_msgs::UInt16 &message, ros::Publisher flag_pub)
{
	bool lastButton;
	lastButton = currentButton;
	currentButton = keys;

	if (lastButton && !currentButton)
	{
		on = !on;
		ROS_INFO("Extend button released");
		flag_pub.publish(message);
	}
		
	if (on)
	{
		ROS_INFO("Extend button on");
		message.data = 140;
	}
	else
	{
		ROS_INFO("Extend button off");
		message.data = 35;
	}
}

void Listener::toggleIntExtend(const bool keys, bool &currentButton, bool &on, std_msgs::UInt16 &message, ros::Publisher extend_pub)
{
	bool lastButton;
	lastButton = currentButton;
	currentButton = keys;

	if (lastButton && !currentButton)
	{
		on = !on;
		ROS_INFO("Extend button released");
		extend_pub.publish(message);
	}
		
	if (on)
	{
		ROS_INFO("Extend button on");
		message.data = 150;
	}
	else
	{
		ROS_INFO("Extend button off");
		message.data = 45;
	}
}
