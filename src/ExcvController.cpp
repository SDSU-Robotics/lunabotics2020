
//This program gets values from a joystick controller publishes them &
// toggles each button on and off after button press
#include <string>
#include <unistd.h>
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <sensor_msgs/Joy.h>
#include "JoyMap.h"

using namespace std;

/****************************************************************************
****     This node interprets joystick values and publishes values for   ****
****         hardware on the Excavation robot                            ****
****     Subscribers:                                                    ****
****          sensor_msgs/Joy /Excv/joy - excv values from joy node      ****
****     Publishers:                                                     ****
****          std_msgs/Float32 ExcvLDrvPwr - excv left motor power       ****
****          std_msgs/Float32 ExcvRDrvPwr - excv right motor power      ****
****          std_msgs/Float32 ExcvConveyorDrvPwr - conveyor motor power ****
****          std_msgs/Float32 ExcvTrencherDrvPwr - trencher motor power ****
****          std_msgs/Float32 ExcvTrencherPos - trencher set position   ****
****************************************************************************/

class Listener
{
public:
	void joyListener(const sensor_msgs::Joy::ConstPtr& Joy);
	void getJoyVals(bool buttons[], double axes[], bool &lTgrInit, bool &rTgrInit);
	void toggleDrvSpeed(const bool down, const bool up, bool &currentButton4, bool &currentButton5, std_msgs::Float32 &message);
	void toggle(const bool keys, bool &currentButton, bool &on, std_msgs::Float32 &message);
	void toggleLinearActuator(const bool keys, bool &currentButton, bool &on, std_msgs::Float32 &message);
	void trencherPitch(double &upTgr, double &downTgr, std_msgs::Float32 &msg);


private:
    bool _buttons[12] = { 0 }; // declare array for button values
	double _axes[6] = { 0 }; // declare array for axes values

	
};


void Listener::joyListener(const sensor_msgs::Joy::ConstPtr& Joy) 
// listens for the array value of buttons and axes from sensor messages
// & sets them to false
{
	for (int i = 0 ; i < 12; i++)
		_buttons[i] = Joy->buttons[i];

    for (int i = 0; i < 6; i++)
        _axes[i] = Joy->axes[i];
}

void Listener::getJoyVals(bool buttons[], double axes[], bool &lTgrInit, bool &rTgrInit)
// gets the values from the buttons and axes arrays and sets them to either true or false
{
    for (int i = 0; i < 12; i++)
        buttons[i] = _buttons[i];

    for (int i = 0; i < 6; i++)
        axes[i] = _axes[i];
/*
	if(!lTgrInit && axes[2] != 0)
	{
		lTgrInit = true;
	}
	else
	{
		axes[2] = 1;
	}
	
	if(!rTgrInit && axes[5] != 0)
	{
		rTgrInit = true;
	}
	else
	{
		axes[5] = 1;
	}*/
}

void Listener::toggleDrvSpeed(const bool down, const bool up, bool &currentButton4, bool &currentButton5, std_msgs::Float32 &message)
{

	double stepSize = 0.1;
	double maxSpeed = 1;
	double minSpeed = -1;
	bool lastButton4;
	bool lastButton5; 
	//gets the last state of the buttons
	lastButton4 = currentButton4;
	lastButton5 = currentButton5;
	//sets the last state of the button to the current state of the button
	currentButton5 = up;
	currentButton4 = down;
	// sets the boolean value of current value to the value in keys

	if (lastButton5 && !currentButton5)
	{
		if (message.data < maxSpeed)
		{
			//if (currentButton5)
			//{
				message.data = message.data + stepSize;
				ROS_INFO("speed increased");
			//}

			

		}
		
		else if(message.data >= maxSpeed)
		{
			message.data = maxSpeed;
			ROS_INFO("max speed reached");
		}
		//Toggle On button release
	}
	else if (lastButton4 && !currentButton4)
		{
				if(message.data > minSpeed)
				{
					message.data = message.data - stepSize;
					ROS_INFO("speed decreased");
				}
				else if (message.data <= minSpeed)
				{
					message.data = minSpeed;
					ROS_INFO("Min Speed reached");
				}
		}
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
		message.data = 0.08;

	}
	else
	{
		ROS_INFO("A button off");
		message.data = 0;
	}
}

void Listener::toggleLinearActuator(const bool keys, bool &currentButton, bool &on, std_msgs::Float32 &message)
{
	bool lastButton;
	lastButton = currentButton;
	currentButton = keys;

	if (lastButton && !currentButton)
	{
		on = !on;
		ROS_INFO("Button released");
	}
		
	if (on)
	{
		ROS_INFO("Button on");
		message.data = 1;

	}
	else
	{
		ROS_INFO("Button off");
		message.data = 0;
	}
}

void Listener::trencherPitch(double &upTgr, double &downTgr, std_msgs::Float32 &msg)
{
	// y = 1/4 (x-1)^2

	if (upTgr - 1)
	{
		msg.data = 0.25 * ((upTgr - 1) * (upTgr - 1));
	}
	else
	{
		msg.data = -0.25 * ((downTgr - 1) * (downTgr - 1));
	}
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "DriveController");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	Listener listener;

	ros::Subscriber joySub = n.subscribe("Excv/joy", 100, &Listener::joyListener, &listener);
	
	bool buttons[12];
	double axes[6];

	//axes
	int ForwardAxis = {JoyMap::ExcvForwardAxis};
    int TurnAxis = {JoyMap::ExcvTurnAxis};

	//buttons
	int ConveyorToggle = {JoyMap::TPortConveyorToggle};
	int TrenchDriveIncrease = {JoyMap::ExcvTrenchDriveIncrease};
	int TrenchDriveDecrease = {JoyMap::ExcvTrenchDriveDecrease};

	// currentButton and on will need to be seperate booleans for each array value. 
	bool currentButton4 = 0;
	//bool on1 = false;
	bool currentButton5 = 0;
	//bool on0 = false;
	bool currentButton1 = 0;
	bool on1 = false;

	bool currentButton2 = 0;
	bool on2 = false;

	bool lTgrInit = false;
	bool rTgrInit = false;

	// Publishes the message to the hardware interface
	ros::Publisher l_speed_pub = n.advertise<std_msgs::Float32>("ExcvLDrvPwr", 100);
    ros::Publisher r_speed_pub = n.advertise<std_msgs::Float32>("ExcvRDrvPwr", 100);
	ros::Publisher conveyor_pwr_pub = n.advertise<std_msgs::Float32>("ExcvConveyorDrvPwr", 100);
	ros::Publisher excavator_pwr_pub = n.advertise<std_msgs::Float32>("ExcvTrencherDrvPwr", 100);
	ros::Publisher excavator_pos_pub = n.advertise<std_msgs::Float32>("ExcvTrencherPos", 100);
	ros::Publisher excavator_pitch_pub = n.advertise<std_msgs::Float32>("ExcvTrencherPitch", 100);

	// sets the message to the message variable
	std_msgs::Float32 l_speed_msg;
	std_msgs::Float32 r_speed_msg;
	std_msgs::Float32 conveyor_pwr_msg;
	std_msgs::Float32 excavator_pwr_msg;
	std_msgs::Float32 excavator_pos_msg;
	std_msgs::Float32 excavator_pitch_msg;


	while (ros::ok()) // runs while ros is running
	{
        listener.getJoyVals(buttons, axes, lTgrInit, rTgrInit);
		listener.toggleDrvSpeed(buttons[4], buttons[5], currentButton4, currentButton5, excavator_pwr_msg);
		listener.toggle(buttons[0], currentButton1, on1, conveyor_pwr_msg);
		listener.toggleLinearActuator(buttons[7], currentButton2, on2, excavator_pos_msg);
		listener.trencherPitch(axes[5], axes[2], excavator_pitch_msg);

		/*l_speed_msg.data = axes[1]; // left Y
		r_speed_msg.data = axes[3]; // right Y (changed from 4 to 3 - check)*/
		
		l_speed_msg.data = axes[ForwardAxis]; // left Y
		r_speed_msg.data = axes[TurnAxis]; // right Y

		l_speed_pub.publish(l_speed_msg); // left speed
		r_speed_pub.publish(r_speed_msg); // right speed

		conveyor_pwr_pub.publish(conveyor_pwr_msg); // conveyor power
		excavator_pwr_pub.publish(excavator_pwr_msg); // excavator power
		
		excavator_pos_pub.publish(excavator_pos_msg);
		excavator_pitch_pub.publish(excavator_pitch_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}


