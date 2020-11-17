
//This program gets values from a joystick controller publishes them &
// toggles each button on and off after button press
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

	//void toggleDrvSpeedUp(const bool keys, bool &currentButton, double maxSpeed,
	// double speed, double stepSize, std_msgs::Float32 &message);
	//void Listener::toggleDrvSpeedDown(const bool keys, bool &currentButton,
	// double maxSpeed, double speed, double stepSize, std_msgs::Float32 &message);

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

void Listener::getJoyVals(bool buttons[], double axes[]) const
// gets the values from the buttons and axes arrays and sets them to either true or falso
{
    for (int i = 0; i < 12; i++)
        buttons[i] = _buttons[i];

    for (int i = 0; i < 6; i++)
        axes[i] = _axes[i];
}
/*
void Listener::toggleDrvSpeedUp(const bool keys, bool &currentButton, double maxSpeed, double speed, double stepSize,  , std_msgs::Float32 &message)
{

	bool lastButton; 
	//gets the last state of the button
	lastButton = currentButton;
	//sets the last state of the button to the current state of the button
	currentButton = keys;
	// sets the boolean value of current value to the value in keys

	if (lastButton && !currentButton)
	{
		if (speed < maxSpeed)
		{
			if (currentButton)
			{
				speed = speed + stepsize;
				message.data = 1;
				ROS_INFO("speed +1");
			}
			

		}
		else if( speed >= maxSpeed)
		{
			speed = maxSpeed;
			ROS_INFO("max speed reached");
		}
		//Toggle On button release
	}
	
	/*if (on)
	{
		//sets Button On
		message.data = 1;
	}
	else
	{
		//sets button off
		message.data = 0;
	}
	
}
*/
/*
void Listener::toggleDrvSpeedDown(const bool keys, bool &currentButton, double maxSpeed, double speed, double stepSize, std_msgs::Float32 &message)
{

	bool lastButton; 
	//gets the last state of the button
	lastButton = currentButton;
	//sets the last state of the button to the current state of the button
	currentButton = keys;
	// sets the boolean value of current value to the value in keys

	if (lastButton && !currentButton)
	{
		if (speed <= maxSpeed)
		{
			if (currentButton)
			{
				speed = speed - stepsize;
				ROS_INFO("speed decreased");
				message.data = 1;
			}
			

		}
		else if (speed > maxSpeed)
			{
				speed = maxSpeed;
				ROS_INFO("Max Speed");
			}
		//Toggle On button release
	}
}
*/

int main (int argc, char **argv)
{
    ros::init(argc, argv, "DriveController");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	Listener listener;

	ros::Subscriber joySub = n.subscribe("joy", 100, &Listener::joyListener, &listener);
	
	bool buttons[12];
	double axes[6];
	// currentButton and on will need to be seperate booleans for each array value. 
	bool currentButton4 = 0;
	bool on1 = false;
	bool currentButton5 = 0;
	bool on0 = false;
	double stepSize = 1;
	double speed = 0;
	double maxSpeed = 10;



	// Publishes the message to the hardware interface
	ros::Publisher l_speed_pub = n.advertise<std_msgs::Float32>("ExcvLDrvPwr", 100);
    ros::Publisher r_speed_pub = n.advertise<std_msgs::Float32>("ExcvRDrvPwr", 100);
	ros::Publisher conveyor_pwr_pub = n.advertise<std_msgs::Float32>("ExcvConveyorDrvPwr", 100);
	ros::Publisher excavator_pwr_pub = n.advertise<std_msgs::Float32>("driveSpeedSub", 100);

	// sets the message to the message variable
    std_msgs::Float32 l_speed_msg;
    std_msgs::Float32 r_speed_msg;
	std_msgs::Float32 conveyor_pwr_msg;
	std_msgs::Float32 excavator_pwr_msg;
	
	while (ros::ok()) // runs while ros is running
	{
        listener.getJoyVals(buttons, axes);
		//listener.toggleDrvSpeedUp(buttons[5], currentButton5, maxSpeed, speed, stepSize, &excavator_pwr_msg, );
		//listener.toggleDrvSpeedDown(buttons[4], currentButton4, maxSpeed, speed, stepSize, &excavator_pwr_msg);

		l_speed_msg.data = axes[1]; // left Y
		r_speed_msg.data = axes[3]; // right Y
		
		l_speed_pub.publish(l_speed_msg); // left speed
		r_speed_pub.publish(r_speed_msg); // right speed
		conveyor_pwr_pub.publish(conveyor_pwr_msg); // conveyor power
		excavator_pwr_pub.publish(excavator_pwr_msg); // excavator power
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}


