//This program interfaces with the trencher hardware on the Excavation bot 
#include "ctre/Phoenix.h"
#include "ctre/phoenix/CANifier.h"
#include "ctre/phoenix/CANifierControlFrame.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "DeviceIDs.h"
#include <time.h>
#include <iostream>

using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

/****************************************************************************
****     This node subscribes to the motor values set in                 ****
****         ExcvConveyorPitchPwr and ExcvTrencherDrvPwr and sets        ****
****         the motors speeds respectively                              ****
****     Subscribers:                                                    ****
****          std_msgs/Float32 ExcvConveyorPitchPwr - pitch motor value  ****
****          std_msgs/Float32 ExcvTrencherDrvPwr - trencher motor value ****
****************************************************************************/

#define targetCurrent 2.5

class Listener
{
    public:
        void setExtendSpeed(const std_msgs::Float32 extendspeed); //Extender arm speed
        void setPitchSpeed(const std_msgs::Float32 pitchspeed); // Pitch speed
		void setDriveSpeed(); //Drive speed 
		void getDriveSpeed(const std_msgs::Float32 drivespeed);
		void setPosition();
		void trencherToggle(const std_msgs::Bool toggle);
		double getActualCurrent();
		double getPercentOutput();
		int linearActuator();
		// motor controls using Victors
        TalonSRX pitchTalon = {DeviceIDs::ExcvPitchTal};
		TalonSRX driveTalon = {DeviceIDs::ExcvDriveTal};

		float TrencherDrvPwr;
		bool PIDEnable;

		float P = 0.1;
		float I = 0.01;
		
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "ExcvTrencherHW");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

//	pcf8591Setup (int pinBase, int 12cAddress);

	phoenix::platform::can::SetCANInterface("can0");

	// Publishes the message to the hardware interface
	//ros::Publisher pitch_current_pub = n.advertise<std_msgs::Float32>("ExcvPitchCurrent", 100);
	ros::Publisher drive_current_pub = n.advertise<std_msgs::Float32>("ExcvDrvCurrent", 100);

	// sets the message type to the message variable
	std_msgs::Float32 pitch_current_msg;
	std_msgs::Float32 drive_current_msg;

	Listener listener;

	//ExcvConveyorPitchPwr
	// get speeds from listeners
	ros::Subscriber pitchSpeedSub = n.subscribe("ExcvTrencherPitchPwr", 100, &Listener::setPitchSpeed, &listener);
	ros::Subscriber driveSpeedSub = n.subscribe("ExcvTrencherDrvPwr", 100, &Listener::getDriveSpeed, &listener);
	ros::Subscriber trencherToggleSub = n.subscribe("ExcvTrencherToggle", 100, &Listener::trencherToggle, &listener);

	while (ros::ok()) // while ros is running
	{
		if(listener.PIDEnable == true)
		{
			listener.setPosition();
		}
		else
		{
			listener.setDriveSpeed();
		}
		
		drive_current_msg.data = listener.driveTalon.GetOutputCurrent();
		drive_current_pub.publish(drive_current_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void Listener::setPosition()
{
	float driveOut;
	float eT = targetCurrent - driveTalon.GetOutputCurrent();

	ros::Time timeCurrent;
	ros::Time timeLast = timeCurrent;
	timeCurrent = ros::Time::now();
	ros::Duration timeDiff = timeCurrent - timeLast;

	double dT = timeDiff.toSec();

	driveOut = P*eT + I*(eT*dT);

	if(driveOut > 0.05)
		driveOut = 0.05;
	else if(driveOut < -0.25)
		driveOut = -0.25;


	//Set motor to newly mapped position
	driveTalon.Set(ControlMode::PercentOutput, -1);
	pitchTalon.Set(ControlMode::PercentOutput, driveOut);


	ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
}

void Listener::setPitchSpeed(const std_msgs::Float32 pitchspeed)
{
    //pitchTalon.Set(ControlMode::PercentOutput, pitchspeed.data);

	ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
}

void Listener::setDriveSpeed()
{
    driveTalon.Set(ControlMode::PercentOutput, TrencherDrvPwr);

	ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
}

void Listener::getDriveSpeed(const std_msgs::Float32 drivespeed)
{
	TrencherDrvPwr = drivespeed.data;
}

void Listener::trencherToggle(const std_msgs::Bool toggle)
{
	PIDEnable = toggle.data;
}

double Listener::getActualCurrent()
{
	return driveTalon.GetOutputCurrent();
}

double Listener::getPercentOutput()
{
	return driveTalon.GetClosedLoopError();
}


