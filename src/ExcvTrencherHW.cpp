//This program interfaces with the trencher hardware on the Excavation bot 
#include "ctre/Phoenix.h"
#include "ctre/phoenix/CANifier.h"
#include "ctre/phoenix/CANifierControlFrame.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/motorcontrol/SensorCollection.h"
#include "DeviceIDs.h"
#include <time.h>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>

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

//#define targetCurrent 4.75
//#define targetPos -1608 // -2208

class Listener
{
    public:
        void setExtendSpeed(const std_msgs::Float32 extendspeed); //Extender arm speed
        void setPitchSpeed(const std_msgs::Float32 pitchspeed); // Pitch speed
		void setDriveSpeed(); //Drive speed 
		void getDriveSpeed(const std_msgs::Float32 drivespeed);
		//void setPosition();
		void setPosition(int angPos);
		//void setpitchTalon.GetSensorCollection().GetQuadratureVelocity()DrivePID(std_msgs::Float32 & l_speed_msg, std_msgs::Float32 & r_speed_msg);
		void trencherToggle(const std_msgs::Bool toggle);
		void trencherDriveToggle(const std_msgs::Bool toggle);
		double getActualCurrent();
		double getPercentOutput();
		int linearActuator();
		// motor controls using Victors
        TalonSRX pitchTalon = {DeviceIDs::ExcvPitchTal};
		TalonSRX driveTalon = {DeviceIDs::ExcvDriveTal};

		float TrencherDrvPwr;
		float pitchSpeed;
		bool PIDEnable;
		bool DrivePIDEnable;

		int targetPos = -1308;
		int variablePos = -1308;
		int maxDepth = -2000;

		/*
		float P = 0.05;
		float I = 0.0001;
		float D = 0.01;
		*/
		float P = 0.01;
		float I = 0.005;
		float D = 0.001;

		float etLast = 0;
		int i = 0;
		int n = 0;
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
	ros::Publisher l_speed_pub = n.advertise<std_msgs::Float32>("ExcvLDrvPwr", 100);
    ros::Publisher r_speed_pub = n.advertise<std_msgs::Float32>("ExcvRDrvPwr", 100);

	// sets the message type to the message variable
	std_msgs::Float32 pitch_current_msg;
	std_msgs::Float32 drive_current_msg;
	std_msgs::Float32 l_speed_msg;
	std_msgs::Float32 r_speed_msg;
	
	


	Listener listener;

	//ExcvConveyorPitchPwr
	// get speeds from listeners
	ros::Subscriber pitchSpeedSub = n.subscribe("ExcvTrencherPitchPwr", 100, &Listener::setPitchSpeed, &listener);
	ros::Subscriber driveSpeedSub = n.subscribe("ExcvTrencherDrvPwr", 100, &Listener::getDriveSpeed, &listener);
	ros::Subscriber trencherToggleSub = n.subscribe("ExcvTrencherToggle", 100, &Listener::trencherToggle, &listener);
	ros::Subscriber trencherDriveToggleSub = n.subscribe("ExcvTrencherDriveToggle", 100, &Listener::trencherDriveToggle, &listener);


	int angPos;

	while (ros::ok()) // while ros is running
	{
		if(listener.DrivePIDEnable == true)
		{
			//listener.setDrivePID(l_speed_msg, r_speed_msg);
			l_speed_pub.publish(l_speed_msg); // left speed
			r_speed_pub.publish(r_speed_msg); // right speed
		}
		else{

			if(listener.PIDEnable == true)
			{
				angPos = listener.pitchTalon.GetSensorCollection().GetQuadraturePosition();
				//cout << angPos << endl;

      			listener.setPosition(angPos);
								
			}
			else
			{
				

				listener.setDriveSpeed();
				listener.pitchTalon.Set(ControlMode::PercentOutput, listener.pitchSpeed);

				ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
			}
		}

		drive_current_msg.data = listener.driveTalon.GetOutputCurrent();
		drive_current_pub.publish(drive_current_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void Listener::setPosition(int angPos)
{
	using namespace std::this_thread;
	using namespace std::chrono;

	

	
	float driveOut;
	//float eT = targetCurrent - driveTalon.GetOutputCurrent();
	float eT = targetPos - angPos;

	int currentPos;
	int newPos;
	int motorSpeed = 0;

	ros::Time timeCurrent;
	ros::Time timeLast = timeCurrent;
	timeCurrent = ros::Time::now();
	ros::Duration timeDiff = timeCurrent - timeLast;

	double dT = timeDiff.toSec();
	double IC = 0;
	IC += I*eT;

	if(IC > 1000)IC = 1000;
	if(IC < -1000)IC = -1000;

	double dC = D * (eT - etLast);

	driveOut = P*eT + IC + dC;
	//driveOut = P*eT + I*(eT*dT);
	
	/*
	if(driveOut > 0.0)
		driveOut = 0.0;
	else if(driveOut < -0.3)
		driveOut = -0.3;
	*/

if(angPos > targetPos/2)
{
	if(driveOut > 0.75)
		driveOut = 0.75;
	else if(driveOut < -0.75)
		driveOut = -0.75;

}
else
{
	if(driveOut > 0.375)
		driveOut = 0.375;
	else if(driveOut < -0.375)
		driveOut = -0.375;

	//motorSpeed = pitchTalon.GetSensorCollection().GetQuadratureVelocity();
	cout << pitchTalon.GetSensorCollection().GetQuadraturePosition() << endl;
	
	if(motorSpeed == 0)
	{
		targetPos = variablePos; 

		if(targetPos != maxDepth)
		{
		//sleep_for(seconds(5));
		variablePos -= 1;
		}
	}

}	

	/*
	if(eT < 0)
		driveOut=0;
	*/
	

	//cout<<driveOut << "\t" << driveTalon.GetOutputCurrent() << endl;
	

	//Set motor to newly mapped position
	//driveTalon.Set(ControlMode::PercentOutput, 0.75);
	pitchTalon.Set(ControlMode::PercentOutput, driveOut);

	ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog

	etLast = eT;
	//lastPos = pitchTalon.GetSensorCollection().GetQuadraturePosition();

	//sleep_for(seconds(5));
	//if(angPos == pitchTalon.GetSensorCollection().GetQuadraturePosition())
	//{
	//	cout << "working" << endl;
	//}
}

/*
void Listener::setDrivePID(std_msgs::Float32 & l_speed_msg, std_msgs::Float32 & r_speed_msg)
{
	float driveOut;
	float eT = targetCurrent - driveTalon.GetOutputCurrent();

	ros::Time timeCurrent;
	ros::Time timeLast = timeCurrent;
	timeCurrent = ros::Time::now();
	ros::Duration timeDiff = timeCurrent - timeLast;

	double dT = timeDiff.toSec();

	driveOut = P*eT + I*(eT*dT);

	if(driveOut > 0.6)
		driveOut = 0.6;
	else if(driveOut < -0.6)
		driveOut = -0.6;

	l_speed_msg.data = driveOut;
	r_speed_msg.data = driveOut;

	//Set motor to newly mapped position
	driveTalon.Set(ControlMode::PercentOutput, -1);


	ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
}
*/
void Listener::setPitchSpeed(const std_msgs::Float32 pitchspeed)
{
	pitchSpeed = pitchspeed.data;
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

void Listener::trencherDriveToggle(const std_msgs::Bool toggle)
{
	DrivePIDEnable = toggle.data;
}

double Listener::getActualCurrent()
{
	return driveTalon.GetOutputCurrent();
}

double Listener::getPercentOutput()
{
	return driveTalon.GetClosedLoopError();
}


