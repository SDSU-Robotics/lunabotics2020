//This program interfaces with the trencher hardware on the Excavation bot 
#include "ctre/Phoenix.h"
#include "ctre/phoenix/CANifier.h"
#include "ctre/phoenix/CANifierControlFrame.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/motorcontrol/SensorCollection.h"
#include "DeviceIDs.h"
#include <time.h>
#include <iostream>
#include <string>

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

const int arrayL = 10;

class Listener
{
    public:
        void setExtendSpeed(const std_msgs::Float32 extendspeed); //Extender arm speed
        void setPitchSpeed(const std_msgs::Float32 pitchspeed); // Pitch speed
		void setDriveSpeed(); //Drive speed 
		void getDriveSpeed(const std_msgs::Float32 drivespeed);
		
		void decrementPosition(const ros::TimerEvent& event);
		void setPosition();
		void setDrivePID(std_msgs::Float32 & l_speed_msg, std_msgs::Float32 & r_speed_msg);
		
		void trencherToggle(const std_msgs::Bool toggle);
		void trencherDriveToggle(const std_msgs::Bool toggle);
		double getActualCurrent();
		double getPercentOutput();

		int linearActuator();

		// motor controls using Victors
        TalonSRX pitchTalon = {DeviceIDs::ExcvPitchTal};
		TalonSRX driveTalon = {DeviceIDs::ExcvDriveTal};
		TalonSRX wheelLTalon = {DeviceIDs::ExcvDrvLTal};
		TalonSRX wheelRTalon = {DeviceIDs::ExcvDrvRTal};


		float TrencherDrvPwr;
		float pitchSpeed;
		bool PIDEnable = false;
		bool DrivePIDEnable;

		int targetPos = -1600;
		int wheelTargetPos = 0;
		int targetCurrent = 3;
		int maxDepth = -2400;
		int decrementPos = 1;

		float movingArray[arrayL];
		int i=0;

		/*
		float P = 0.05;
		float I = 0.0001;
		float D = 0.01;
		*/

		float trencher_etLast = 0;
		float wheel_etLast = 0;
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
	ros::Publisher angPos_pub = n.advertise<std_msgs::Int32>("ExcvPitchPos", 100);

	// sets the message type to the message variable
	std_msgs::Float32 pitch_current_msg;
	std_msgs::Float32 drive_current_msg;
	std_msgs::Float32 l_speed_msg;
	std_msgs::Float32 r_speed_msg;
	std_msgs::Int32 angPos_msg;
	
	Listener listener;

	//ExcvConveyorPitchPwr
	// get speeds from listeners
	ros::Subscriber pitchSpeedSub = n.subscribe("ExcvTrencherPitchPwr", 100, &Listener::setPitchSpeed, &listener);
	ros::Subscriber driveSpeedSub = n.subscribe("ExcvTrencherDrvPwr", 100, &Listener::getDriveSpeed, &listener);
	ros::Subscriber trencherToggleSub = n.subscribe("ExcvTrencherToggle", 100, &Listener::trencherToggle, &listener);
	ros::Subscriber trencherDriveToggleSub = n.subscribe("ExcvTrencherDriveToggle", 100, &Listener::trencherDriveToggle, &listener);

	ros::Timer timer = n.createTimer(ros::Duration(0.25), &Listener::decrementPosition, &listener);

	while (ros::ok()) // while ros is running
	{
		if(listener.PIDEnable == true)
		{		
			
			listener.setPosition();
			
			angPos_msg.data = listener.pitchTalon.GetSensorCollection().GetQuadraturePosition();
			angPos_pub.publish(angPos_msg);

			listener.setDrivePID(l_speed_msg, r_speed_msg);
			
			l_speed_pub.publish(l_speed_msg); // left speed
			r_speed_pub.publish(r_speed_msg); // right speed

		}
		else
		{
			//cout << listener.driveTalon.GetOutputCurrent() << endl;
			//cout << listener.pitchTalon.GetSensorCollection().GetQuadraturePosition() << endl;
			angPos_msg.data = listener.pitchTalon.GetSensorCollection().GetQuadraturePosition();
			angPos_pub.publish(angPos_msg);

			listener.setDriveSpeed();
			listener.pitchTalon.Set(ControlMode::PercentOutput, listener.pitchSpeed);

			ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
		}

		drive_current_msg.data = listener.driveTalon.GetOutputCurrent();
		drive_current_pub.publish(drive_current_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void Listener::decrementPosition(const ros::TimerEvent& event)
{
	int angPos = pitchTalon.GetSensorCollection().GetQuadraturePosition();

	//reading motor variables
	int motorSpeed;
	int excvMotorCurrent;

	//store calculated current values
	int currentSum = 0;
	int avgCurrent = 0;

	motorSpeed = pitchTalon.GetSensorCollection().GetQuadratureVelocity();
	excvMotorCurrent = driveTalon.GetOutputCurrent();

	//happens after array initialization is finished
	if(i == arrayL)
	{
		//sort array to move forward
		for(int k = arrayL-1; k > 0; k--)
		{
			movingArray[k] = movingArray[k-1];
		}

		//set position 0 to new value
		movingArray[0] = excvMotorCurrent;
	
		//calculate sum of array
		for(int x = 0; x < arrayL; x++)
		{
			currentSum += movingArray[x];
		}
	}
	else
	{
		//fill array with current values
		for(;i<arrayL;i++)
		{
			movingArray[i] = excvMotorCurrent;
		}
	}

	avgCurrent = currentSum/arrayL;

	if(motorSpeed == 0 && angPos < -760)
	{
		driveTalon.Set(ControlMode::PercentOutput, 0.75);

		//decrementPosition according to current
		if(avgCurrent <= targetCurrent)
		{
			if(targetPos > maxDepth)
			{
				targetPos -= decrementPos;
				if(angPos < -1900)
					wheelTargetPos += 1;
			}
				
		}

	}

	cout << targetPos << " " << angPos << " " << excvMotorCurrent << " " << avgCurrent << " " << wheelTargetPos << " " << wheelLTalon.GetSensorCollection().GetQuadraturePosition() << wheelRTalon.GetSensorCollection().GetQuadraturePosition() << endl;

}


void Listener::setPosition()
{
	int angPos = pitchTalon.GetSensorCollection().GetQuadraturePosition();

	//variables to store error between target and current positions + value for motor current
	float driveOut;
	float eT = targetPos - angPos;

	float P = angPos > -760 ? 0.001 : 0.002;
	float I = angPos > -760 ? 0.008 : 0.036;
	float D = angPos > -760 ? 0.036 : 0.002; //0.001

	double IC;
	IC += I*eT;

	if(IC > 0.3)IC = 0.3;
	if(IC < -0.3)IC = -0.3;

	double dC = D * (eT - trencher_etLast);

	driveOut = P*eT + IC + dC;	
	
/*
if(angPos > -790)
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

	if(eT < 0)
		driveOut=0;
*/

	//Set motor to newly mapped position
	pitchTalon.Set(ControlMode::PercentOutput, driveOut);

	ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog

	trencher_etLast = eT;

}
void Listener::setDrivePID(std_msgs::Float32 & l_speed_msg, std_msgs::Float32 & r_speed_msg)
{
	int angPos = pitchTalon.GetSensorCollection().GetQuadraturePosition();

	float excvMotorCurrent = driveTalon.GetOutputCurrent();
	int wheelPos = wheelLTalon.GetSensorCollection().GetQuadraturePosition();

	float driveOut;
	float eT = angPos > -1900 ? 0 - wheelPos : wheelTargetPos - wheelPos;

	float P = 0.003;
	float I = 0.001;
	float D = 0.0001;

	double IC;
	IC += I*eT;

	if(IC > 0.3)IC = 0.3;
	if(IC < -0.3)IC = -0.3;

	double dC = D * (eT - wheel_etLast);

	driveOut = P*eT + IC + dC;

	if(driveOut > 0.8)
		driveOut = 0.8;
	else if(driveOut < -0.8)
		driveOut = -0.8;

	l_speed_msg.data = driveOut;
	r_speed_msg.data = driveOut;

	//Set motor to newly mapped position
	//driveTalon.Set(ControlMode::PercentOutput, -1);

	//ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog

	wheel_etLast = eT;
}

/*void originalFn(){
	float eT = targetCurrent - driveTalon.GetOutputCurrent();

	float driveOut;
	float eT = targetPos - angPos;

	int currentPos;
	int newPos;
	int motorSpeed = 0;

	int driveCurrent;
	int currentSum = 0;
	int averageCurrent;
	

	ros::Time timeCurrent;
	ros::Time timeLast = timeCurrent;
	timeCurrent = ros::Time::now();
	ros::Duration timeDiff = timeCurrent - timeLast;

	double dT = timeDiff.toSec();
	double IC;
	IC += I*eT;

	if(IC > 0.3)IC = 0.3;
	if(IC < -0.3)IC = -0.3;


	if(driveOut > 0.0)
		driveOut = 0.0;
	else if(driveOut < -0.3)
		driveOut = -0.3;
	

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


