#include "ctre/Phoenix.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "DeviceIDs.h"
#include "ctre/phoenix/motorcontrol/SensorCollection.h"
#include <iostream>
#include <string>

using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;
//using namespace ctre::phoenix::MotorControl::SensorCollection;

#define LINEAR_ADJ 1
#define ANGULAR_ADJ 1

/*******************************************************************************
****     This node subscribes to the motor values set in ExcvLDrvPwr and 	****
****         ExcvRDrvPwr and sets the motors speeds respectively         	****
****     Subscribers:                                                    	****
****          std_msgs/Float32 ExcvLDrvPwr - left motor value            	****
****          std_msgs/Float32 ExcvRDrvPwr - right motor value              ****
****		  geometry_msgs/Twist cmd_vel   - tport left & right motor power****
*******************************************************************************/

class Listener
{
    public:

		Listener();

        void getLSpeed(const std_msgs::Float32 lspeed);
        void getRSpeed(const std_msgs::Float32 rspeed);
		void getTwistSpeed(const geometry_msgs::Twist twist);
		void setMotorOutput(const float left, const float right);
		float leftPower = 0;
		float rightPower = 0;
		
        TalonSRX leftDrive = {DeviceIDs::ExcvDrvLTal};
        TalonSRX rightDrive = {DeviceIDs::ExcvDrvRTal};
	//	SensorCollection lDrive(leftDrive);//(leftDrive.getSensorCollection());

		//ctre::phoenix::motorcontrol::SensorCollection::SensorCollection lDrive(leftDrive);
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "ExcvDriveBase");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	// Publishes the message to the hardware interface
	ros::Publisher l_current_pub = n.advertise<std_msgs::Float32>("ExcvLDrvCurrent", 100);
	ros::Publisher r_current_pub = n.advertise<std_msgs::Float32>("ExcvRDrvCurrent", 100);


    // sets the message to the message variable
	std_msgs::Float32 l_current_msg;
	std_msgs::Float32 r_current_msg;

	phoenix::platform::can::SetCANInterface("can0");
	
	Listener listener;

	ros::Subscriber lSpeedSub = n.subscribe("ExcvLDrvPwr", 100, &Listener::getLSpeed, &listener);
	// Left speed of excavator drive power

	ros::Subscriber rSpeedSub = n.subscribe("ExcvRDrvPwr", 100, &Listener::getRSpeed, &listener);
	// Right speed of excavator drive power

	ros::Subscriber twistSpeedSub = n.subscribe("cmd_vel", 100, &Listener::getTwistSpeed, &listener);
	// Right and Left speed of excavator drive power

	

	int x;
	string mssg;

	while (ros::ok())
	{
		//x = phoenix::motorcontrol::SensorCollection::GetQuadratureVelocity(listener.leftDrive); 
		x = listener.leftDrive.GetSensorCollection().GetQuadratureVelocity();
		mssg = to_string(x);

		ROS_INFO_STREAM("Msg: " << mssg);

		listener.setMotorOutput(listener.leftPower, listener.rightPower);
		l_current_msg.data = listener.leftDrive.GetOutputCurrent();
		l_current_pub.publish(l_current_msg);
		r_current_msg.data = listener.rightDrive.GetOutputCurrent();
		r_current_pub.publish(r_current_msg);
		
		

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

Listener::Listener()
{
	rightDrive.SetInverted(true);
	//lDrive = leftDrive.GetSensorCollection();
}

void Listener::getLSpeed(const std_msgs::Float32 lspeed)
{
	leftPower = lspeed.data;
}

void Listener::getRSpeed(const std_msgs::Float32 rspeed)
{
	rightPower = rspeed.data;
}

void Listener::getTwistSpeed(const geometry_msgs::Twist twist)
{
	leftPower = LINEAR_ADJ * twist.linear.x + ANGULAR_ADJ * twist.angular.z;
	rightPower = LINEAR_ADJ * twist.linear.x - ANGULAR_ADJ * twist.angular.z;
}

void Listener::setMotorOutput(const float left, const float right)
{
	leftDrive.Set(ControlMode::PercentOutput, left);
	rightDrive.Set(ControlMode::PercentOutput, right);

	ctre::phoenix::unmanaged::FeedEnable(100);	
}