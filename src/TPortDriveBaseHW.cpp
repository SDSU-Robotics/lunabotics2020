#include "ctre/Phoenix.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "DeviceIDs.h"
#include <iostream>

using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

#define LINEAR_ADJ 1
#define ANGULAR_ADJ 1

/****************************************************************************
****     This node subscribes to the motor values set in TportRDrvPwr and****
****          TPortLDrvPwr and sets the motor speeds respectively        ****
****     Subscribers:                                                    ****
****          std_msgs/Float32 TPortRDrvPwr - tport right motor power    ****
****          std_msgs/Float32 TPortLDrvPwr - tport left motor power     ****
****************************************************************************/

class Listener
{
    public:

		Listener();

        void setLSpeed(const std_msgs::Float32 lspeed);
        void setRSpeed(const std_msgs::Float32 rspeed);
		void setTwistSpeed(const geometry_msgs::Twist twist);


   //private:
        TalonSRX leftDrive = {DeviceIDs::TPortDrvLTal};
        TalonSRX rightDrive = {DeviceIDs::TPortDrvRTal};
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "TPortDriveBase");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	phoenix::platform::can::SetCANInterface("can0");

	Listener listener;

	ros::Subscriber lSpeedSub = n.subscribe("TPortLDrvPwr", 100, &Listener::setLSpeed, &listener);
	ros::Subscriber rSpeedSub = n.subscribe("TPortRDrvPwr", 100, &Listener::setRSpeed, &listener);
	ros::Subscriber twistSpeedSub = n.subscribe("cmd_vel", 100, &Listener::setTwistSpeed, &listener);

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

Listener::Listener()
{
	rightDrive.SetInverted(true);
}

void Listener::setLSpeed(const std_msgs::Float32 lspeed)
{
    leftDrive.Set(ControlMode::PercentOutput, lspeed.data);

	ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
}

void Listener::setRSpeed(const std_msgs::Float32 rspeed)
{
    rightDrive.Set(ControlMode::PercentOutput, rspeed.data);

	ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
}

void Listener::setTwistSpeed(const geometry_msgs::Twist twist)
{
	leftDrive.Set(ControlMode::PercentOutput, LINEAR_ADJ * twist.linear.x + ANGULAR_ADJ * twist.angular.z)
	rightDrive.Set(ControlMode::PercentOutput, LINEAR_ADJ * twist.linear.x - ANGULAR_ADJ * twist.angular.z)

	ctre::phoenix::unmanaged::FeedEnable(100);
}
