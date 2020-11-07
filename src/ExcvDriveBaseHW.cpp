#include "ctre/Phoenix.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "DeviceIDs.h"

using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

class Listener
{
    public:

		Listener();

        void setLSpeed(const std_msgs::Float32 lspeed);
        void setRSpeed(const std_msgs::Float32 rspeed);

    private:
        TalonSRX leftDrive = {DeviceIDs::ExcvDrvLTal};
        TalonSRX rightDrive = {DeviceIDs::ExcvDrvRTal};
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "ExcvDriveBase");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	phoenix::platform::can::SetCANInterface("can0");

	Listener listener;

	ros::Subscriber lSpeedSub = n.subscribe("ExcvLDrvPwr", 100, &Listener::setLSpeed, &listener);
	ros::Subscriber rSpeedSub = n.subscribe("ExcvRDrvPwr", 100, &Listener::setRSpeed, &listener);

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