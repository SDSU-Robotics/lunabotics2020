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
        void setExtendSpeed(const std_msgs::Float32 extendspeed);
        void setPitchSpeed(const std_msgs::Float32 pitchspeed);
	void setDriveSpeed(const std_msgs::Float32 drivespeed);

    private:
        VictorSPX extendVictor = {DeviceIDs::extendVictor};
        VictorSPX pitchVictor = {DeviceIDs::pitchVictor};
	VictorSPX driveVictor = {DeviceIDs::driveVictor};
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "HardwareInterface");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	phoenix::platform::can::SetCANInterface("can0");

	Listener listener;

	ros::Subscriber extendSpeedSub = n.subscribe("extend_speed", 100, &Listener::setExtendSpeed, &listener);
	ros::Subscriber pitchSpeedSub = n.subscribe("pitch_speed", 100, &Listener::setPitchSpeed, &listener);
	ros::Subscriber driveSpeedSub = n.subscribe("drive_speed", 100, &Listener::setDriveSpeed, &listener);

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void Listener::setExtendSpeed(const std_msgs::Float32 extendspeed)
{
    extendVictor.Set(ControlMode::PercentOutput, extendspeed.data);

	ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
}

void Listener::setPitchSpeed(const std_msgs::Float32 pitchspeed)
{
    pitchVictor.Set(ControlMode::PercentOutput, pitchspeed.data);

	ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
}
void Listener::setDriveSpeed(const std_msgs::Float32 drivespeed)
{
    driveVictor.Set(ControlMode::PercentOutput, drivespeed.data);

	ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
}

