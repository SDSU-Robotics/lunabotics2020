//This program interfaces with the trencher hardware on the Excavation bot 
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
        void setExtendSpeed(const std_msgs::Float32 extendspeed); //Extender arm speed
        void setPitchSpeed(const std_msgs::Float32 pitchspeed); // Pitch speed
		void setDriveSpeed(const std_msgs::Float32 drivespeed); //Drive speed 

    private:
		// motor controls using Victors
        VictorSPX extendVictor = {DeviceIDs::ExcvExtendVic};
        VictorSPX pitchVictor = {DeviceIDs::ExcvPitchVic};
		VictorSPX driveVictor = {DeviceIDs::ExcvConveyorDrvVic};
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "ExcvTrencherHW");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	phoenix::platform::can::SetCANInterface("can0");

	Listener listener;

	// get speeds from listeners
	ros::Subscriber pitchSpeedSub = n.subscribe("ExcvConveyorDrvPwr", 100, &Listener::setPitchSpeed, &listener);
	ros::Subscriber driveSpeedSub = n.subscribe("ExcvTrencherDrvPwr", 100, &Listener::setDriveSpeed, &listener);

	while (ros::ok()) // while ros is running
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

// Sets speeds and sends information to victors
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

