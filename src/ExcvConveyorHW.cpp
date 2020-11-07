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
        void setSpeed(const std_msgs::Float32 hm_speed);
       
    private:
        VictorSPX HMDrive = {DeviceIDs::ExcvConveyorDrvVic};
        
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "ExcvConveyorHw");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	phoenix::platform::can::SetCANInterface("can0");

	Listener listener;

	ros::Subscriber SpeedSub = n.subscribe("ExcvConveyorDrvPwr", 100, &Listener::setSpeed, &listener);
	

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void Listener::setSpeed(const std_msgs::Float32 hm_speed)
{
    HMDrive.Set(ControlMode::PercentOutput, hm_speed.data);

	ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
}
