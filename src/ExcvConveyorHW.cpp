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
		void toggle(const bool keys, bool &currentButton, bool &on, std_msgs::Float32 &message);
       
    private:
        VictorSPX HMDrive = {DeviceIDs::ExcvConveyorDrvVic};
        
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "ExcvConveyorHw");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	phoenix::platform::can::SetCANInterface("can0"); // set Canable

	std_msgs::Float32 conveyor_pwr;

	bool buttons[12] = {0};
	double axes[6] = {0};
	bool currentButton = false;
	bool on = false;

	Listener listener;

	ros::Publisher conveyor_pub = n.advertise<std_msgs::Float32>("ExcvConveyorDrvPWR", 100);
	ros::Subscriber SpeedSub = n.subscribe("ExcvConveyorDrvPwr", 100, &Listener::setSpeed, &listener);
	// get the speed from the publisher
	

	while (ros::ok()) //while ros is running
	{
		ros::spinOnce();
		loop_rate.sleep();
		listener.toggle(buttons[0], currentButton, on, conveyor_pwr);
	}

	return 0;
}

void Listener::setSpeed(const std_msgs::Float32 hm_speed)
{
    HMDrive.Set(ControlMode::PercentOutput, hm_speed.data); // set drive speed

	ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
}

void Listener::toggle(const bool keys, bool &currentButton, bool &on, std_msgs::Float32 &message)
{
	bool lastButton;
	lastButton = currentButton;
	currentButton = keys;

	if (lastButton && !currentButton)
	{
		on = !on;
		ROS_INFO("A button released");
	}
		
	if (on)
	{
		ROS_INFO("A button on");
		message.data = 1;
	}
	else
	{
		ROS_INFO("A button off");
		message.data = 0;
	}
}
