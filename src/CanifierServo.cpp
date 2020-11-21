#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#include "DeviceIDs.h"

#include "ros/ros.h"
#include "std_msgs/Float32.h"

using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

#define MIN_PULSE 450.0		// us
#define MAX_PULSE 2250.0	// us
#define MIN_INPUT 0
#define MAX_INPUT 1.0


class Listener
{
public:
	void setPosition(const std_msgs::Float32 msg);

private:
	CANifier _canifer = {DeviceIDs::canifier};
};


int main (int argc, char **argv)
{
	ros::init(argc, argv, "CanifierServo");
	ros::NodeHandle n;
	
	ctre::phoenix::platform::can::SetCANInterface("can0");

	Listener listener;

	ros::Subscriber speed_sub = n.subscribe("position", 1000, &Listener::setPosition, &listener);
	

	ros::spin();

	return 0;
}


void Listener::setPosition(const std_msgs::Float32 msg)
{
	// limit values
	float pos = msg.data;
	if (pos < MIN_INPUT)	pos = MIN_INPUT;
	if (pos > MAX_INPUT)	pos = MAX_INPUT;

	_canifer.SetGeneralOutput(CANifier::GeneralPin::SPI_CLK_PWM0P, false, true);

	float pulse = LinearInterpolation::Calculate(pos, MIN_INPUT, MIN_PULSE, MAX_INPUT, MAX_PULSE); // pulse length in us

	_canifer.SetPWMOutput(0, pulse / 4200.0); // 4.2 ms period
	
	_canifer.EnablePWMOutput(0, true);

	ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
}