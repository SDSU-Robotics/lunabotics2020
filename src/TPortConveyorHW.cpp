#include "ctre/Phoenix.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "DeviceIDs.h"

#define MIN_PULSE 450.0		// us
#define MAX_PULSE 2250.0	// us
#define MIN_INPUT 0
#define MAX_INPUT 1.0

using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

class Listener
{
    public:
        void setExtendSpeed(const std_msgs::Float32 msg);
        void setDriveSpeed(const std_msgs::Float32 drivespeed);

    private:
        TalonSRX ExcvConveyorDrvTal = {DeviceIDs::ExcvConveyorDrvTal};
        CANifier _canifer = {DeviceIDs::canifier};
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "TPortConveyorHW");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    phoenix::platform::can::SetCANInterface("can0");


    Listener listener;

    ros::Subscriber extendSpeedSub = n.subscribe("TPortExtendPwr", 100, &Listener::setExtendSpeed, &listener);
    ros::Subscriber driveSpeedSub = n.subscribe("TPortConveyorDrvPwr", 100, &Listener::setDriveSpeed, &listener);	

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void Listener::setExtendSpeed(const std_msgs::Float32 msg)
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

void Listener::setDriveSpeed(const std_msgs::Float32 msg)
{
    ExcvConveyorDrvTal.Set(ControlMode::PercentOutput, msg.data);

    ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
}

