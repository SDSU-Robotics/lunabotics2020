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

/****************************************************************************
****     This node sets the excavation conveyor motor to the speed set   ****
****         in ExcvConveyorDrvPwr                                       ****
****     Subscribers:                                                    ****
****          std_msgs/Float32 ExcvConveyorDrvPwr - conveyor power value ****
****************************************************************************/

#define CONVEYOR_SPEED 0.05

class Listener
{
    public:
        void setSpeed(const std_msgs::Float32 hm_speed);
        VictorSPX ConveyorVictor = {DeviceIDs::ExcvConveyorDrvVic};
        
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "ExcvConveyorHw");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	phoenix::platform::can::SetCANInterface("can0"); // set Canable

	ros::Publisher hm_current_pub = n.advertise<std_msgs::Float32>("ExcvHmDrvCurrent", 100);

	std_msgs::Float32 hm_current_msg;
	std_msgs::Float32 conveyor_pwr;

	Listener listener;

	ros::Subscriber SpeedSub = n.subscribe("ExcvConveyorDrvPwr", 100, &Listener::setSpeed, &listener);
	

	while (ros::ok()) //while ros is running
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void Listener::setSpeed(const std_msgs::Float32 msg)
{
    ConveyorVictor.Set(ControlMode::PercentOutput, msg.data* CONVEYOR_SPEED); // set drive speed

	ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
}
