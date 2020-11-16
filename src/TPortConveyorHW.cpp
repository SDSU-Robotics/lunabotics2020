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
        void setDriveSpeed(const std_msgs::Float32 drivespeed);

    private:
        VictorSPX ExcvConveyorDrvVic = {DeviceIDs::ExcvConveyorDrvVic};
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "HardwareInterface");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    phoenix::platform::can::SetCANInterface("can0");

    Listener listener;

    ros::Subscriber extendSpeedSub = n.subscribe("TPortConveyorEXT", 100, &Listener::setExtendSpeed, &listener);
    ros::Subscriber driveSpeedSub = n.subscribe("TPortConveyorPWR", 100, &Listener::setDriveSpeed, &listener);
    
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void Listener::setExtendSpeed(const std_msgs::Float32 extendspeed)
{

}

void Listener::setDriveSpeed(const std_msgs::Float32 extendspeed)
{
    ExcvConveyorDrvVic.Set(ControlMode::PercentOutput, extendspeed.data);

    ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
}