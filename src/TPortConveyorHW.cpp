#include "ctre/Phoenix.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt16.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "DeviceIDs.h"

using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

/*****************************************************************************
****     This node sets the trasnportation conveyor motor to the speed    ****
****         set in TPortConveyorDrvPwr and sets the linear actuator to   ****
****         the value set in TPortExtendPwr                              ****
****     Subscribers:                                                     ****
****          std_msgs/Float32 TPortConveyorDrvPwr - conveyor power value ****
****          std_msgs/Int8 TportExtendPwr - extender true/false value    ****
****     Publishers:                                                      ****
****          std_msgs/UInt16 TportExtendPos - tport extender position    ****
*****************************************************************************/

class Listener
{
    public:
        void setExtendSpeed(const std_msgs::Int8 msg);
        void setDriveSpeed(const std_msgs::Float32 drivespeed);
        void setExtendPos(std_msgs::UInt16 extend_pos);
        

    private:
        VictorSPX TPortConveyorDrvVic = {DeviceIDs::TPortConveyorDrvVic};
        CANifier _canifer = {DeviceIDs::canifier};
        int extendVal = 0;

};

void Listener::setExtendPos(std_msgs::UInt16 extend_pos)
{
    int maxPos = 140;
    int minPos = 45;

    if(extendVal == 1)
    {
        extend_pos.data = maxPos;
    }
    else if(extendVal == -1)
    {
        extend_pos.data = minPos;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "TPortConveyorHW");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    phoenix::platform::can::SetCANInterface("can0");


    Listener listener;

    ros::Subscriber extendSpeedSub = n.subscribe("TPortExtendPwr", 100, &Listener::setExtendSpeed, &listener);
    ros::Subscriber driveSpeedSub = n.subscribe("TPortConveyorDrvPwr", 100, &Listener::setDriveSpeed, &listener);

    ros::Publisher extendPos_pub = n.advertise<std_msgs::UInt16>("TPortExtendPos", 100);

    std_msgs::UInt16 extend_pos;	

    while (ros::ok())
    {
        listener.setExtendPos(extend_pos);

        extendPos_pub.publish(extend_pos);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void Listener::setExtendSpeed(const std_msgs::Int8 msg)
{
	// limit values
	extendVal = msg.data;
}



void Listener::setDriveSpeed(const std_msgs::Float32 msg)
{
    TPortConveyorDrvVic.Set(ControlMode::PercentOutput, msg.data);

    ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
}

