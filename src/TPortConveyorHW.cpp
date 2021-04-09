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

#define CONVEYOR_SPEED_SCALE 0.1

class Listener
{
    public:
        void setExtendSpeed(const std_msgs::Int8 msg);
        void setFlagSpeed(const std_msgs::Int8 msg);
        void setDriveSpeed(const std_msgs::Float32 drivespeed);
        //void setExtendPos(std_msgs::UInt16 &extend_pos);
        //void setFlagPos(std_msgs::UInt16 &flag_pos);
        VictorSPX TPortConveyorDrvVic = {DeviceIDs::TPortConveyorDrvVic};   

    private:
        int extendVal = 0;
        int flagVal = 0;

};
/*
void Listener::setExtendPos(std_msgs::UInt16 &extend_pos)
{
    int maxPos = 150;
    int minPos = 45;

    if(extendVal == 1)
    {
        extend_pos.data = maxPos;
    }
    else if(extendVal == 0)
    {
        extend_pos.data = minPos;
    }
}

void Listener::setFlagPos(std_msgs::UInt16 &flag_pos)
{
    int maxPos = 140;
    int minPos = 45;

    if(flagVal == 1)
    {
        flag_pos.data = maxPos;
    }
    else if(flagVal == 0)
    {
        flag_pos.data = minPos;
    }
}
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "TPortConveyorHW");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    phoenix::platform::can::SetCANInterface("can0");
    
    std_msgs::Float32 conveyor_current_msg;

    Listener listener;

    //ros::Subscriber extendSpeedSub = n.subscribe("TPortExtendPwr", 100, &Listener::setExtendSpeed, &listener);
    //ros::Subscriber flagSpeedSub = n.subscribe("TPortFlag", 100, &Listener::setFlagSpeed, &listener);
    ros::Subscriber driveSpeedSub = n.subscribe("TPortConveyorDrvPwr", 100, &Listener::setDriveSpeed, &listener);

    ros::Publisher extendPos_pub = n.advertise<std_msgs::UInt16>("TPortExtendPos", 100);
    ros::Publisher flagPos_pub = n.advertise<std_msgs::UInt16>("TPortFlagPos", 100);
    ros::Publisher conveyor_current_pub = n.advertise<std_msgs::Float32>("TPortConveyorDrvCurrent", 100);

    std_msgs::UInt16 extend_pos;	
    std_msgs::UInt16 flag_pos;	

    while (ros::ok())
    {   
        

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
/*
void Listener::setExtendSpeed(const std_msgs::Int8 msg)
{
	// limit values
	extendVal = msg.data;
    setExtendPos(extend_pos);
    extendPos_pub.publish(extend_pos);
}

void Listener::setFlagSpeed(const std_msgs::Int8 msg)
{
	// limit values
	flagVal = msg.data;
    setFlagPos(flag_pos);
    flagPos_pub.publish(flag_pos);
}
*/
void Listener::setDriveSpeed(const std_msgs::Float32 msg)
{
    TPortConveyorDrvVic.Set(ControlMode::PercentOutput, msg.data * CONVEYOR_SPEED_SCALE);

    ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
}

