#include "ros/ros.h"
#include "std_msgs/Float32.h" 
#include "std_msgs/UInt16.h"
//#include "move_base_msgs/MoveBaseAction.h"
#include "GlobalVariables.h"
#include "DeviceIDs.h"

 class DugTasks
{       

    public:

        bool startConveyor(std_msgs::Float32 &msg);
        bool stopConveyor(std_msgs::Float32 &msg);
        bool extLinAct(std_msgs::UInt16 &msg);
        bool extFlags(std_msgs::UInt16 &msg);
        bool drvForward(std_msgs::Float32 &RMsg, std_msgs::Float32 &LMsg);
        bool drvBackward(std_msgs::Float32 &Rmsg, std_msgs::Float32 &LMsg);
        bool stopDrv(std_msgs::Float32 &RMsg, std_msgs::Float32 &LMsg);
        bool turnLeft(std_msgs::Float32 &RMsg, std_msgs::Float32 &LMsg);
        bool turnRight(std_msgs::Float32 &RMsg, std_msgs::Float32 &LMsg);


};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "DugTasks");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    DugTasks dugTasks;
    
    ros::Publisher l_speed_pub = n.advertise<std_msgs::Float32>("TPortRDrvPwr", 100);
    ros::Publisher r_speed_pub = n.advertise<std_msgs::Float32>("TPortLDrvPwr", 100);
    ros::Publisher ExtendLinearActuatorExtendPwrPub = n.advertise<std_msgs::UInt16>("TPortExtendPwr", 100);
    ros::Publisher DugConveyorTogglePub = n.advertise<std_msgs::Float32>("TPortConveyorDrvPwr", 100);

    std_msgs::Float32 DugConveyorEnableMsg;
    std_msgs::UInt16 ExtLinActMsg;
    std_msgs::UInt16 ExtFlagsMsg;
    std_msgs::Float32 l_speed_msg;
    std_msgs::Float32 r_speed_msg;

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        dugTasks.startConveyor(DugConveyorEnableMsg); //to check data being published to topic, do not run start/stop conveyor functions at same time
        dugTasks.stopConveyor(DugConveyorEnableMsg);
        dugTasks.extLinAct(ExtLinActMsg);
        dugTasks.extFlags(ExtFlagsMsg);
        dugTasks.drvForward(r_speed_msg, l_speed_msg);
        dugTasks.drvBackward(r_speed_msg, l_speed_msg);
        dugTasks.stopDrv(r_speed_msg, l_speed_msg);
        dugTasks.turnLeft(r_speed_msg, l_speed_msg);
        dugTasks.turnRight(r_speed_msg, l_speed_msg);
        l_speed_pub.publish(l_speed_msg);
		r_speed_pub.publish(r_speed_msg);
        DugConveyorTogglePub.publish(DugConveyorEnableMsg);
        ExtendLinearActuatorExtendPwrPub.publish(ExtLinActMsg);
    }
    
    return 0;
}

/*

Finished tasks:

        dugTasks.startConveyor(DugConveyorEnableMsg); 
        dugTasks.stopConveyor(DugConveyorEnableMsg);
        dugTasks.extLinAct(ExtLinActMsg);
        dugTasks.drvForward(r_speed_msg, l_speed_msg);
        dugTasks.drvBackward(r_speed_msg, l_speed_msg);
        dugTasks.stopDrv(r_speed_msg, l_speed_msg);
        dugTasks.turnLeft(r_speed_msg, l_speed_msg);
        dugTasks.turnRight(r_speed_msg, l_speed_msg);

Unfinished tasks:
        dugTasks.extFlags(ExtFlagsMsg);

*/


//prepare flags and lin act
bool DugTasks::extLinAct(std_msgs::UInt16 &msg)
{
    // Topic: TPortExtendPwr
    // Message: ExtLinActMsg
    
    //set msg to 1
    msg.data = 1;

    return false;
}

bool DugTasks::extFlags(std_msgs::UInt16 &msg)
{
    // Topic: TPortExtendPwr
    // Message: ExtFlagsMsg

    msg.data = 1;

    return false;
}

bool DugTasks::startConveyor(std_msgs::Float32 &msg)
{
    // This function starts the conveyor on Dug.
    
    // Topic: TPortConveyorDrvPwr
    // Message: DugConveyorEnableMsg
    
    msg.data = 1;

    return false;
}

bool DugTasks::stopConveyor(std_msgs::Float32 &msg)
{
    // This function stops the conveyor on Dug.
    
    // Topic: TPortConveyorDrvPwr
    // Message: DugConveyorEnableMsg

    msg.data = 0;

    return false;
}

bool DugTasks::drvForward(std_msgs::Float32 &RMsg, std_msgs::Float32 &LMsg)
{
    // Topic: TPortRDrvPwr (left), TPortLDrvPwr (right)
    // yes, TPortRDrvPwr controls the left and TPortLDrvPwr controls the right
    // Message: l_speed_msg, r_speed_msg
    
    // This function sets wheel motors on (forward direction)
    RMsg.data = 1;
    LMsg.data = 1;

    return false;
}

bool DugTasks::drvBackward(std_msgs::Float32 &RMsg, std_msgs::Float32 &LMsg)
{
    // Topic: TPortRDrvPwr (left), TPortLDrvPwr (right)
    // yes, TPortRDrvPwr controls the left and TPortLDrvPwr controls the right
    // Message: l_speed_msg, r_speed_msg
    
    // This function sets wheel motors on (reverse direction)
    RMsg.data = -1;
    LMsg.data = -1;

    return false;
}

bool DugTasks::stopDrv(std_msgs::Float32 &RMsg, std_msgs::Float32 &LMsg)
{
    // Topic: TPortRDrvPwr (left), TPortLDrvPwr (right)
    // yes, TPortRDrvPwr controls the left and TPortLDrvPwr controls the right
    // Message: l_speed_msg, r_speed_msg
    
    // This function stops driving
    
    RMsg.data = 0;
    LMsg.data = 0;

    return false;
}

bool DugTasks::turnLeft(std_msgs::Float32 &RMsg, std_msgs::Float32 &LMsg)
{
    // Topic: TPortRDrvPwr (left), TPortLDrvPwr (right)
    // yes, TPortRDrvPwr controls the left and TPortLDrvPwr controls the right
    // Message: l_speed_msg, r_speed_msg
    
    // This function turns left
    
    RMsg.data = 0.5;
    LMsg.data = 1;

    return false;
}

bool DugTasks::turnRight(std_msgs::Float32 &RMsg, std_msgs::Float32 &LMsg)
{
    // Topic: TPortRDrvPwr (left), TPortLDrvPwr (right)
    // yes, TPortRDrvPwr controls the left and TPortLDrvPwr controls the right
    // Message: l_speed_msg, r_speed_msg
    
    // This function turns right
    
    RMsg.data = 1;
    LMsg.data = 0.5;

    return false;
}
/*
//drive to beginning of current trench being excavated
bool DugTasks::drvTrench()
{

}

//drive straight until docked with Dig and flipped gravel switch
bool DugTasks::collectGravel()
{

}

//reverse over trench and traverse to collector
bool DugTasks::drvCollector()
{

}

//align with collector and dock collector
bool DugTasks::alignCollector()
{

}

//activate belt to transfer gravel to collector
bool DugTasks::dumpGravel()
{

}
*/