#include "ros/ros.h"
#include "std_msgs/Float32.h" 
#include "std_msgs/UInt16.h"
//#include "move_base_msgs/MoveBaseAction.h"
#include "GlobalVariables.h"

class DugTasks
{
    private:
        // listeners are private
        bool prime(); //prepare flags and lin act
        bool drvTrench; //drive to beginning of current trench being excavated
        bool collectGravel; //drive straight until docked with Dig and flipped gravel switch
        bool drvCollector; //reverse over trench and traverse to collector
        bool alignCollector; //align with collector and dock collector
        bool dumpGravel; //activate belt to transfer gravel to collector
        std_msgs::UInt16 LinearActuatorExtendPwrMsg;

    public:
        //void setLinearActuatorExtendSpeed(const std_msgs::Int8 msg);
        //void setLinearActuatorExtendPos(std_msgs::UInt16 &extend_pos);
        //float extendSpeed = 0;
        //float linearActuatorExtendPos = 0;
        bool startConveyor(std_msgs::Float32 &msg);
        bool stopConveyor(std_msgs::Float32 &msg);
        bool extLinAct(std_msgs::UInt16 &msg);
        bool extFlags(std_msgs::UInt16 &msg);


};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "DugTasks");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    DugTasks dugTasks;
    ros::Publisher ExtendLinearActuatorExtendPwrPub = n.advertise<std_msgs::UInt16>("TPortExtendPwr", 100);
    //ros::Subscriber ExtendLinearActuatorExtendPosSub = n.subscribe("TPortExtendPwr", 100, &DugTasks::setLinearActuatorExtendPos, &dugTasks);
    ros::Publisher DugConveyorTogglePub = n.advertise<std_msgs::Float32>("TPortConveyorDrvPwr", 100);

    std_msgs::Float32 DugConveyorEnableMsg;
    std_msgs::UInt16 ExtLinActMsg;
    std_msgs::UInt16 ExtFlagsMsg;

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        dugTasks.startConveyor(DugConveyorEnableMsg);
        dugTasks.stopConveyor(DugConveyorEnableMsg);
        dugTasks.extLinAct(ExtLinActMsg);
        dugTasks.extFlags(ExtFlagsMsg);
    }
    
    return 0;
}


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