#include "ros/ros.h"
#include "std_msgs/Float32.h" 
//#include "move_base_msgs/MoveBaseAction.h"
#include "GlobalVariables.h"
#include "std_msgs/Bool.h"
#include <string>
#include <iostream>
using namespace std;

class DigTasks
{
    public:
        //void setExcvLinearActuatorVar(std_msgs::Float32 msg);
        bool startExcv(std_msgs::Bool &msg); //lower linear actuator and apply motor current for torque while conveyor belt is activated
        bool stopExcv(std_msgs::Bool &msg); //stop above defined process
        bool startConveyor(std_msgs::Float32 &msg);
        bool stopConveyor(std_msgs::Float32 &msg);
        bool extLinAct(std_msgs::Float32 &msg);

        //float excvLinearActuatorCurrent;
};

    /*
    void DigTasks::setExcvLinearActuatorVar(std_msgs::Float32 msg)
    {

	    // Set linear actuator position
	    excvLinearActuatorCurrent = msg.data;

    }*/
    

    /*this is used for autonomous raising/lowering
    void DigTasks::trencherToggle(const std_msgs::Bool msg)
    {
        PIDEnable = true;
    }*/ 
  



int main(int argc, char **argv)
{
    ros::init(argc, argv, "DigTasks");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    DigTasks digTasks;

	ros::Publisher ExcvLinearActuatorPosPub = n.advertise<std_msgs::Float32>("ExcvTrencherPos", 100);
    //ros::Publisher TrencherEnablePub = n.advertise<std_msgs::Float32>("ExcvTrencherToggle", 100);
    ros::Publisher conveyorTogglePub = n.advertise<std_msgs::Float32>("ExcvConveyorDrvPwr", 100);
    
	
    //ros::Subscriber ExcvLinearActuatorPosSub = n.subscribe("ExcvExtendCurrent", ExcvTrencherPos
    std_msgs::Float32 ExcvConveyorEnableMsg;
    std_msgs::Float32 ExcvLinActPosMsg;

    
    string s = "";

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        digTasks.extLinAct(ExcvLinActPosMsg);
        digTasks.startConveyor(ExcvConveyorEnableMsg);
        digTasks.stopConveyor(ExcvConveyorEnableMsg);

        conveyorTogglePub.publish(ExcvConveyorEnableMsg);
        ExcvLinearActuatorPosPub.publish(ExcvLinActPosMsg);
    }
    
    return 0;
}

/*
Finished Tasks:
        digTasks.startConveyor(ExcvConveyorEnableMsg);
        digTasks.stopConveyor(ExcvConveyorEnableMsg);
        digTasks.extLinAct(ExcvLinActPosMsg);

Unfinished Tasks:

*/



// fully extends linear actuator
bool DigTasks::extLinAct(std_msgs::Float32 &msg)
{
    //Topic: ExcvTrencherPos
    //Message: 

    //set message to 1
    msg.data = 1;

    return false;

    
    
    /*bool extending = true;
    excvLinearActuatorCurrent = GlobalVariables::ExcvMaxPotReading;

    if (excvLinearActuatorCurrent == GlobalVariables::ExcvMaxPotReading)
    {
        extending = false;
    }

    return extending;*/
}



bool DigTasks::startConveyor(std_msgs::Float32 &msg)
{
    // This function starts the conveyor on Dig.
    
    // Topic: ExcvConveyorDrvPwr
    // Message: ExcvConveyorEnableMsg

    msg.data = 1;

    return false;
}

bool DigTasks::stopConveyor(std_msgs::Float32 &msg)
{
    // This function stops the conveyor on Dig.

    // Topic: ExcvConveyorDrvPwr
    // Message: ExcvConveyorEnableMsg

    msg.data = 0;

    return false;
}

/*
bool DigTasks::prime()
{
    //lift lin act to vertical
    bool lifting = true;

}

/*
// drives to excavation zone
bool DigTasks::drvExcvZone()
{

}


//excavates while slowly moving forward after switch is flipped
bool DigTasks::slowAndSteady()
{
    
}

// tells robot to relocate because robot has reached a wall/obstacle
bool DigTasks::relocate()
{

}

// reverses robot out of spot
bool DigTasks::reverse()
{
    //if (relocate)
    {

    }
}

// lifts the linear actuator out of the ground to relocate
bool DigTasks::liftLinAct()
{
    //if (relocate)
    {
        PotReading = x;
    }
}

// moves robot to a new spot to start digging again
bool DigTasks::newZone()
{

}*/