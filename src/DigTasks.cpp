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
    private:
        // listeners are private
        bool prime; //put linear actuator in vertical position
        bool drvExcvZone; //traverse straight until excavation zone
        bool extLinAct(); //extend linear actuator to full extension
        bool slowAndSteady; //once linear actuator is 180 degress from prime position, drive forward at constant rate
        bool relocate; //if too close to edge, reverse from trench and begin new trench
        bool reverse; //reverse function for relocating
        bool liftLinAct; //when reversed from wall, lift lin act to vertical position
        bool newTrench; //repeat necessary above functions
        std_msgs::Float32 ExcvLinearActuatorPosMsg; 
        std_msgs::Float32 ExcvDrvCurrentMsg;
        
        //std_msgs::Bool toggleConveyorMsg;

    public:
        void setExcvLinearActuatorVar(const std_msgs::Float32 msg);
        void setexcvDrvCurrent(const std_msgs::Float32 msg);
        bool startExcv(std_msgs::Bool &msg); //lower linear actuator and apply motor current for torque while belt is activated
        bool stopExcv(std_msgs::Bool &msg); //stop excavating
        bool startConveyor(std_msgs::Bool &msg);
        bool stopConveyor(std_msgs::Bool &msg);
        //void trencherToggle(const std_msgs::Bool msg);
        //void toggleConveyorOn(const std_msgs::Bool msg);
        float excvLinearActuatorPos = 0;
        float excvDrvCurrent = 0;
        float ExcvConveyorDrvPwr = 0;
        
        //bool toggleConveyorMsg = false;
};


    void DigTasks::setExcvLinearActuatorVar(const std_msgs::Float32 msg)
    {

	    // Set linear actuator position
	    excvLinearActuatorPos = msg.data;
    }



    /*void DigTasks::trencherToggle(const std_msgs::Bool msg)
    {
        PIDEnable = true;
    }*/ //this is used for autonomous raising/lowering



    void setexcvDrvCurrent(const std_msgs::Float32 msg, float excvDrvCurrent)
    {
        //Set conveyor belt speed
        excvDrvCurrent = msg.data;
    }




int main(int argc, char **argv)
{
    ros::init(argc, argv, "DigTasks");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    DigTasks digTasks;
	ros::Publisher ExcvLinearActuatorPosPub = n.advertise<std_msgs::Float32>("ExcvTrencherPos", 100);
    ros::Publisher ConveyorSpeedPub = n.advertise<std_msgs::Float32>("ExcvDrvCurrent", 100);
    ros::Publisher TrencherEnablePub = n.advertise<std_msgs::Bool>("ExcvTrencherToggle", 100);
    ros::Publisher conveyorTogglePub = n.advertise<std_msgs::Bool>("ExcvConveyorDrvPwr", 100);
	//ros::Subscriber ExcvLinearActuatorPosSub = n.subscribe("ExcvExtendCurrent", 1000, &DigTasks::setExcvLinearActuatorVar, &digTasks);

    std_msgs::Bool ExcvTrencherEnableMsg;
    std_msgs::Bool ExcvConveyorEnableMsg;
    
    string s = "";

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        digTasks.startExcv(ExcvTrencherEnableMsg);
        digTasks.stopExcv(ExcvTrencherEnableMsg);
        digTasks.startConveyor(ExcvConveyorEnableMsg);
        digTasks.stopConveyor(ExcvConveyorEnableMsg);
        TrencherEnablePub.publish(ExcvTrencherEnableMsg);
    }
    
    return 0;
}

// fully extends linear actuator
bool DigTasks::extLinAct()
{
    bool extending = true;
    excvLinearActuatorPos = GlobalVariables::ExcvMaxPotReading;

    if (excvLinearActuatorPos == GlobalVariables::ExcvMaxPotReading)
    {
        extending = false;
    }

    return extending;
}


// excavates
bool DigTasks::startExcv(std_msgs::Bool &msg)
{
    //This function is meant to begin the trencher of dig to excavate.
    //For conveyor functionalities, refer to the conveyor naming convention (NOT Trencher!!!)
    
    // Topic: ExcvTrencherToggle
    // Message: ExcvTrencherEnableMsg
    
    msg.data = true;

    return false;  
} 

// stop excavating
bool DigTasks::stopExcv(std_msgs::Bool &msg)
{
    // This function stops the trencher to stop excavating.
    
    // Topic: ExcvTrencherToggle
    // Message: ExcvTrencherEnableMsg
    
    msg.data = false;

    return false;
}

bool DigTasks::startConveyor(std_msgs::Bool &msg)
{
    // This function starts the conveyor on Dig.
    
    // Topic: ExcvConveyorDrvPwr
    // Message: ExcvConveyorEnableMsg

    msg.data = true;

    return false;
}

bool DigTasks::stopConveyor(std_msgs::Bool &msg)
{
    // This function stops the conveyor on Dig.

    // Topic: ExcvConveyorDrvPwr
    // Message: ExcvConveyorEnableMsg

    msg.data = false;

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