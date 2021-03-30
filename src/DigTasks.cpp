#include "ros/ros.h"
#include "std_msgs/Float32.h" 
//#include "move_base_msgs/MoveBaseAction.h"
#include "GlobalVariables.h"
#include "std_msgs/Bool.h"
#include <string>
#include <iostream>
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Float64.h>
#include "std_msgs/Header.h"
using namespace std;

#define NAVIGATION_FRAME "frame"

class DigTasks
{
    public:
        void setExcvLinearActuatorVar(std_msgs::Float32 &msg);
        void setExcvDrvCurrent(std_msgs::Float32 msg, float excvDrvCurrent);
        bool startExcv(std_msgs::Bool &msg); //lower linear actuator and apply motor current for torque while conveyor belt is activated
        bool stopExcv(std_msgs::Bool &msg); //stop above defined process
        bool startConveyor(std_msgs::Bool &msg);
        bool stopConveyor(std_msgs::Bool &msg);
<<<<<<< HEAD
        bool NavTask(double xPt, double zPt, double yRot, int time, int duration, geometry_msgs::PoseStamped &Position);
        //void trencherToggle(const std_msgs::Bool msg);
        //void toggleConveyorOn(const std_msgs::Bool msg);
        float excvLinearActuatorPos = 0;
        float excvDrvCurrent = 0;
        float ExcvConveyorDrvPwr = 0;
        void timercallback(const ros::TimerEvent&);
=======
        bool extLinAct(std_msgs::Float32 &msg);
>>>>>>> 4cad30e9303176cfc8695e10a7a20a72a48c50d4
        
};

    /*
    void setExcvLinearActuatorVar(std_msgs::Float32 &msg)
    {

	    // Set linear actuator position
	    excvLinearActuatorPos = msg.data;
    }
    */


    /*void DigTasks::trencherToggle(const std_msgs::Bool msg)
    {
        PIDEnable = true;
    }*/ //this is used for autonomous raising/lowering



    void setExcvDrvCurrent(std_msgs::Float32 msg, float excvDrvCurrent)
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
<<<<<<< HEAD
    ros::Publisher NavTaskPub = n.advertise<geometry_msgs::PoseStamped>("NavTaskData", 100);
	//ros::Subscriber ExcvLinearActuatorPosSub = n.subscribe("ExcvExtendCurrent", 1000, &DigTasks::setExcvLinearActuatorVar, &digTasks);

    std_msgs::Bool ExcvTrencherEnableMsg;
    std_msgs::Bool ExcvConveyorEnableMsg;
    geometry_msgs::PoseStamped Position; //PoseStamped msg for NavTask
=======
	ros::Subscriber ExcvLinearActuatorPosSub = n.subscribe("ExcvExtendCurrent", 1000, &DigTasks::setExcvLinearActuatorVar, &digTasks);

    std_msgs::Bool ExcvTrencherEnableMsg;
    std_msgs::Bool ExcvConveyorEnableMsg;
    std_msgs::Float32 excvLinActPosMsg;

>>>>>>> 4cad30e9303176cfc8695e10a7a20a72a48c50d4
    
    // data for NavTask function
    double xPt = 5;
    double zPt = 6;
    double yRot = 7;
    int time;
    int duration;
    string s = "hello";

    ros::Timer timer = n.createTimer(ros::Duration(1), timercallback);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
<<<<<<< HEAD
        //digTasks.startExcv(ExcvTrencherEnableMsg);
        //digTasks.stopExcv(ExcvTrencherEnableMsg);
        //digTasks.startConveyor(ExcvConveyorEnableMsg);
        //digTasks.stopConveyor(ExcvConveyorEnableMsg);
        digTasks.NavTask(xPt, zPt, yRot, time, duration, Position);
        TrencherEnablePub.publish(ExcvTrencherEnableMsg);
        NavTaskPub.publish(Position);
=======
        digTasks.extLinAct(excvLinActPosMsg);
        digTasks.startConveyor(ExcvConveyorEnableMsg);
        digTasks.stopConveyor(ExcvConveyorEnableMsg);
        conveyorTogglePub.publish(ExcvConveyorEnableMsg);
>>>>>>> 4cad30e9303176cfc8695e10a7a20a72a48c50d4
    }
    
    return 0;
}


/*
// fully extends linear actuator
bool extLinAct(std_msgs::Float32 &msg)
{
    bool extending = true;
    excvLinearActuatorPos = GlobalVariables::ExcvMaxPotReading;

    if (excvLinearActuatorPos == GlobalVariables::ExcvMaxPotReading)
    {
        extending = false;
    }

    return extending;
}
*/


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

bool DigTasks::NavTask(double xPt, double zPt, double yRot, int time, int duration, geometry_msgs::PoseStamped &Position)
{
    Position.header.stamp.sec = time;
    Position.header.stamp.nsec = duration;
    Position.header.frame_id = NAVIGATION_FRAME;

    
    Position.pose.position.x = xPt;
    Position.pose.position.y = 0;
    Position.pose.position.z = zPt;
    Position.pose.orientation.x = 0;
    Position.pose.orientation.y = yRot;
    Position.pose.orientation.w = 0;
    Position.pose.orientation.z = 0;
    
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