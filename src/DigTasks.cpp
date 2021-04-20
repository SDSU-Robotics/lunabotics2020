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
        bool extLinAct(std_msgs::Float32 &msg);
        bool startConveyor(std_msgs::Float32 &msg);
        bool stopConveyor(std_msgs::Float32 &msg);
        bool NavTask(double xPt, double zPt, double yRot, int time, int duration, geometry_msgs::PoseStamped &Position);
        bool Pause();

        float excvLinearActuatorPos = 0;
        float excvDrvCurrent = 0;
        float ExcvConveyorDrvPwr = 0;  
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DigTasks");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    
    DigTasks digTasks;
	
    ros::Publisher ExcvLinearActuatorPosPub = n.advertise<std_msgs::Float32>("ExcvTrencherPos", 100);
    ros::Publisher conveyorTogglePub = n.advertise<std_msgs::Float32>("ExcvConveyorDrvPwr", 100);
    ros::Publisher NavTaskPub = n.advertise<geometry_msgs::PoseStamped>("NavTaskData", 100);
    ros::Publisher PausePub = n.advertise<std_msgs::Bool>("PausePublisher", 100);
	

    //ros::Timer Pause = n.createTimer(ros::Duration(5), timerCallback, bool oneshot = true);

    std_msgs::Float32 ExcvConveyorEnableMsg;
    geometry_msgs::PoseStamped Position; //PoseStamped msg for NavTask
	//ros::Subscriber ExcvLinearActuatorPosSub = n.subscribe("ExcvExtendCurrent", 1000, &DigTasks::setExcvLinearActuatorVar, &digTasks);
    std_msgs::Float32 excvLinActPosMsg;
    
    // data for NavTask function
    double xPt = 5;
    double zPt = 6;
    double yRot = 7;
    int time;
    int duration;
    string s = "hello";

    //ros::Timer timer = n.createTimer(ros::Duration(1), timercallback);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        digTasks.startConveyor(ExcvConveyorEnableMsg);
        digTasks.stopConveyor(ExcvConveyorEnableMsg);
        digTasks.extLinAct(excvLinActPosMsg);
        digTasks.NavTask(xPt, zPt, yRot, time, duration, Position);
        //digTasks.Pause();

        NavTaskPub.publish(Position);
        conveyorTogglePub.publish(ExcvConveyorEnableMsg);

    }
    
    return 0;
}



// fully extends linear actuator
bool DigTasks::extLinAct(std_msgs::Float32 &msg)
{
    //Topic: ExcvTrencherPos
    //Message: ExcvLinActPosMsg

    //set message to 1
    msg.data = 1;

    return false;
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

bool DigTasks::Pause()
{
    // ros::Duration(x).sleep(); sleeps for x seconds
    ros::Duration(5).sleep(); 
    ROS_INFO("paused for 5 seconds");

    return false;
}