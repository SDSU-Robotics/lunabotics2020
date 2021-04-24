#include <iostream>
#include "Task.h"
#include "TaskingVariables.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include <ros/timer.h>

using namespace std;

//Navigation Variables

Task::Task()
{
}

Task::Task(double _xPos, double _zPos, double _yRot, int time, int duration, geometry_msgs::PoseStamped &position)
{
    taskType = NAVIGATION;

    //Save data needed for navigation function

    xPos = _xPos;
    zPos = _zPos;
    yRot = _yRot;
    posMsg = &position;
}

Task::Task(std_msgs::Float32 &msg)
{
    float32 = &msg;
}

Task::Task(std_msgs::Bool &msg)
{
    boolean = &msg;
}

Task::Task(std_msgs::UInt16 &msg)
{
    uint16 = &msg;
}

Task::Task(bool otherbool, float otherfloat)
{
    ROS_INFO("Task::Task(bool otherbool, float otherfloat) running");
    cbool = otherbool;
    cfloat = otherfloat;
    n = new ros::NodeHandle;    

    cout << cbool << " " << cfloat << endl;   
}

Task::Task(geometry_msgs::TransformStamped tf, std_msgs::Float32 &f1, std_msgs::Float32 &f2)
{
    transformStamped = tf;
    float32 = &f1;
    float32_2 = &f2;
}

Task::Task(geometry_msgs::TransformStamped &tfmsg, float &currentX, float &finalX, float &currentZ, float &finalZ)
{
    tfstamp = &tfmsg;
    currentXPos = &currentX;
    finalXPos = &finalX;
    currentZPos = &currentZ;
    finalZPos = &finalZ;
}

void Task::callback(const ros::TimerEvent&)
{
    ROS_INFO("callback called");
    cbool = false;

}

bool Task::initialize()
{
    return false;
}
bool Task::basic()
{
    ROS_INFO("DEFULT TASK");
    return false;
}
bool Task::onFinish()
{
    return false;
}
bool Task::navigation()
{
    return false;
}



