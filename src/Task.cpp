#include <iostream>
#include "Task.h"
#include "TaskingVariables.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"

using namespace std;

//Navigation Variables

Task::Task(double _xPos, double _zPos,double _yRot, geometry_msgs::PoseStamped &position)
{
    taskType = NAVIGATION;

    //Save data needed for navigation function

    xPos = _xPos;
    zPos = _zPos;
    yRot = _yRot;
    posMsg = &position;
}

Task::Task(std_msgs::UInt16 &msg)
{
    uint16 = &msg;
}

Task::Task(std_msgs::Float32 &msg)
{
    float32 = &msg;
}

Task::Task(std_msgs::Bool &msg)
{
    boolean = &msg;
}

Task::Task(bool otherbool, float otherfloat)
{
    cbool = otherbool;
    cfloat = otherfloat;
    cout << cbool << " " << cfloat << endl;   
}

Task::Task()
{
    task = new Task(cbool, cfloat);
}

void Task::callback(const ros::TimerEvent&)
{
    ROS_INFO("called");
    boolean -> data = false;
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



