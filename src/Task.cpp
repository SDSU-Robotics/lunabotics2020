#include <iostream>
#include "Task.h"
#include "TaskingVariables.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"

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
    f32Msg = &msg;
}

Task::Task(std_msgs::Bool &msg)
{
    boolMsg = &msg;
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