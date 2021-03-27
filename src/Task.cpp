#include <iostream>
#include "Task.h"
#include "TaskingVariables.h"
#include "geometry_msgs/PoseStamped.h"

using namespace std;

//Navigation Variables

Task::Task()
{
}

Task::Task(double _xPos, double _zPos,double _yRot, geometry_msgs::PoseStamped &position)
{
    taskType = NAVIGATION;

    //Save data needed for navigation function

    xPos = _xPos;
    zPos = _zPos;
    yRot = _yRot;
    posMsg = &position;
}

bool Task::initialize()
{
    return false;
}
bool Task::basic()
{
    //geometry_msgs:Pose
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