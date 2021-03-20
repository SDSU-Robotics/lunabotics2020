#include <iostream>
#include "Task.h"
#include "TaskingVariables.h"
#include "geometry_msgs/Pose.h"

using namespace std;

Task::Task()
{


    
}

bool Task::initialize()
{
    return false;
}
bool Task::basic()
{
    //geometry_msgs:/Pose
    return false;
}
bool Task::onFinish()
{
    return false;
}
bool Task::navigation(geometry_msgs:Pose pos)
{
    return false;
}