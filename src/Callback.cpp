#include "Callback.h"

void Callback::callback(const ros::TimerEvent&)
{
    ROS_INFO("called");
}
