#include "ros/ros.h"
#include "Task.h"
#include "Tasking.h"
#include <list>

bool func()
{
    ROS_INFO("FUNCTION RAN");
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "NewNode");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    
    Task t(&func);
    Tasking g;
    g.addTask(t);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}