#include "ros/ros.h"
#include "Task.h"
#include "Tasking.h"
#include <list>

bool func()
{
    ROS_INFO("FUNCTION RAN");
    return true;
}

bool func1()
{
    ROS_INFO("FUNCTION 1 RAN");
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "NewNode");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    
    Task t(&func);
    Task t1(&func1);
    Tasking g;
    g.addTask(t);
    g.addTask(t1);
    g.getTask(1).task.run();
    g.getTask(1).task.run();
    g.getTask(0).task.run();

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}