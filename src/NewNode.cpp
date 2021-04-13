#include "ros/ros.h"
#include "Task.h"
#include "TaskManager.h"
#include <list>

bool func()
{

    return true;
}

bool func1()
{
    ROS_INFO("FUNCTION 1 RAN");
    return true;
}

class NewTask : public Task
{
    public:
    bool basic() override
    {
        ROS_INFO("Task RAN");
        ros::Duration(0.01).sleep();
       
        return false;
    }
};

class NewTask1 : public Task
{
    public:
    bool initialize() override
    {        
        ROS_INFO("Task1 Init");
        ros::Duration(0.01).sleep();
       
        return false;
    }

    bool basic() override
    {
        ROS_INFO("Task1 RAN");
        ros::Duration(0.01).sleep();
       
        return false;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "NewNode");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    NewTask t1;
    NewTask1 t2;
    
    TaskManager tm;
    tm.addTask(t1);
    tm.addTask(t2);
    
    while (ros::ok())
    {
        tm.cycle();
        //TaskLoop::exitLoop();

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}