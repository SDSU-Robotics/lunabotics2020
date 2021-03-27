#include "ros/ros.h"
#include "Task.h"
#include "TaskManager.h"
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

class NewTask : public Task
{
    public:
    bool basic() override
    {
        ROS_INFO("New Task");

        return false;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "NewNode");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    NewTask t1;
    NewTask t2;
    TaskManager tm;
    tm.addTask(t1);
    tm.addTask(t2);
    t1.basic();
    
    while (ros::ok())
    {
        tm.cycle();

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}