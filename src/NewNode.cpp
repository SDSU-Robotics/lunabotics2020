#include "ros/ros.h"
#include "Task.h"
#include "Tasking.h"
#include <list>
class A : private Task
{
    public:

    private:
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "NewNode");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    Task t();
    Tasking g();
    A a();

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}