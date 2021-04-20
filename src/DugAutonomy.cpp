#include "ros/ros.h"

class StartConveyor : public Task
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
    ros::init(argc, argv, "DugAutonomy");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}