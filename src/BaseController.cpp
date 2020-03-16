#include "ros/ros.h"
#include <unistd.h>
#include <geometry_msgs/Twist.h>
using namespace std;

class Listener
{
    public:
        void goalListener(const geometry_msgs::Twist::ConstPtr& cmd_vel);
};

void Listener::goalListener(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{

}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "BaseController");
	ros::NodeHandle n;
	//ros::Rate loop_rate(100);

    Listener listener;

    ros::Subscriber goalSub = n.subscribe("cmd_vel", 100, &Listener::goalListener, &listener);

    while(ros::ok())
    {

        ros::spinOnce();
    }

    return 0;
}