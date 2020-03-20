#include "ros/ros.h"
#include <unistd.h>
#include <geometry_msgs/TwistStamped.h>

using namespace std;

class Listener
{
    public:
        void twistListener(const geometry_msgs::TwistStamped::ConstPtr& publish_cmd);
};

void Listener::twistListener(const geometry_msgs::TwistStamped::ConstPtr& publish_cmd)
{

}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "BaseController");
	ros::NodeHandle n;
	//ros::Rate loop_rate(100);

    Listener listener;

    ros::Subscriber goalSub = n.subscribe("publish_cmd", 100, &Listener::twistListener, &listener);

    while(ros::ok())
    {

        ros::spinOnce();
    }

    return 0;
}