#include "ros/ros.h"

using namespace std;

int main (int argc, char **argv)
{ 
    ros::init(argc, argv, "ExcvDriveBase");
    ros::NodeHandle n;
    
    ros::Rate loop_rate(100);
    
    while (ros::ok())
    {
        ros::spinOnce();
		loop_rate.sleep();
    }
    return 0;
}
