#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

class Listener
{
public:
  void RSpeedListener(std_msgs::Float32 r_speed_msg);
  void LSpeedListener(std_msgs::Float32 l_speed_msg);
  double getRSpeed() { return r_speed; }
  double getLSpeed() { return l_speed; }
private:
    double r_speed;
    double l_speed;
};


void Listener::RSpeedListener(std_msgs::Float32 r_speed_msg)
{
    r_speed = r_speed_msg.data;
}
void Listener::LSpeedListener(std_msgs::Float32 l_speed_msg)
{
    l_speed = l_speed_msg.data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TwistPub");

  ros::NodeHandle n;

  Listener listener;

  double r_speed_val, l_speed_val;

  ros::Subscriber RSpeedSub = n.subscribe("speed", 100, &Listener::RSpeedListener, &listener);
  ros::Subscriber LSpeedSub = n.subscribe("turn", 100, &Listener::LSpeedListener, &listener);

  ros::Publisher l_vel_pub = n.advertise<std_msgs::Float64>("/transportation/leftWheel_velocity_controller/command", 1);
  ros::Publisher r_vel_pub = n.advertise<std_msgs::Float64>("/transportation/rightWheel_velocity_controller/command", 1);
  ros::Publisher lr_vel_pub = n.advertise<std_msgs::Float64>("/transportation/leftRearWheel_velocity_controller/command", 1);
  ros::Publisher rr_vel_pub = n.advertise<std_msgs::Float64>("/transportation/rightRearWheel_velocity_controller/command", 1);

  ros::Rate loop_rate(5);

  while (ros::ok())
  {
  
    std_msgs::Float64 left;
    std_msgs::Float64 right;

    vel_pub.publish(cmd_vel);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}