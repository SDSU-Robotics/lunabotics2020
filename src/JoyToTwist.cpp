
#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

class Listener
{
public:
  void RSpeedListener(std_msgs::Float64 r_speed_msg);
  void LSpeedListener(std_msgs::Float64 l_speed_msg);
  double getRSpeed() { return r_speed; }
  double getLSpeed() { return l_speed; }
private:
    double r_speed;
    double l_speed;
};


void Listener::RSpeedListener(std_msgs::Float64 r_speed_msg)
{
    r_speed = r_speed_msg.data;
}
void Listener::LSpeedListener(std_msgs::Float64 l_speed_msg)
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

  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  ros::Rate loop_rate(5);

  while (ros::ok())
  {
    

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = listener.getRSpeed();
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;//listener.getLSpeed();
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = listener.getLSpeed();

    vel_pub.publish(cmd_vel);

    //vel_pub_0.publish(cmd_vel);
    //vel_pub_1.publish(cmd_vel);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}