#include <ros.h>
#include <std_msgs/Float32.h>
#include <Servo.h>

Servo lidarServo;

ros::NodeHandle nh;

std_msgs::Float32 angle_msg;

ros::Publisher angle_pub("lidar_angle", &angle_msg);

float pos = 0;

float deg2micro(float angle)
{
  return angle * 10.302 + 543.3;
}

float micro2deg(float micro)
{
  return (micro - 543.3) / 10.302;
}

void setup() {
  lidarServo.attach(3);

  nh.initNode();
  nh.advertise(angle_pub);
}

void loop() {
  for (pos = 0; pos <= 45; pos += 0.2) { // goes from 0 degrees to 180 degrees
    lidarServo.writeMicroseconds(deg2micro(pos));
    angle_msg.data = pos;
    angle_pub.publish(&angle_msg);
    nh.spinOnce();
    delay(10);
  }
  for (pos = 45; pos >= 0; pos -= 0.2) { // goes from 180 degrees to 0 degrees
    lidarServo.writeMicroseconds(deg2micro(pos));
    angle_msg.data = pos;
    angle_pub.publish(&angle_msg);
    nh.spinOnce();
    delay(10);
  }
}
