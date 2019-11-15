#include <ros.h>
#include <std_msgs/Float32.h>

//define pin name
#define lpwm 10
#define ldir 12
#define rpwm 11
#define rdir 13

ros::NodeHandle nh;

void left_cb( const std_msgs::Float32 msg)
{
  if (msg.data < 0)
    digitalWrite(ldir, false);
  else
    digitalWrite(ldir, true);

  analogWrite(lpwm, abs(msg.data) * 255);
}

void right_cb( const std_msgs::Float32 msg)
{
  if (msg.data < 0)
    digitalWrite(rdir, false);
  else
    digitalWrite(rdir, true);

  analogWrite(rpwm, abs(msg.data) * 255);
}

ros::Subscriber<std_msgs::Float32> lsub("l_speed", &left_cb );
ros::Subscriber<std_msgs::Float32> rsub("r_speed", &right_cb );

void setup() {
  pinMode(lpwm, OUTPUT);
  pinMode(rpwm, OUTPUT);
  pinMode(ldir, OUTPUT);
  pinMode(rdir, OUTPUT);

  nh.initNode();
  nh.subscribe(lsub);
  nh.subscribe(rsub);
}

void loop() {
  nh.spinOnce();
  //delay(1);
}
