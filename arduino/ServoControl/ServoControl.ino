/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

Servo servo;
Servo flag1;
Servo flag2;

int extendPos;
int flag1Pos;
int flag2Pos;

void servo_cb( const std_msgs::UInt16& cmd_msg){
  extendPos = cmd_msg.data; //set servo angle, should be from 0-180
    //toggle led  
}

void flag_cb( const std_msgs::UInt16& msg){
  flag1Pos = msg.data; //set servo angle, should be from 0-180  
  flag2Pos = msg.data;
  digitalWrite(13, HIGH-digitalRead(13));
}



ros::Subscriber<std_msgs::UInt16> sub("TPortExtendPos", servo_cb);
ros::Subscriber<std_msgs::UInt16> flagsub("TPortFlagPos", flag_cb);

void setup(){
  pinMode(13, OUTPUT);

  servo.attach(9); //attach it to pin 9
  flag1.attach(11); //attach it to pin 11
  flag2.attach(5); //attach it to pin 5

  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(flagsub);
  
  

}

void loop(){
  nh.spinOnce();
  servo.write(extendPos); //set servo angle, should be from 0-180
  flag1.write(flag1Pos); //set servo angle, should be from 0-180  
  flag2.write(flag2Pos);
  //delay(1);
}
