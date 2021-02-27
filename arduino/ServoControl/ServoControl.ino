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

void servo_cb( const std_msgs::UInt16& cmd_msg){
  servo.write(cmd_msg.data); //set servo angle, should be from 0-180
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

void flag_cb( const std_msgs::UInt16& msg){
  flag1.write(msg.data); //set servo angle, should be from 0-180  
  flag2.write(msg.data);
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
  //delay(1);
}
