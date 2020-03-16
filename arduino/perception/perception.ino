/*
 sweeps motor between 2 degree values

 This program drives a unipolar or bipolar stepper motor.
 The motor is attached to digital pins 8 - 11 of the Arduino.

 The motor will step one step at a time, very slowly.  You can use this to
 test that you've got the four wires of your stepper wired to the correct
 pins. If wired correctly, all steps should be in the same direction.

 Use this also to count the number of steps per revolution of your motor,
 if you don't know it.  Then plug that number into the oneRevolution
 example to see if you got it right.

 1/25/2019
 Jack Winter

 */
#include <ros.h>
#include <std_msgs/Float32.h>
#include <Stepper.h>

//ROS Stuff
ros::NodeHandle nh;
std_msgs::Float32 angle_msg;
ros::Publisher angle_pub("lidar_angle", &angle_msg);

const int stepsPerRevolution = 513;  // change this to fit the number of steps per revolution
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

int currentStep = 0;         // number of steps the motor has taken
const int STEPINC = 3;
const float GEAR_RATIO = 31.0/10.0;

float lidarAngle(float,float,float);
int degToStep(float);//converts a degree(float) to a step(int)
float stepToDeg(int);//convert a step(int) to a degree(float)
int gotToStep(int); //input desired position in steps //currentStep position


const int upperPos = degToStep( 0);
const int lowerPos = degToStep( -45);

void setup() 
{ 
  //ROS Stuff
  nh.initNode();
  nh.advertise(angle_pub);  
  
  // set the speed at 60 rpm:
  myStepper.setSpeed(60);
  // initialize the serial port:
  //Serial.begin(9600);

  //get lidar to 0 degrees

  //move until button press

  //while(!buttonPress)
  //{
  //  myStepper.step(-1);
  //}
  //currentStep = 0;
}

void loop() {

for(int i = lowerPos; i<=upperPos; i+=STEPINC) //move clockwise
  {
     goToStep(i);
     angle_msg.data = stepToDeg(currentStep);
     angle_pub.publish(&angle_msg);
     nh.spinOnce();
     delay(10);
  }  
  for(int i = upperPos; i>=lowerPos; i-=STEPINC) //move counter-clockwise
  {
     goToStep(i);
     angle_msg.data = stepToDeg(currentStep);
     angle_pub.publish(&angle_msg);
     nh.spinOnce();
     delay(10);
  }
}

int goToStep(int nextStep)
{
  myStepper.step(nextStep - currentStep);
  currentStep = nextStep;
  return (currentStep);
}

float stepToDeg(int steps)
{
  float degreeOut = (steps * 360.0) / ((float) stepsPerRevolution * 4 * GEAR_RATIO);

  return degreeOut;
}

int degToStep(float degreesIn)
{
  //convert degree of interest into steps 
   int stepsOut = (degreesIn / 360.0) * (float)stepsPerRevolution *4.0 * GEAR_RATIO;

    return stepsOut;
}
