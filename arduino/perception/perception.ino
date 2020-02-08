/*
 moves motor according to input in degrees

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

#include <Stepper.h>

const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

int stepCount = 0;         // number of steps the motor has taken

void setup() {
  // set the speed at 60 rpm:
  myStepper.setSpeed(60);
  // initialize the serial port:
  Serial.begin(9600);
}

float rx_float;
int   rx_int;


void loop() {
//input degrees
     if (Serial.available() > 0) {    // is a character available?
    rx_float = Serial.parseFloat();       // get the character
    Serial.println(rx_float);
    
   
//convert from 360* to 2048 steps per revolution
    rx_float = rx_float * stepsPerRevolution/360;
    rx_int = (int) rx_float;
    Serial.println(rx_int);
//move motor
   
    myStepper.step(rx_int);
  
  // step one step:
  /*myStepper.step(1);
  Serial.print("steps:");
  Serial.println(stepCount);
  stepCount++;
  delay(500);
  */
  }
  else{
    Serial.flush();
  }
}
