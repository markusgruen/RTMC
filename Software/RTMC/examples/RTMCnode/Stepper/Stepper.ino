/*****************************************************
  This is an example for an RTMCnode, that controls a 
  stepper motor.
  --> https://github.com/markusgruen/RTMC

  Copyright (c) 2024 Markus Grün
  MIT license
*****************************************************/


#include "RTMCnode.h"
#include <AccelStepper.h>

// you can change the name of your node here:
Node node("L298N Stepper");


// put your global variables here:
AccelStepper stepper(4,16,17,18,19);

int targetPosition = 0;

void setup() {
  // add your inputs and outputs here:
  node.addOutput("target position");  // echo the target position, so that it appears in the RTMChub's output
  node.addOutput("current position");  
  node.addInput("target position", "int");  // only "int" or "float" are allowed
  node.begin();  
  // insert your setup code here

  stepper.setMaxSpeed(200);
  stepper.setAcceleration(400);

  // end of custom setup code
  node.start();
}

void readSensorData(){
  // add your code for sensor readout here
  long currentPosition = stepper.currentPosition();
  node.output[0].write(targetPosition);
  node.output[1].write((int) currentPosition);

}

void processInputData(void){
  int newTargetPosition = node.input[0].readInt();
  if(targetPosition != newTargetPosition){
    targetPosition = newTargetPosition;
    stepper.moveTo(targetPosition);
  }
} 

void failsafe(void){
  stepper.disableOutputs();
}  

void RTMCloop(void){
  stepper.run();
}
