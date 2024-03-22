/*****************************************************
  This is an example for an RTMCnode, that reads the
  RPI's onboard temperature sensor.
  --> https://github.com/markusgruen/RTMC

  Copyright (c) 2024 Markus Grün
  MIT license
*****************************************************/

#include "RTMCnode.h"

// you can change the name of your node here:
Node node("RPi onboard Temp");


// put your global variables here:


void setup() {
  // add your inputs and outputs here:
  node.addOutput("Temp [degC]");  

  node.begin();  
  // insert your setup code here



  // end of custom setup code
  node.start();
}

void readSensorData(){
  // add your code for sensor readout here
  node.output[0].write(analogReadTemp());
}



void processInputData(){}   // leave this empty if this is a sensor-Node
void failsafe(){}           // leave this empty if this is a sensor-Node
void RTMCloop(){}           // leave this empty if this is a sensor-Node
