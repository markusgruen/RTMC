/*****************************************************
  This is an example for an RTMCnode, connected to an
  ADXL355 accelerometer.
  --> https://github.com/markusgruen/RTMC

  Copyright (c) 2024 Markus Grün
  MIT license
*****************************************************/

#include "RTMCnode.h"
#include <PL_ADXL355.h>

Node node("ADXL 355"); // you can change the name of your node here

// put your global variables here:
PL::ADXL355 adxl355(17);
auto range = PL::ADXL355_Range::range2g;

void setup() {
  // add your inputs and outputs here:
  node.addOutput("acc_x [g]");  
  // node.addOutput("acc_y [g]");  
  // node.addOutput("acc_z [g]");  

  node.begin();  
  // insert your setup code here

  adxl355.begin();
  adxl355.setRange(range);
  adxl355.enableMeasurement();
  
  // end of custom setup code
  node.start();
}

void readSensorData(){
  // add your code for sensor readout here
  auto accelerations = adxl355.getAccelerations();
  node.output[0].write(accelerations.x);
  // node.output[1].write(accelerations.y);
  // node.output[2].write(accelerations.z);
}

void processInputData(){}   // leave this empty if this is a sensor-Node
void failsafe(){}           // leave this empty if this is a sensor-Node
void RTMCloop(){}           // leave this empty if this is a sensor-Node
