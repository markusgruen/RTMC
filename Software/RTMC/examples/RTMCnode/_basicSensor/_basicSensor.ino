/*****************************************************
  This is the "empty" template for creating an RTMCnode
  that reads a sensor. 
  --> https://github.com/markusgruen/RTMC

  Copyright (c) 2024 Markus Grün
  MIT license
*****************************************************/

/* Hints:
  For i2c, use "Wire"
  for SPI, use "SPI"; CS (chip select) is on pin 17
  for UART, use "Serial2"
  for analog, use "analogRead(pin)", with pin= 26, 27, or 28 (ADC0...2)
*/

#include "RTMCnode.h"

// you can change the name of your node here:
Node node("mySensorName");


// put your global variables here:


void setup() {
  // add your inputs and outputs here:
  node.addOutput("myOutputValue");  

  node.begin();  
  // insert your setup code here



  // end of custom setup code
  node.start();
}

void readSensorData(){
  // add your code for sensor readout here
  float a = 0.0;
  node.output[0].write(a);
}



void processInputData(){}   // leave this empty if this is a sensor-Node
void failsafe(){}           // leave this empty if this is a sensor-Node
void RTMCloop(){}           // leave this empty if this is a sensor-Node
