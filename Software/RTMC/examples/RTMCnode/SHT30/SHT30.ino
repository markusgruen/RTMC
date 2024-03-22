/*****************************************************
  This is an example for an RTMCnode, connected to an
  SHT30 temperature and humidity sensor.
  --> https://github.com/markusgruen/RTMC

  Copyright (c) 2024 Markus Grün
  MIT license
*****************************************************/

#include "RTMCnode.h"
#include "SHT31.h"

// you can change the name of your node here:
Node node("SHT30");


// put your global variables here:
SHT31 sht;

void setup() {
  // add your inputs and outputs here:
  node.addOutput("Temperature [degC]");  
  node.addOutput("Humidity [%]");  

  node.begin();  
  // insert your setup code here
  
  Serial.println("Start");

  Wire.begin();
  sht.begin(0x44);
  Wire.setClock(100000);

  
  // end of custom setup code
  node.start();
}

void readSensorData(){
  // add your code for sensor readout here
  sht.read();
  node.output[0].write(sht.getTemperature());
  node.output[1].write(sht.getHumidity());
}



void processInputData(){} // leave this empty if this is a sensor-Node
void failsafe(){}         // leave this empty if this is a sensor-Node
void RTMCloop(){}         // leave this empty if this is a sensor-Node
