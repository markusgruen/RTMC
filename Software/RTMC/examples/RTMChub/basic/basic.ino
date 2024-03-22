/*****************************************************
  This is the basic code for RTMChub, for operating as 
  a DAQ or datalogger (only sensor-nodes connected) 
  --> https://github.com/markusgruen/RTMC

  Copyright (c) 2024 Markus Grün
  MIT license
*****************************************************/

#include "RTMChub.h"


RTMChub rtmcHub;

void setup() {
  rtmcHub.begin();
  // put your setup code here

  // ***************************************
  // No need to put any code here for basic functionality
  // ***************************************

  // end of custom setup code
  rtmcHub.start();
}


void processData() {
  // ***************************************
  // No need to put any code here for basic functionality
  // ***************************************
}

