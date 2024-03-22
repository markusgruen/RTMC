/*****************************************************
  This is an example for an RTMChub, that receives data
  from a node connected on port0 and transmits this data
  to the node connected on port1
  --> https://github.com/markusgruen/RTMC

  Copyright (c) 2024 Markus Grün
  MIT license
*****************************************************/

#include "RTMChub.h"

RTMChub rtmcHub;

void setup() {
  rtmcHub.begin();
  // put your setup code here


  // end of custom setup code
  rtmcHub.start();
}


void processData() {
  // put your main code here, which processes input data and writes output data.
  int value = node[0].output[0].readInt();
  node[1].input[0].write(value);
}

