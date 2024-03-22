/*****************************************************
  This is the "empty" template for creating an RTMCnode
  that controls an actuator. 
  --> https://github.com/markusgruen/RTMC

  Copyright (c) 2024 Markus Grün
  MIT license
*****************************************************/



#include "RTMCnode.h"
#include <PL_ADXL355.h>

// you can change the name of your node here:
Node node("This-is-my-name");


// put your global variables here:


void setup() {
  // add your inputs and outputs here:
  node.addOutput("myOutput");  
  node.addInput("myInput", "int")  // only "int" or "float" are allowed

  node.begin();  
  // insert your setup code here



  // end of custom setup code
  node.start();
}

void readSensorData(){
  // add your code for actuator status here
  
  // e.g.:
  // node.output[0].write(actuator.getCurrentPosition);
}



void processInputData(){
  // add your code for processing the input data here
  // DO NOT USE BLOCKING FUNCTIONALITIES (such delay(), etc.)
  
  // e.g.
  // actuator.moveTo(node.input[0].readInt());
}   

void RTMCloop(){
  // the "classic" loop function. Add your code here to be performed continuously

  // e.g.
  // actuator.run();
}

void failsafe(){
  // add your code here for actions that should be performed, if the communication to the host is disturbed

  // e.g.
  // actuator.stop() ;
}