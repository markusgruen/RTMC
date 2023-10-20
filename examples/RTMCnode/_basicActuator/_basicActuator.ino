/**********************************
  basic code required for RTMCnode.
  */



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
  // add your code for sensor readout here
  float a = 0.0;
  node.output[0].write(a);
}



void processInputData(void){};  // leave this empty if this is a sensor-Node
void failsafe(void){};  // leave this empty if this is a sensor-Node
