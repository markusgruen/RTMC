#include "RTMCnode.h"
#include <PL_ADXL355.h>

// you can change the name of your node here:
Node node("ADXL 355");


// put your global variables here:
PL::ADXL355 adxl355(17);
auto range = PL::ADXL355_Range::range2g;

void setup() {
  // add your inputs and outputs here:
  node.addOutput("acc_x [g]");  

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
}



void processInputData(void){};  // leave this empty if this is a sensor-Node
void failsafe(void){};  // leave this empty if this is a sensor-Node
