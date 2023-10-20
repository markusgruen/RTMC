/********************************************
  example for receiving data from node connected
  on port 0 and transmitting this data to the
  node connected on port 1 
  *******************************************/
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

