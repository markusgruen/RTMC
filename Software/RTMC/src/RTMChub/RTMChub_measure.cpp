/* MIT License

Copyright (c) 2024 Markus Grün

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifdef __IMXRT1062__
#include "RTMChub/RTMChub_measure.h"
#include "RTMChub/RTMChub_main.h" 
#include "common/serialComm.h"  // for TIMEOUT_MILLIS_NODE_RESET
#include <algorithm>  // for "std::max_element(first, last)"

void RTMCmeasure::nodes(){
  long transmitTime_us[NUM_NODES][MEASURE_CYCLES] = {0};
  long responseTime_us[NUM_NODES][MEASURE_CYCLES] = {0};
  long receiveTime_us[NUM_NODES][MEASURE_CYCLES] = {0};
  
  if(debug) {
      Serial.println(F("[DEBUG] Measuring node response times"));
  }

  // ensure data sent during response measurement is zero
  for(uint8_t i: activeNodes) {
    for(int k=0; k<node[i].numInputs; k++){
      node[i].input[k].write(0);
    }
  }
  TXenable();
  RTMCcore::resetBuffers();

  for(uint8_t i: activeNodes) {
    if(debug) {
      Serial.print(F("[DEBUG] Node: "));
      Serial.print(i);
      Serial.print(F(": "));
    }

    sendCommand(node[i], COMMAND_TEST); 
    delay(10);

    for(int k=0; k<MEASURE_CYCLES; k++){
      transmitTime_us[i][k] = measureTransmitTime_us(i);
      responseTime_us[i][k] = measureResponseTime_us(i);
      receiveTime_us[i][k] = measureReceiveTime_us(i);

      if(responseTime_us[i][k] < 0 || receiveTime_us[i][k] < 0){
        node[i].dataValid = false;  // flag node as invalid
        snprintf(node[i].error, sizeof(node[i].error)-1, "Node took too long to response");
      }

      if(debug) {
        Serial.print(responseTime_us[i][k]);
        Serial.print("  ");
      }
    }

    TXenable();
    sendCommand(node[i], COMMAND_STOP); 
    delay(10);

    node[i].responseTime_us = *std::max_element(responseTime_us[i], responseTime_us[i]+MEASURE_CYCLES);  
    node[i].transmitTime_us = *std::max_element(transmitTime_us[i], transmitTime_us[i]+MEASURE_CYCLES);  
    node[i].receiveTime_us = *std::max_element(receiveTime_us[i], receiveTime_us[i]+MEASURE_CYCLES);  

  if(debug) {
    Serial.println();
    Serial.print(F("[DEBUG] transmit time: "));
    Serial.print(node[i].transmitTime_us);
    Serial.print(F(" us, response time: "));
    Serial.print(node[i].responseTime_us);
    Serial.print(F(" us, receive time: "));
    Serial.print(node[i].receiveTime_us);
    Serial.println(F(" us"));
  }

  }
  removeInvalidNodes(debug);
}

long RTMCmeasure::measureTransmitTime_us(uint8_t i){
  unsigned long startMicros = micros();
  
  TXenable();
  delayMicroseconds(10);  // TODO

  transmitData(node[i]); // transmit Data
  node[i].flush();
  
  delayMicroseconds(10);   // TODO  // make sure data is received on the node's end
  return micros() - startMicros;
}

long RTMCmeasure::measureResponseTime_us(uint8_t i){
  unsigned long startMicros = micros();
  unsigned long responseTime_us = 0;
  
  RXenable();  // rising edge on TRIGGER_PIN triggers nodes's measurement
  // wait until expected number of bytes has been received or timeout
  while((micros() - startMicros) < 1000*TIMEOUT_MILLIS_NODE_RESET) {
    if(node[i].available() == node[i].numReceiveBytes) {
      responseTime_us= micros()-startMicros;
      break;
    }
  }
  if((micros() - startMicros) < 1000*TIMEOUT_MILLIS_NODE_RESET){
    return responseTime_us;
  }
  else {
    if(debug) {
      Serial.print(F(": Timeout"));
    }
    return -1;
  }
}

long RTMCmeasure::measureReceiveTime_us(uint8_t i){
  int numBytesProcessed = 0;
  int8_t numBytesReceived = 0;
  char dummyData[MAX_MESSAGE_LENGTH];
  unsigned long startMicros = micros();
 
  numBytesReceived = node[i].recvData_wait(dummyData, node[i].numReceiveBytes, TIMEOUT_MICROS_SERIAL_RECEIVE);
  if(numBytesReceived == node[i].numReceiveBytes-1){  // -1, because last byte is checksum
    numBytesProcessed = 0;
    for(uint8_t k=0; k<node[i].numOutputs; k++){
      memcpy(node[i].output[k].data, dummyData+numBytesProcessed, sizeof(node[i].output[k].data));
      numBytesProcessed += sizeof(node[i].output[k].data);
    }
    memset(dummyData, 0, sizeof(dummyData));
    return micros()-startMicros;
  }
  else{
    if(debug) {
      Serial.print(F("Node "));
      Serial.print(i);
      Serial.print(F(": Timeout - numBytesReceived: "));
      Serial.println(numBytesReceived);
    }
    return -1;
  }
}

void RTMCmeasure::processTime(){
  unsigned long startMicros = micros();
  processData();
  systemTimes.processT_us = micros()-startMicros;
  
  if(debug) {
    Serial.print(F("[DEBUG] Measuring process time [us]: "));
    Serial.println(systemTimes.processT_us);
  }
}


void RTMCmeasure::printTime(){
  // printing to Serial is very fast. About 0.3 us per character
  // Assumption: 10 characters for each value --> 3 us per value that gets printed out
  // this will over-estimate the required time, but printing time will
  // still be much shorter than the response time
  //
  // otherwise actually measuring the printTime will lead to an actual printout of 
  // random data to Serial. This cannot be deleted and will show up on the screen
  // and on the saved data.
  if(settings.mode == SERIAL_GERMAN || settings.mode == SERIAL_ENGLISH || settings.mode == SERIAL_MONITOR) {
    uint8_t numTotalValues = 0;

    for(uint8_t i: activeNodes) {
      numTotalValues += node[i].numOutputs;
    }
    systemTimes.printT_us = 3*numTotalValues + 3; // +3us for printout of "time"
  }  
  else{
    // TODO
    // measure printout to SD
    systemTimes.printT_us = 0;
  }
}

#endif