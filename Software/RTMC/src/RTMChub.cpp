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

#include "RTMChub.h"
#include "RTMChub/RTMChub_main.h"
#include "RTMChub/RTMChub_pinmapping.h"
#include "util/RTMCutil.h"
#include <Arduino.h>
#include <RTClib.h>
//#include "DST_RTC.h"

constexpr float VERSION{0.4};

// Real Time Clock
RTC_DS3231 rtc;
DateTime now;

// Nodes
Node node[NUM_NODES] = {&Serial1, &Serial2, &Serial3, &Serial4, &Serial5, &Serial6, &Serial7, &Serial8};


// Timer for trigger signal
IntervalTimer trigger;    // IntervalTimer is a built-in function by Teensy, which generates interrupt-based events
IntervalTimer receiver;

volatile IRQhandle irqHandle = {0,      // triggerTime_us
                                0,      // previousTime_us
                                0,      // startTime_us
                                true,   // firstTrigger
                                false}; //startButton

volatile enum SystemStates state = STATE_IDLE;


RTMChub::RTMChub(){
  RTMChub(false);
}

RTMChub::RTMChub(bool _debug){
  debug = _debug;

  init.setDebug(debug);
  measure.setDebug(debug);
  init.gpio();

  analogWriteResolution(15);
  analogWriteFrequency(START_LED_PIN, 4577.64);

  attachInterrupt(digitalPinToInterrupt(START_BUTTON_PIN), startButtonIRQ, FALLING);
  attachInterrupt(digitalPinToInterrupt(MAIN_RESET_PIN), restart, FALLING);
}

void RTMChub::begin(){
  // start the RTC
  rtc.begin();

  // start all Node's UARTs
  for(int i=0; i<NUM_NODES; i++){
    node[i].begin(BAUDRATE_NODE);
  }

  Serial.begin(BAUDRATE_HOST);
  while(!Serial);

  init.getMode(); 
  print.version(VERSION);
  
  init.resetNodes();
  init.getNodeInfo();
  init.getNodeError();

  init.getOutputInfos();
  init.getNodeError();

  init.getInputInfos();
  init.getNodeError();

  measure.nodes();
  measure.processTime();
  measure.printTime();
  calculateSampleTime();

  print.info(systemTimes.samplingT_us);
}

void RTMChub::start(){
  RXenable(); // to make the green LED light up again...

  Serial.println(F("press the START button, press ENTER, or type 'Tx.xx' to start the measurement with a different sampling time [s]\r\n"));
  waitForStartCommand();

  // determine, after how many cycles a node is restarted (because it failed to answer the trigger signal)
  for(unsigned int i: activeNodes) {
    if( (1.2*node[i].responseTime_us / systemTimes.samplingT_us) < 5 ){  // TODO remove magic numbers
      node[i].maxFailCnt = 5;
    }
    else{
      node[i].maxFailCnt = 1.2*node[i].responseTime_us / systemTimes.samplingT_us;
    } 
    if(debug){
      for(unsigned int i: activeNodes) {
        Serial.print(F("[DEBUG] node "));
        Serial.print(i);
        Serial.print(F(" maxFailCnt: "));
        Serial.println(node[i].maxFailCnt);
      }
    }
  }

  // switch on the START-LED
  pinMode(START_LED_PIN, OUTPUT); // must be repeated, to re-enable digitalWrite (after using analogWrite)
  digitalWrite(START_LED_PIN, LOW); // START_LED_ON;

  print.samplingTime(systemTimes.samplingT_us);
  print.startTime(rtc.now());
  print.header();
  resetBuffers();
  state = STATE_IDLE;

  // prepare all variables and start the timers for trigger and receiving
  TXenable();
  sendCommand(COMMAND_MEASURE);
  delay(20); // give the nodes some time to process the command
  noInterrupts()
    irqHandle.triggerTime_us = 0;
    irqHandle.previousTime_us = 0;
    irqHandle.startTime_us = 0;
    irqHandle.firstTrigger = true;
  interrupts();

  trigger.begin(triggerIRQ, systemTimes.samplingT_us);
  triggerIRQ(); // immediatly call the triggerIRQ, to have the first measurement right after pressing the start button
  delayMicroseconds(systemTimes.samplingT_us - systemTimes.lowT_us);  // pulse length -- not systemTimes.highT, to allow user to overwrite samplingT
  receiver.begin(receiverIRQ, systemTimes.samplingT_us);
  receiverIRQ(); // immediatly call the receiverIRQ, to have the first measurement right after pressing the start button
}

void RTMChub::calculateSampleTime(){
  unsigned long maxTransmitTime_us = 0;
  unsigned long maxResponseTime_us = 0;
  unsigned long sumReceiveTime_us = 0;

  for(unsigned int i: activeNodes){
    maxTransmitTime_us = std::max(maxTransmitTime_us, node[i].transmitTime_us);
    maxResponseTime_us = std::max(maxResponseTime_us, node[i].responseTime_us);
    sumReceiveTime_us += node[i].receiveTime_us;
  }

  systemTimes.transmitT_us = maxTransmitTime_us * (100 + TIME_OFFSET_PERCENTAGE)/100;
  systemTimes.responseT_us = maxResponseTime_us * (100 + TIME_OFFSET_PERCENTAGE)/100;
  systemTimes.receiveT_us = sumReceiveTime_us * (100 + TIME_OFFSET_PERCENTAGE)/100;
  
  systemTimes.lowT_us = systemTimes.receiveT_us + systemTimes.processT_us + systemTimes.transmitT_us;
  systemTimes.highT_us = std::max(systemTimes.responseT_us, systemTimes.printT_us);

  if(systemTimes.lowT_us < MIN_PULSE_TIME_US){
     systemTimes.samplingT_us = MIN_PULSE_TIME_US + systemTimes.highT_us;
  }
  else {
    systemTimes.samplingT_us = systemTimes.lowT_us + systemTimes.highT_us;
  }

  if(debug){
    Serial.print(F("[DEBUG] lowTime: "));
    Serial.println(systemTimes.lowT_us);
    Serial.print(F("[DEBUG] highTime: "));
    Serial.println(systemTimes.highT_us);
  }
}

void RTMChub::waitForStartCommand(){
  char inputBuffer[16] = {0};
  int numReceivedBytes = -1;
  bool validInput = false;

  while(1) {
    while(!Serial.available() && !irqHandle.startButton); // wait for user input 

    numReceivedBytes = recvWithEndMarker(inputBuffer, sizeof(inputBuffer)); // read user input
    if(numReceivedBytes == 0 || irqHandle.startButton){  // "ENTER" or Startbutton
      irqHandle.startButton = false;
      break;  // break the while loop to proceed
    }
    else if(numReceivedBytes > 0){  // user has provided some input
      switch(inputBuffer[0]){
        case 't': case 'T':
          if(atof(inputBuffer+1) > 0){
            // expand the highTime / lowTimes equally, keeping the original ratio
            float ratio = (float)systemTimes.highT_us / (float)systemTimes.samplingT_us;
            systemTimes.samplingT_us = 1000000 * atof(inputBuffer+1); 
            systemTimes.highT_us = ratio * systemTimes.samplingT_us;
            validInput = true;
          }          
          else{
            validInput = false;
          }
          break;
        default:
          validInput = false;
          break;
      }
      if(validInput){ // check for valid input
        break; // break the while-loop to proceed
      }
    }
  }
}

void RTMChub::checkForStopCommand(){
  char inputBuffer[16] = {0};

  if(Serial.available() || irqHandle.startButton){
    recvWithEndMarker(inputBuffer, sizeof(inputBuffer)); // read user input (empty the receive buffer)
    if(inputBuffer[0] == 0 || irqHandle.startButton){
      irqHandle.startButton = false;
      digitalWrite(START_LED_PIN, HIGH); //START_LED_OFF;
      trigger.end();
      receiver.end();
      printData();
      
      TXenable();
      sendCommand(COMMAND_STOP);
      Serial.println("Measurement finished.\r\n");
      delay(100); // give the nodes some time to process the command
      start();
    }
  }
}

void RTMChub::restart(){
  // Magic number to restart the Teensy. Info from the prjc.com forum.
  SCB_AIRCR = 0x05FA0004; 
}

void RTMChub::receiveData(){
  RTMCcore::receiveData();
}

void RTMChub::transmitData(){
  RTMCcore::transmitData();
}

void RTMChub::printData(){
  print.data();
}


#endif