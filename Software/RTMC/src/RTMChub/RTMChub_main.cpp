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

#include "RTMChub/RTMChub_main.h"
#include "RTMChub/RTMChub_pinmapping.h"
#include "RTMChub.h"


void TXenable(){
  digitalWriteFast(TRIGGER_PIN, HIGH);
}

void RXenable(){
  digitalWriteFast(TRIGGER_PIN, LOW);
}

void triggerIRQ(void){
  RXenable();
  irqHandle.triggerTime_us = micros();
  if(irqHandle.firstTrigger){
    irqHandle.startTime_us = irqHandle.triggerTime_us;
    state = STATE_IDLE; // don't evoke RTMCloop_printData();
    irqHandle.firstTrigger = false;
  }
  else{
    state = STATE_TRIGGER_SET;
  }
}

void receiverIRQ(void){
  TXenable();
  irqHandle.previousTime_us = irqHandle.triggerTime_us;
  state = STATE_RECEIVE;
}

void startButtonIRQ(void){
  delay(100); // debounce really badly
  while(!digitalRead(START_BUTTON_PIN)); // wait for user to release the button
  irqHandle.startButton = true; 
}

void loop() {
  //..._____                    _____________________________________
  //        |                  |                                     |
  //        |__________________|                                     |__...
  //        ^      lowT        ^                highT                ^
  //   RECEIVE_IRQ         TRIGGER_IRQ                          RECEIVE_IRQ
  //
  // lowT:  MAIN: read data, process data, send data  // max(node.readTime) + max(node.processTime) + max(node.sendTime)
  //        NODE: -
  // highT: MAIN: print data of previous cycle        // max( max(node.responseTime), base.printTime )
  //        NODE: perform measurement / read data, process data, send data

  switch (state) {
    case STATE_TRIGGER_SET:     rtmcHub.printData(); break;
    case STATE_RECEIVE:         rtmcHub.receiveData(); break;
    case STATE_DATA_READ:       processData(); state = STATE_DATA_PROCESSED; break;
    case STATE_DATA_PROCESSED:  rtmcHub.transmitData(); break;
    case STATE_IDLE:            rtmcHub.checkForStopCommand(); break;
    default:                    rtmcHub.checkForStopCommand(); break;
  }
}


#endif