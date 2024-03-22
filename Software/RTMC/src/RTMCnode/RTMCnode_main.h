/*
This is the RTMCnode's library that contains the functions that are associated 
with what otherwise would be contained in the main program. It contains the 
interrupt routine for the trigger-impulse, functions to enable and disable the 
interrupt and the enums for the state machine. It also contains declarations
for the functions located in the ino-file.

MIT License

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

#ifdef ARDUINO_ARCH_RP2040
#ifndef RTMCNODE_MAIN_H
#define RTMCNODE_MAIN_H

enum StateMachine{STATE_IDLE,
                  STATE_TRIGGERED,
                  STATE_DATA_RECEIVED,
                  STATE_DATA_PROCESSED,
                  STATE_DATA_READ,
                  STATE_FAILSAFE,
                  };
extern volatile StateMachine state;

enum Modes{MODE_IDLE,
           MODE_TEST,
           MODE_RUN,
           };
extern Modes mode;


void triggerIRQ();
void enableTrigger();
void disableTrigger();

void readSensorData();
void processInputData();
void RTMCloop();
void failsafe();


#endif
#endif