/*
This is the RTMChub's library that contains the functions that are associated 
with what otherwise would be contained in the main program. It contains the 
interrupt routines, the functions to select if the hub is sending or receiving, 
the enums for the state machine and the struct for the handling the interrupts. 
Also, the functions located in the ino-file are declared here.
 
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


#ifdef __IMXRT1062__

#ifndef RTMCHUB_MAIN_H
#define RTMCHUB_MAIN_H

#include <Arduino.h>  // for uint64_t

struct IRQhandle{uint64_t triggerTime_us;
                 uint64_t previousTime_us;
                 uint64_t startTime_us;
                 bool firstTrigger;
                 bool startButton;
};
extern volatile struct IRQhandle irqHandle;

enum SystemStates {STATE_IDLE, 
                   STATE_TRIGGER_SET, 
                   STATE_RECEIVE, 
                   STATE_DATA_READ, 
                   STATE_DATA_PROCESSED};
extern volatile enum SystemStates state;

void RXenable();
void TXenable();
void triggerIRQ(void);
void receiverIRQ(void);
void startButtonIRQ(void);

void loop();
void extern processData(void);


#endif
#endif