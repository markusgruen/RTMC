/*
This is the library for the RTMC-node. It is designed specifically to work 
with the RTMC-node's hardware, which runs on a Raspberry Pi pico
See https://github.com/markusgruen/RTMC

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

#ifndef RTMCNODE_H
#define RTMCNODE_H

#include "common/node.h"  // for "BaseNode"
#include "RTMCnode/RTMCnode_main.h" // for "state", "mode", "loop()"" and other function declarations


#ifndef ARDUINO_ARCH_RP2040
  #error "Make sure to select 'Raspberry Pi Pico' from Tools/Board"
#else


#define RESET_WATCHDOG false
#define RESET_RUN_PIN  true
#define MAX_FAIL_COUNT  5  // TODO WAS IST DAS??


class Node : public BaseNode{ 
  public:
    Node(const char *_name);
    
    void begin();
    void start();
    void receive();
    void transmitData();
    void reset();  

  private:
    void sendName();
    void sendOutputInfo(uint8_t idx);  
    void sendInputInfo(uint8_t idx);
    void sendErrorMessage();
    void receiveData();
    bool receiveCommand();
    void clearResetReason();

    long serialStartTime_ms = 0;
    long receiveTimeout_us = 0;
    bool hasSerial = false;
    uint8_t inputDataStream[MAX_MESSAGE_LENGTH];  // TODO
    uint8_t inputDataCnt; // TODO
};
extern Node node;


#endif
#endif
