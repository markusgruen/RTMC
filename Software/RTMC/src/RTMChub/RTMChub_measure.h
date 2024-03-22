/*
This is the library that contains the RTMChubs's functions to measure the 
node's and hub's response and processing times to determine the minimum sample
time.
 
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
#ifndef RTMCHUB_MEASURE_H
#define RTMCHUB_MEASURE_H


#include "RTMChub/RTMChub_core.h"
#include <Arduino.h>  // for "uint8_t",...

#define MEASURE_CYCLES    10  // how often the node's sensors is read out to determine the node's response time


/*!
 * @brief class with RTMChub's measuring functionalities,
 * that are used to determine the minimum sampling time
 */
class RTMCmeasure : public RTMCcore{
  public:
    RTMCmeasure(){};

    /*!
     * @brief activates the debug-output
     */ 
    void setDebug(bool _debug){
      debug = _debug;
    };

    /*!
     * @brief For all nodes: measures MEASURE_CYCLES
     * times the:
     * "transmit time" (time to transmit all 
     *                  "input"-values via Serial)
     * "response time" (time the node requires to 
     *                  measure and transmit the
     *                  data)
     * "receive time" (time to read the serial's 
     *                 receive buffer and to 
     *                 parse the data)
     * takes the max or average from these measurements
     * and saves the data to the node's variables.
     */
    void nodes();

    /*!
     * @brief Measures the time required to execute
     * the user's "processData()" function. And saves
     * the data to the system's variables
     */
    void processTime();

    /*!
     * @brief If output is to the host-PC via Serial,
     * the time required to transmit all "print"-data
     * over serial is estimated.
     * If output is to the SD-card, TODO 
     * NOTHING IS DONE HERE, BECAUSE THIS IS NOT YET IMPLEMENTED
     */
    void printTime();

  private:
    long measureTransmitTime_us(uint8_t i);
    long measureResponseTime_us(uint8_t i);
    long measureReceiveTime_us(uint8_t i);
    bool debug;
};


#endif
#endif