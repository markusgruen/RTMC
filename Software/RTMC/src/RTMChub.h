/*
This is the library for the RTMC-hub. It is designed specifically to work 
with the RTMC-hub's hardware, which runs on a Teensy 4.1
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

#ifndef RTMCHUB_H
#define RTMCHUB_H


#ifndef __IMXRT1062__
  #error "#error "Make sure to select 'Teensy 4.1' from Tools/Board""
#else 

#include "common/node.h"  // for "BaseNode"
#include "RTMChub/RTMChub_core.h" 
#include "RTMChub/RTMChub_init.h"
#include "RTMChub/RTMChub_measure.h"
#include "RTMChub/RTMChub_print.h"
#define DEBUG true   // <-- do not uncomment, this is just a macro

const unsigned long MIN_PULSE_TIME_US = 10; // [µs] additional time pulse "systemTimes.lowT"
const unsigned long TIME_OFFSET_PERCENTAGE = 2; // [%] additional time for response time


/*!
 * @brief class with RTMChub's main functionalities
 */
class RTMChub : public RTMCcore{
  public:
    RTMChub();
    RTMChub(bool _debug);

    /*!
     * @brief Sequence of functions to prepare
     * RTMC for measuring. Init the uart's, collect
     * node infos, measuring node system times,
     * printing infos.
     */
    void begin();

    /*!
     * @brief Sequence of functions to start 
     * the measurement.
     */
    void start();

    /*!
     * @brief Checks if STOP-condition is met,
     * if yes, sequence of functions to stop
     * the measurement.
     */
    void checkForStopCommand();

    /*!
     * @brief Prints all the data from the 
     * current measurement-cycle to either
     * the host's serial (or to SD-card).
     */
    void printData();

    /*!
     * @brief Reads all node's serial receive
     * buffer and parses the data.
     */
    void receiveData();

    /*!
     * @brief Transmits all node's serial
     * transmit buffer.
     */
    void transmitData();

  private:
    void calculateSampleTime();
    void waitForStartCommand();
    static void restart(); // static, because it gets attached to a IRQ
    bool debug;

    RTMCinit init;
    RTMCmeasure measure;
    RTMCprint print;
}; extern RTMChub rtmcHub;

#endif
#endif
