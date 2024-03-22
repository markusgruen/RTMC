/*
This is the library that contains the RTMChubs's printing functions.

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
#ifndef RTMCHUB_PRINT_H
#define RTMCHUB_PRINT_H

#include "RTMChub/RTMChub_core.h"
#include <RTClib.h>  // for "DateTime"

/*!
 * @brief class with RTMChub's printing functionalities
 */
class RTMCprint : public RTMCcore{
  public:
    /*!
     * @brief Prints all node's response times, output- and 
     * input infos in form of a table and prints the sampling
     * time.
     * @param samplingTime: the sampling time in [us] to be printed.
     */
    void info(long samplingTime);

    /*!
     * @brief Prints software's version
     */
    void version(float version);

    /*!
     * @brief Prints all node's output names. These make up the
     * column names of the following data. 
     */
    void header();

    /*!
     * @brief Prints the sampling time
     * @param The sampling time in [us] to be printed.
     */
    void samplingTime(long samplingTime);

    /*!
     * @brief Prints the current time from the
     * real-time-clock.
     */
    void startTime(DateTime now);

    /*!
     * @brief Prints all data of the current measurement cycle 
     * to the host-Serial. Depending on the CSV-settings, the
     * decimal-point is replaced with the character specified.
     */ 
    void data();
};


#endif
#endif