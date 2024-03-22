/*
This is the RTMChubs's core library that contains the initialization 
functions, which are called during startup, to initialize the hub and 
to detect which nodes are connected.

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
#ifndef RTMCHUB_INIT_H
#define RTMCHUB_INIT_H

#include "RTMChub/RTMChub_core.h"  // for "Settings"


/*!
 * @brief class with RTMChub's initialization functionalities
 */
class RTMCinit : public RTMCcore{
  public:
    RTMCinit(){};

    /*!
     * @brief activates the debug-output
     */ 
    void setDebug(bool _debug){
      debug = _debug;
    };

    /*!
     * @brief Sets the Teensy's GPIOs as required
     */ 
    void gpio();

    /*!
     * @brief Reads the rotary knob and sets the
     * CSV-settings accordingly.
     */ 
    void getMode();

    /*!
     * @brief Hardware-resets all nodes by pulling
     * the RPi's RUN-pin low
     */ 
    void resetNodes();

    /*!
     * @brief Waits 1 second and then reads all serial
     * receive buffers, which either are empty (no node
     * connected), or contain the node's name and number
     * of output and inputs. 
     * Adds all nodes with proper response to the list of
     * "activeNodes".
     */ 
    void getNodeInfo();

    /*!
     * @brief For all nodes: Sends an "ERROR"-command 
     * to all nodes to retreive their error messages 
     * (if any). If an error message is received, the 
     * node is removed from the list of "activeNodes". 
     */ 
    void getNodeError();

    /*!
     * @brief For all nodes: Sends "OUTPUT"-commands
     * to retreive and parse the node's output infos. 
     */ 
    void getOutputInfos();

    /*!
     * @brief For all nodes: Sends "INPUT"-commands
     * to retreive and parse the node's input infos. 
     */ 
    void getInputInfos();

    bool debug;
};


#endif
#endif