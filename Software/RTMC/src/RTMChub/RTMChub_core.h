/*
This is the library for the RTMChub's core functions. It contains a the 
"core"-class with the basic functions to send and receive data, messages 
and commands. It also contains the RTMChub's node-class.
 
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
#ifndef RTMCHUB_CORE_H
#define RTMCHUB_CORE_H

#include <Arduino.h>      // for "uint8_t",...
#include "common/node.h"  // for "BaseNode"
#include <vector>

#define NUM_NODES   8  // max. number of nodes 
#define WAIT_RXTX   5  // [µs]  wait this long after pulling the trigger-signal low to send a command over the RS485 and after sending before pulling the trigger high again


// enum that holds all system-times.
struct Times{unsigned long samplingT_us;
             unsigned long highT_us;
             unsigned long lowT_us;
             unsigned long responseT_us;
             unsigned long receiveT_us;
             unsigned long processT_us;
             unsigned long transmitT_us;
             unsigned long printT_us;
};

// enum that holds the RTMChub's modes that can be set by the rotary switch
enum Mode{SERIAL_GERMAN, SERIAL_ENGLISH, SD_GERMAN, SD_ENGLISH, SERIAL_MONITOR};

// struct that holds settings
struct Settings{
	char separator;
  char decimalPoint;
  char nanString[5];
  Mode mode;
};


/*!
 * @brief RTMChub's node class
 */
class Node : public BaseNode{
  public:
    Node(HardwareSerial* port) : BaseNode(port){};

    unsigned long responseTime_us = 0;  // [µs]
    unsigned long transmitTime_us = 0;  // [µs]
    unsigned long receiveTime_us = 0;   // [µs]
    bool resetting = false;
};
extern Node node[NUM_NODES];


/*!
 * @brief class with RTMChub's core functionalities
 */
class RTMCcore{
  public:
   /*!
   * @brief Sends a command byte to all nodes and waits
   * until the command has actually been sent
   * @param _command The command character
   */
    void sendCommand(char _command);

    /*!
    * @brief Sends a command byte to one specific node
    * waits until the command has actually been sent
    * @param _node The node the command will be sent to
    * @param _command The command character
    */
    void sendCommand(Node _node, char _command);

    /*!
    * @brief Sends a command byte and one user defnied
    * byte to one specific node
    * waits until the command has actually been sent
    * @param _node The node the command will be sent to
    * @param _command The command character
    * @param _byte The user defined byte
    */
    void sendCommand(Node _node, char _command, uint8_t _byte);

    /*!
    * @brief For all nodes: reads the serial receive buffers 
    * and parses the received bytes, but only if at least 
    * the expected number of bytes have been received. 
    * If too few bytes have been received, the receive buffer
    * will be left untouched.
    * Also, sets the "nodeReset-Flag" for resetting the nodes 
    * if no bytes have been received too often.
    * Changes the state-machines state to "STATE_DATA_READ".
    */
    void receiveData();

    /*!
    * @brief For all nodes: transmits the data from the node's
    * transmit-buffer.
    * Changes the state-machines state to "IDLE"
    */
    void transmitData();

    /*!
    * @brief Transmits the data from the node's transmit buffer
    * @param _node The node to which data will be transmitted
    */
    void transmitData(Node _node);

    /*!
     * @brief For all nodes: resets/clears the serial receive
     * buffers.
     */
    void resetBuffers();

    /*!
     * @brief Removes all nodes from the list of "activeNodes",
     * who's value "node[i].dataValid" is "false". 
     */
    void removeInvalidNodes(bool debug);

    // mit "static" ist der Inhalt von der Variablen für alle abgeleiteten Klassen gleich!
    static Times systemTimes;
    static Settings settings;
    static std::vector<int> activeNodes;
};



#endif
#endif