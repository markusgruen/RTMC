/* 
This is the RTMC's core library that contains the base class for the inputs, 
outputs and  the node. The node's base class is derived from the SerialComm 
class.

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

#ifndef NODE_H
#define NODE_H

#include "serialComm.h"

#define MAX_OUTPUTS 8  // arbitrarily set. Can be increased if needed (check Teensy UART input buffer size!)
#define MAX_INPUTS 7   // RPi Serial input buffer is only 31 bytes --> only up to 7 inputs (7*4+1 = 29) fit into the buffer


enum dTypes { INVALID = 0,
              INT,
              FLOAT,
              NOT_SET = 0xFF };

/*! 
 * set of templates to get the dType of a parameter
 */
template<typename T, typename U> struct is_same {
  enum { value = 0 };
  operator bool()
  {
    return false;
  }
};

template<typename T> struct is_same<T, T> {
  enum { value = 1 };
  operator bool()
  {
    return true;
  }
};

template<typename T> dTypes get_type(T value) {
  if (is_same<int, T>())
  {
    return INT;
  } 
  else if (is_same<float, T>())
  {
    return FLOAT;
  } 
  else
  {
  	return INVALID;
  } 
};


/*!
 * @brief class for outputs
 * with functions to read and write the data and parameters that 
 * hold the ouput's dtype and precision.
 */
class Output{
public:
  Output(){};
  /*!
   * @brief creates a new 'Output'-object
   * @param _name Name of the output to be set
   * @param _precision The output's precision (no. of decimals) to be printed 
   */
  Output(const char* _name, uint8_t _precision) {
    strlcpy(name, _name, sizeof(name));
    precision = _precision;
  };

  /*!
   * @brief writes a value to the transmit buffer
   * @param value the value to be written
   */
  template<typename T> void write(T value) {
    static_assert(sizeof(value) <= sizeof(data), "error: input data type too large");
    memcpy(data, &value, sizeof(value));
    if (dType == NOT_SET) {
      dType = get_type(value);
    }
  }

  /*!
   * @brief reads an integer from the receive buffer
   * @return Returns the received value as an integer
   */
  int readInt() {
    int out;
    memcpy(&out, data, sizeof(data));
    return out;
  };

  /*!
   * @brief reads a float-value from the receive buffer
   * @return Returns the received value as a 'float'
   */
  float readFloat() {
    float out;
    memcpy(&out, data, sizeof(data));
    return out;
  };

  char name[NAME_LENGTH];
  uint8_t data[sizeof(float)];    // ACHTUNG, funktioniert nur, weil int und float beide 4 bytes groß sind!
  uint8_t precision = 0;
  dTypes dType = NOT_SET;
};


/*!
 * @brief class for inputs
 * with functions to read and write the data
 * and parameters that hold the inputs's dtype
 */
class Input : public Output{
public:
  Input(){};
  /*!
   * @brief creates a new 'Input'-object
   * @param _name Name of the input to be set
   * @param _dType The data type of the input (int or float)
   */
  Input(const char* _name, dTypes _dType) {
    strlcpy(name, _name, sizeof(name));
    dType = _dType;
  };
};


/*!
 * @brief Base class of a RTMC-node
 * contains the basic functions to add outputs
 * and inputs and a set of parameters.
 */
class BaseNode : public SerialComm {
public:
  BaseNode(HardwareSerial* port) : SerialComm(port){};

  /*!
   * @brief adds an output
   * @param _name The name of the output to be set
   * @param _precision The output's precision (no. of decimals) to be printed 
   */
  void addOutput(const char* _name, uint8_t _precision = 2);

  /*!
   * @brief adds an input
   * @param _name The name of the input to be set
   * @param _dType The data type of the input (int or float)
   */
  void addInput(const char* _name, dTypes _dType);

  /*!
   * @brief adds an input
   * @param _name The name of the input to be set
   * @param _typeString The data type of the input ("int" or "float")
   */
  void addInput(const char* _name, String _typeString);
  
  char name[NAME_LENGTH];
  uint8_t numOutputs = 0;
  uint8_t numInputs = 0;
  Output output[MAX_OUTPUTS];
  Input input[MAX_INPUTS];

  char error[MAX_MESSAGE_LENGTH] = "\0";
  bool dataValid = false;
  uint8_t numReceiveBytes = 0;
  uint16_t failCnt = 0;
  uint16_t maxFailCnt = 0;
};


#endif
