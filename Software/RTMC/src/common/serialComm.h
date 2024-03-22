/*
This is the RTMC's core library that contains the base class "SerialComm" 
that provides the protocol's core functions for sending and receiving data
 
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


#ifndef SERIALCOMM_H
#define SERIALCOMM_H

#include <Arduino.h>
#include <assert.h>

#define BAUDRATE_HOST 115200    // the baudrate for communicating with the PC
#define BAUDRATE_NODE 500000    // the baudrate for communication between the nodes and the hub

#define MAX_MESSAGE_LENGTH 64   // this is the Arduino's UART receive buffer size
#define NAME_LENGTH MAX_MESSAGE_LENGTH - 10    // 2 bytes for length + crc, 2 bytes for numInput/Output, 4 bytes for version, 2 bytes reserve

#define TIMEOUT_MILLIS_NODE_RESET 500 // max. allowed time for a node to start up..
#define TIMEOUT_MILLIS_NODE_INIT  4000 // max. allowed time for a node to execute the setup
#define TIMEOUT_MILLIS_NODE_RESPONSE 200 // max. allowed time for a node to respond to command
// if the "recv_message" function is called while serial data is received at the same time, this is the time that the function waits until all data has been arrived.
#define TIMEOUT_MICROS_SERIAL_RECEIVE   1000*1000*MAX_MESSAGE_LENGTH*10 / BAUDRATE_NODE

// command characters to indicate the content of the following message
#define COMMAND_MEASURE 'M'     
#define COMMAND_STOP 'S'
#define COMMAND_RESET 'R'
#define COMMAND_OUTPUT 'O'
#define COMMAND_INPUT 'I'
#define COMMAND_TEST 'T'
#define COMMAND_ERROR 'E'
#define ENDMARKER '\n'

#define ERRORCODE_NO_RESPONSE -1
#define ERRORCODE_SMALL_BUFFER -2
#define ERRORCODE_TOO_MANY_BYTES -3
#define ERRORCODE_TOO_FEW_BYTES -4
#define ERRORCODE_CHECKSUM -5
#define ERRORCODE_DTYPE_NOT_SET -6


/*!
 * @brief class for low-level serial communication between hub and node
 */
class SerialComm {
public:
  //SerialComm();

  /*!
   * @brief creates a new 'serialComm'-object
   * @param _port The Serial interface to use
   */
  SerialComm(HardwareSerial* _port);

  /*!
   * @brief Starts the serial connection
   * @param _baudrate The serial connection's baudrate
   */
  void begin(uint32_t _baudrate);

  /*!
   * @brief Sends an array of bytes over serial and adds a CRC
   * if numBytes == 0, nothing is sent.
   * @param _data The pointer to the array to be sent
   * @param _numBytes The length of the array to be sent
   */
  void sendData(char* _data, unsigned int _numBytes);

  /*!
   * @brief Composes a message from an array of bytes 
   * (length, dataStream, CRC) and sends it over serial.
   * @param _message The pointer to the array to be sent
   */
  // void sendMessage(char* _message);

  /*!
   * @brief Composes a message from an array of bytes 
   * (length, dataStream, CRC) and sends it over serial.
   * If numBytes == 0, length = 1 and CRC are sent.
   * @param _message The pointer to the array to be sent
   * @param _numBytes The length of the array to be sent
   */
  void sendMessage(char* _message, unsigned int _numBytes);

  /*!
   * @brief Parses the received data without waiting 
   * (assumes data is already in the receive buffer)
   * @param _dest The pointer to the array where the received 
   * data will be copied to
   * @param _numExpected The expected number of bytes to be received
   * @return Returns the number of bytes received excl. CRC (positive value)
   * or an errorcode (negative value)
   */
  int8_t recvData(char* _dest, uint8_t _numExpected);
  
  /*!
   * @brief Waits for data and parses the received data 
   * @param _dest The pointer to the array where the received 
   * data will be copied to
   * @param _numExpected The expected number of bytes to be received
   * @param _waitMicros The timeout, after which the function will 
   * return an error, if nothing has been received 
   * @return Returns the number of bytes received excl. CRC (positive value)
   * or an errorcode (negative value)
   */
  int8_t recvData_wait(char* _dest, uint8_t _numExpected, unsigned long _waitMicros);

  /*!
   * @brief Parses the received data without waiting
   * (assumes the message is already in the receive buffer)
   * @param _dest The pointer to the array where the received 
   * data will be copied to
   * @param _maxBytes The size of the destination array
   * @return Returns the number of bytes received excl. CRC (positive value)
   * or an errorcode (negative value)
   */
  int8_t recvMessage(char* _dest, uint8_t _maxBytes);

  /*!
   * @brief Waits for data and parses the received data 
   * @param _dest The pointer to the array where the received 
   * data will be copied to
   * @param _maxBytes The size of the destination array
   * @return Returns the number of bytes received excl. CRC (positive value)
   * or an errorcode (negative value)
   */
  int8_t recvMessage_wait(char* _dest, uint8_t _maxBytes, unsigned long _waitMillis);

  /*!
   * @brief Checks if data is in the receive buffer
   * @return Returns the number of bytes in the receive buffer
   */
  int8_t available();

  /*!
   * @brief Clears the Serial receive buffer 
   */
  void resetBuffer();

  /*!
   * @brief Waits for the transmission of outgoing data to complete
   */
  void flush();

private:
  /*!
   * @brief Checks the transmission of the received data
   * @param _data The pointer to the array to be checked
   * @param _numReceived the number of received bytes
   * @param _numExpected the expected number of bytes to be received
   * @return Returns the number of bytes received excl. CRC (positive value)
   * or an errorcode (negative value) if there has been a hickup
   */
  int8_t checkData(char* _data, uint8_t _numReceived, uint8_t _numExpected);

  /*!
   * @brief Checks the received checksum
   * @param _data The pointer to the array (which includes the CRC as the last byte)
   * @param _numBytes The length of the data array
   * @return Returns true if the checksum of data equals the CRC
   */
  bool checkCRC(const char* _data, uint8_t _numBytes);

  /*!
   * @brief Calculates the CRC8 checksum
   * @param _data The pointer to the array from which 
   * the checksum shall be calculated
   * @param _numBytes The length of the data array
   * @return Returns the CRC8 checksum
   */
  char CRC8(const char* _data, uint8_t _numBytes);

  HardwareSerial* port;
};


#endif