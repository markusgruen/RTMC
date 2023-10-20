#ifndef RTMC_H
#define RTMC_H

#include <Arduino.h>
#include <assert.h>
#include "common/templates.h"

#define BAUDRATE_HOST 115200
#define BAUDRATE_NODE 500000

#define MAX_MESSAGE_LENGTH 64  // this is the Arduino receive buffer size
#define MAX_OUTPUTS 8
#define MAX_INPUTS 7  // RPi Serial input buffer is only 31 bytes --> only up to 7 inputs (7*4+1 = 29) fit into the buffer
#define NAME_LENGTH MAX_MESSAGE_LENGTH - 4    // 2 bytes for length + crc, 1 byte for numValues, 1 byte reserve
#define HEADER_LENGTH MAX_MESSAGE_LENGTH - 4  // 2 bytes for length + crc, 1 byte for format, 1 byte reserve

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

#define TIMEOUT_SERIAL_RECEIVE 10000  // [µs]  = 10 ms
// #define TIMEOUT_NODE_RESET 2500       // [ms]
#define TIMEOUT_NODE_RESET 4000       // [ms]


class InOut {
public:
  InOut(){};
  template<typename T> void write(T value) {
    static_assert(sizeof(value) <= sizeof(data), "error: input data type too large");
    memcpy(data, &value, sizeof(value));
    if (dType == NOT_SET) {
      dType = get_type(value);
    }
  }
  int readInt() {
    int out;
    memcpy(&out, data, sizeof(data));
    return out;
  };
  float readFloat() {
    float out;
    memcpy(&out, data, sizeof(data));
    return out;
  };

  char name[NAME_LENGTH];
  uint8_t precision = 0;
  uint8_t dType = NOT_SET;
  uint8_t data[sizeof(float)];    // ACHTUNG, funktioniert nur, weil int und float beide 4 bytes groß sind!
  char error[MAX_MESSAGE_LENGTH];
};


class Output : public InOut {
public:
  Output(){};
  Output(const char* _name, uint8_t _precision) {
    strlcpy(name, _name, sizeof(name));
    precision = _precision;
  };
};

class Input : public InOut {
public:
  Input(){};
  Input(const char* _name, uint8_t _dType) {
    strlcpy(name, _name, sizeof(name));
    dType = _dType;
  };
};


class RTMC {
public:
  RTMC();
  RTMC(HardwareSerial* _port);
  void begin(uint32_t _baudrate);
  void sendData(char* _dataStream, unsigned int numBytes);
  void sendMessage(char* _message);
  void sendMessage(char* _message, unsigned int numBytes);
  // int8_t recvData(char* _dest, uint8_t _numBytes, unsigned long _timeout = TIMEOUT_SERIAL_RECEIVE);
  int8_t recvData(char* _dest, uint8_t _numBytes, unsigned long _timeout);
  int8_t recvData(char* _dest, uint8_t _numBytes);
  int8_t recvMessage(char* _dest, uint8_t _maxBytes);
  int8_t recvMessage(char* _dest, uint8_t _maxBytes, unsigned long _wait);
  int8_t available();
  void resetBuffer();
  void flush();

private:

  int8_t recvError(char* _message, uint8_t _cnt, uint8_t _numBytes);
  bool checkCRC(const char* data, uint8_t len);
  char CRC8(const char* data, uint8_t len);

  HardwareSerial* port;
};


class BaseNode : public RTMC {
public:
  BaseNode(HardwareSerial* port) : RTMC(port){};
  void addOutput(const char* _name);
  void addOutput(const char* _name, uint8_t _precision);
  void addInput(const char* _name, dTypes _dType);
  void addInput(const char* _name, String _typeString);
  char name[NAME_LENGTH];
  Output output[MAX_OUTPUTS];
  Input input[MAX_INPUTS];

  char error[MAX_MESSAGE_LENGTH] = "\0";
  uint8_t numOutputs = 0;
  uint8_t numInputs = 0;
  uint8_t numReceiveBytes = 0;
  bool dataValid = false;
  uint16_t failCnt = 0;
  uint16_t maxFailCnt = 0;
};



#endif