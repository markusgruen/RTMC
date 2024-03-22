/* MIT License

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


#include "serialComm.h"


SerialComm::SerialComm(HardwareSerial* _port){
  port = _port;
}

void SerialComm::begin(uint32_t _baudrate){
  port->begin(_baudrate);
}

int8_t SerialComm::available(){
  return port->available();
}

void SerialComm::sendData(char* _data, unsigned int _numBytes){
  if(_numBytes > 0) { // avoids sending a CRC if no data is to be sent
    port->write(_data, _numBytes);
    port->write(CRC8(_data, _numBytes));
  }
}

void SerialComm::sendMessage(char* _message, unsigned int numBytes){
  if(numBytes == 0) // enables sending an empty (error) message
  { 
    port->write(1);
    port->write((byte)0); // CRC = 0
  }
  else
  {
    port->write(numBytes+1);
    sendData(_message, numBytes);
  }
}

int8_t SerialComm::recvData(char* _dest, uint8_t _numExpected){
  // returns the number of received bytes (excluding the CRC)
  // or an Errorcode
  uint8_t numReceived = 0; 
  int8_t errorCode = 0;
  char dataStream[_numExpected] = {0};

  // read out serial buffer until empty or until expected amount of bytes
  while(port->available() > 0 && numReceived < _numExpected)
  { 
     dataStream[numReceived++] = port->read();
  }

  // Error handling
  errorCode = checkData(dataStream, numReceived, _numExpected);
  if(errorCode > 0)
  {
    memcpy(_dest, dataStream, numReceived-1);  // -1 because checksum must not be copied
  }
  
  return errorCode;
}

int8_t SerialComm::recvData_wait(char* _dest, uint8_t _numExpected, unsigned long _waitMicros){
  unsigned long startTime = micros();
  uint8_t numReceived = 0;  // the number of bytes received
  int8_t errorCode = 0;
  char dataStream[_numExpected] = {0};

  // read out serial buffer until empty or until the expected number of bytes has been read or until timeout
  while((micros() - startTime) < _waitMicros)
  {
    while(port->available() > 0 && numReceived < _numExpected)
    { 
      dataStream[numReceived++] = port->read();
    }

    // if everything is as expected, break the while-loop to proceed
    if((port->available() == 0) && (numReceived == _numExpected))
    {
      break;
    }
  }

  // Error handling
  errorCode = checkData(dataStream, numReceived, _numExpected);
  if(errorCode > 0)
  {
    memcpy(_dest, dataStream, numReceived-1);  // -1 because checksum must not be copied
  }
  
  return errorCode;
}

int8_t SerialComm::recvMessage(char* _dest, uint8_t _maxBytes){
  // returns the number of received bytes (excluding the CRC)
  // or an Errorcode

  if(port->available() == 0)
  {
    return ERRORCODE_NO_RESPONSE;
  }

  uint8_t numReceived = port->read();
  if(numReceived > _maxBytes) 
  {
    return ERRORCODE_SMALL_BUFFER;
  }
  else
  {
    return recvData_wait(_dest, numReceived, TIMEOUT_MICROS_SERIAL_RECEIVE);
  }
}

int8_t SerialComm::recvMessage_wait(char* _dest, uint8_t _maxBytes, unsigned long _waitMillis){
  unsigned long startTime = millis();  
  while(!port->available() && ((millis() - startTime) < _waitMillis))
  {
    if(port->available())  // at least the message's first byte has been received
    {
      break;
    }
  }

  return recvMessage(_dest, _maxBytes);
}

int8_t SerialComm::checkData(char* _data, uint8_t _numReceived, uint8_t _numExpected){
  if(port->available() == 0 && _numReceived == _numExpected)
  {
    if(checkCRC(_data, _numReceived))
    {
      return _numReceived-1;   // -1 because for checksum
    }
    else
    {
      return ERRORCODE_CHECKSUM;
    }
  }

  else if(port->available() == 0 && _numReceived == 0)
  {
    return ERRORCODE_NO_RESPONSE;
  }

  else if(port->available() > 0 && _numReceived == _numExpected) 
  {
    resetBuffer();
    return ERRORCODE_TOO_MANY_BYTES;
  }

  else
  {
    return ERRORCODE_TOO_FEW_BYTES;
  }
}

void SerialComm::resetBuffer(){
  while(port->available())
  {
    port->read();
  }
}

void SerialComm::flush(){
  port->flush();
}

char SerialComm::CRC8(const char* _data, uint8_t _numBytes){
  // aus den Weiten des Internet.. Muss die Quelle nochmal finden
  char crc = 0x00;
  while (_numBytes--) {
    char extract = *_data++;
    for (char tempI = 8; tempI; tempI--) {
      char sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum) {
        crc ^= 0x8C;
      }
      extract >>= 1;
    }
  }
  return crc;
}

bool SerialComm::checkCRC(const char *_data, uint8_t _numBytes) {
  return(CRC8(_data, _numBytes-1) == _data[_numBytes-1]);
}



