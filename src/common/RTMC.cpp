#include "RTMC.h"


RTMC::RTMC(){};
RTMC::RTMC(HardwareSerial* _port){
  port = _port;
  };

void RTMC::begin(uint32_t _baudrate){
  port->begin(_baudrate);
  }

int8_t RTMC::available(){
  return port->available();
  }

void RTMC::sendData(char* _dataStream, unsigned int _numBytes){
  if(_numBytes > 0) { // avoids sending a CRC if no data is to be sent
    port->write(_dataStream, _numBytes);
    port->write(CRC8(_dataStream, _numBytes));
  }
  }

void RTMC::sendMessage(char* _message){
  unsigned int numBytes = strlen(_message);
  port->write(numBytes+1);  // +1 for checksum
  sendData(_message, numBytes);
  }

void RTMC::sendMessage(char* _message, unsigned int numBytes){
  if(numBytes == 0){ // enables sending an empty (error) message
    port->write(1);
    port->write((byte)0);
  }
  else{
    port->write(numBytes+1);
    sendData(_message, numBytes);
  }
  }

int8_t RTMC::recvMessage(char* _dest, uint8_t _maxBytes){
  // returns the number of received bytes (excluding the CRC)
  // or an Errorcode

  if(port->available() == 0){
    return ERRORCODE_NO_RESPONSE;
  }

  uint8_t numBytes = port->read();
  if(numBytes > _maxBytes) {
    return ERRORCODE_SMALL_BUFFER;
  }
  else {
    return recvData(_dest, numBytes, TIMEOUT_SERIAL_RECEIVE);
  }
  }

int8_t RTMC::recvMessage(char* _dest, uint8_t _maxBytes, unsigned long _wait){
  // returns the number of received bytes (excluding the CRC)
  // or an Errorcode
  
  unsigned long startTime = millis();  
  while(!port->available() && (millis() - startTime) < _wait) {
    if(port->available()) {
      break;
    }
  }

  return recvMessage(_dest, _maxBytes);
  }

int8_t RTMC::recvData(char* _dest, uint8_t _numBytes, unsigned long _timeout){
  // returns the number of received bytes (excluding the CRC)
  // or an Errorcode

  unsigned long startTime = micros();
  uint8_t cnt = 0;  // the number of bytes received
  int8_t errorCode = 0;
  char message[_numBytes] = {0};

  // read out serial buffer until empty or until the expected number of bytes has been read or until timeout
  while((micros() - startTime) < _timeout) {
    while(port->available() > 0 && cnt < _numBytes) { 
      message[cnt++] = port->read();
    }

    // if everything is as expected, break the while-loop to proceed
    if(port->available() == 0 && cnt == _numBytes) {
      break;
    }
  }

  // Error handling
  errorCode = recvError( message, cnt, _numBytes);
  if(errorCode > 0) {
    memcpy(_dest, message, cnt-1);  // -1 because checksum must not be copied
  }
  
  return errorCode;
  }

int8_t RTMC::recvData(char* _dest, uint8_t _numBytes){
  // returns the number of received bytes (excluding the CRC)
  // or an Errorcode
  uint8_t cnt = 0;  // the number of bytes received
  int8_t errorCode = 0;
  char message[_numBytes] = {0};

  // read out serial buffer until empty or until expected amount of bytes
  while(port->available() > 0 && cnt < _numBytes) { 
     message[cnt++] = port->read();
  }

  // Error handling
  errorCode = recvError( message, cnt, _numBytes);
  if(errorCode > 0) {
    memcpy(_dest, message, cnt-1);  // -1 because checksum must not be copied
  }
  
  return errorCode;
  }

int8_t RTMC::recvError(char* _message, uint8_t _cnt, uint8_t _numBytes){
  if(port->available() == 0 && _cnt == _numBytes){
    if(checkCRC(_message, _cnt)) {
      return _cnt-1;   // -1 because for checksum
    }
    else{
      return ERRORCODE_CHECKSUM;
    }
  }
  else if(port->available() == 0 && _cnt == 0){
    return ERRORCODE_NO_RESPONSE;
  }
  else if(port->available() > 0 && _cnt == _numBytes) {
    resetBuffer();
    return ERRORCODE_TOO_MANY_BYTES;
  }
  else{
    return ERRORCODE_TOO_FEW_BYTES;
  }
  }

void RTMC::resetBuffer(){
  while(port->available()){
    port->read();
  }
  }

void RTMC::flush(){
  port->flush();
  }

char RTMC::CRC8(const char *data, uint8_t len) {
  // aus den Weiten des Internet.. Muss die Quelle nochmal finden
  char crc = 0x00;
  while (len--) {
    char extract = *data++;
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

bool RTMC::checkCRC(const char *data, uint8_t len) {
  return(CRC8(data, len-1) == data[len-1]);
  // char crc = CRC8(data, len-1);
  // if(crc == data[len-1]) {
  //   return true;
  // }
  // else{
  //   return false;
  // }
}

void BaseNode::addOutput(const char* _name){
  addOutput(_name, 2);
  }
void BaseNode::addOutput(const char* _name, uint8_t _precision){
  if(numOutputs < MAX_OUTPUTS){
    strlcpy(output[numOutputs].name, _name, sizeof(output[numOutputs].name));
    output[numOutputs].precision = _precision;
    numOutputs++;
  }
  else{
    //Serial.print("ERROR: too many outputs - max. number of outputs = ");
    //Serial.println(MAX_OUTPUTS);
    strlcpy(error, "too many outputs - max.number of outputs = 8", sizeof(error));
  }
  }
  
void BaseNode::addInput(const char* _name, dTypes _dType){
  if(numInputs < MAX_INPUTS){
    strlcpy(input[numInputs].name, _name, sizeof(input[numInputs].name));
    input[numInputs].dType = _dType;
    numInputs++;
  }
  else{
    //Serial.print("ERROR: too many inputs - max. number of inputs = ");
    //Serial.println(MAX_INPUTS);
    strlcpy(error, "too many inputs - max.number of inputs = 7", sizeof(error));
  }
}
void BaseNode::addInput(const char* _name, String _typeString){
  if(numInputs < MAX_INPUTS){
    String typeString = _typeString;
    typeString.toUpperCase();

    strlcpy(input[numInputs].name, _name, sizeof(input[numInputs].name));
    if(typeString == "INT")
      input[numInputs].dType = INT;
    else if(typeString == "FLOAT")
      input[numInputs].dType = FLOAT;
    else
      strlcpy(error, "input value type not supported", sizeof(error));
    
    numInputs++;
  }
  else{
    //Serial.print("ERROR: too many inputs - max. number of inputs = ");
    //Serial.println(MAX_INPUTS);
    strlcpy(error, "too many inputs - max.number of inputs = 7", sizeof(error));
  }
}





