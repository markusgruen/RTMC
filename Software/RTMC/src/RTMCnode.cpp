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

#ifdef ARDUINO_ARCH_RP2040

#define __RTMC_DEBUG__


#include <Wire.h>  // to be able to map the SDA/SCL pins correctly via Wire.setSDA() and Wire.setSCL();
#include "RTMCnode.h"
#include "RTMCnode/RTMCnode_pinmapping.h"


constexpr float VERSION{0.4};

bool __uninitialized_ram(resetReason);  // resetReason will remain in RAM even during restart via watchdog!

volatile enum StateMachine state = STATE_IDLE;
enum Modes mode = MODE_IDLE;


Node::Node(const char* _name) : BaseNode(&Serial1){
  strlcpy(name, _name, sizeof(name));
  strlcpy(error, 0, sizeof(name));
  }

void Node::begin(){
  pinMode(TRIGGER_PIN, INPUT);
  pinMode(TX_EN_PIN, INPUT);

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);

  numReceiveBytes = numInputs*sizeof(Input::data) + 1;  // +1 for checksum
  receiveTimeout_us = 1000UL * 1000 * numReceiveBytes*10 / BAUDRATE_NODE + 20;  // [us] timeout when waiting for serial transfer of data. Assuming 10 bits per byte; +20 is for unexpected overhead
  maxFailCnt = MAX_FAIL_COUNT;

  SerialComm::begin(BAUDRATE_NODE);

  // establish Serial only when NO re-start via watchdog took place
  if(resetReason == RESET_RUN_PIN)
  {
    serialStartTime_ms = millis();
    Serial.begin(BAUDRATE_HOST);
    #ifdef __RTMC_DEBUG__
      Serial2.begin(115200);
      Serial2.print(F("Start"));
      #endif
    mode = MODE_IDLE;
    state = STATE_IDLE;
    sendName();
  }
  else // if reset via watchdog, immediatly go into Measure-mode
  { 
    resetBuffer();
    mode = MODE_RUN;
    state = STATE_IDLE;
    enableTrigger();
  }
  
  clearResetReason();
  }

void Node::start(){
  if(mode == MODE_IDLE)  // skip "node.start()" when resetting via watchdog
  {
    //while(millis()-serialStartTime_ms < TIMEOUT_MILLIS_NODE_RESET-100)
    while(millis()-serialStartTime_ms < 1000)  //number arbitrarily set by testing how long it takes to connect to Serial
    {
      if(Serial) 
      { 
        Serial.print(F("RTMCnode -- Version: "));
        Serial.println(VERSION);
        
        hasSerial = true;
        break;
      }
    }
    readSensorData();  // read data, to get the dTypes of the output channels
    state = STATE_IDLE;  // reset the state to IDLE (readData set the state to "DATA_READ")
    enableTrigger();
  }
  }

void Node::sendName(){
  char message[MAX_MESSAGE_LENGTH-1] = {0};  // -1 to leave room for the checksum
  size_t length = 0; 

  message[length++] = numOutputs;  // first byte: numOutputs
  message[length++] = numInputs;   // second byte: numInputs (may be '0')
  
  // from the 3rd byte on, add the name (and leave enough space for the version)
  strlcpy(message+length, name, sizeof(message)-length-sizeof(VERSION));

  // due to strLcpy, the name is always Null-terminated --> strlen() will work as expected
  length += strlen(message+length) + 1;  // +1 for the Null-Terminator

  // now add the version (there should always be sufficient memory)
  memcpy(message+length, &VERSION, sizeof(VERSION));
  length += sizeof(VERSION);

  while(digitalReadFast(TRIGGER_PIN) == 0);  // wait until the Master releases the Trigger;
  sendMessage(message, length);
  }

void Node::sendOutputInfo(uint8_t i){
  char message[MAX_MESSAGE_LENGTH-1] = {0};  // -1 to leave room for the checksum
  size_t length = 0;

  message[length++] = output[i].dType;
  message[length++] = output[i].precision;
  strlcpy(message+length, output[i].name, sizeof(message)-length);
  length += strlen(message+length) + 1;  // +1 für den Null-Terminator

  while(digitalReadFast(TRIGGER_PIN) == 0);  // wait until the Master releases the Trigger;
  sendMessage(message, length); 
  }

void Node::sendInputInfo(uint8_t i){
  char message[MAX_MESSAGE_LENGTH-1] = {0};  // -1 to leave room for the checksum
  size_t length = 0;

  message[length++] = input[i].dType;
  strlcpy(message+length, input[i].name, sizeof(message)-length);
  length += strlen(message+length) + 1;  // +1 für den Null-Terminator

  while(digitalReadFast(TRIGGER_PIN) == 0);  // wait until the Master releases the Trigger
  sendMessage(message, length);
  }

void Node::sendErrorMessage(){
  while(digitalReadFast(TRIGGER_PIN) == 0);  // wait until the Master releases the Trigger
  sendMessage(node.error, strlen(node.error));
  }

void Node::transmitData(){
  // Ich gehe davon aus, dass MAX_MESSAGE_LENGTH-1 immer groß genug ist: 63 > 8*4 <-- passt!
  // Dies ist die schnellste Methode, "dataStream" zu beschreiben (getestet gegen über start = sprintf(start... wie hier beschrieben: https://www.mikrocontroller.net/topic/93067 
  char dataStream[MAX_MESSAGE_LENGTH-1] = {0}; // -1 for Checksum
  int numBytes = 0;

  for(int i=0; i < numOutputs; i++)
  {
    for(uint8_t k=0; k<sizeof(output[i].data); k++){
      dataStream[numBytes++] = output[i].data[k];
    }
  }
  while(digitalReadFast(TRIGGER_PIN) == 0);  // wait until the Master releases the Trigger;
  sendData(dataStream, numBytes);
  
  enableTrigger(); // re-enable the trigger (it got disabled in the triggerIRQ)
  state = STATE_IDLE;  
  }

void Node::receive(){ 
  // node without inputs
  if(numInputs == 0)
  {
    if(available() > 0)  // if a command has been received
    {
      receiveCommand();
      state = STATE_IDLE;
    }
    else
    {
      state = STATE_DATA_PROCESSED;
    }
  }

  // node with inputs
  if(numInputs > 0)
  {
    if(available() >= numReceiveBytes)
    {
      receiveData();
      state = STATE_DATA_RECEIVED;
    }
    else if(available() == 3 || available() == 4)
    {
      if(receiveCommand()) 
      {
        state = STATE_IDLE;
      }
      else
      {
        failCnt++;
        state = STATE_DATA_RECEIVED;
      }
    }
    else 
    {
      failCnt++;
      state = STATE_DATA_RECEIVED;
    }

    // evaluate failCnt
    if(mode == MODE_RUN && failCnt >= maxFailCnt){
      state = STATE_FAILSAFE;
    }
  }
  }

bool Node::receiveCommand(){
  char messageBuffer[MAX_MESSAGE_LENGTH] = {0};
  memset(messageBuffer, 0, sizeof(messageBuffer));

  int8_t messageLength = recvMessage(messageBuffer, sizeof(messageBuffer));
  if(messageLength == 1)
  { 
    switch(messageBuffer[0]) 
    {
      case COMMAND_MEASURE: 
        enableTrigger();
        mode = MODE_RUN;
        failCnt = 0;
        #ifdef __RTMC_DEBUG__
          Serial2.write(COMMAND_MEASURE);
          #endif
        return true;
 
      case COMMAND_STOP:
        disableTrigger();
        mode = MODE_IDLE;
        failCnt = 0;
        #ifdef __RTMC_DEBUG__
          Serial2.write(COMMAND_STOP);
          #endif
        return true;

      case COMMAND_TEST:
        #ifdef __RTMC_DEBUG__
          Serial2.write(COMMAND_TEST);
          #endif 
        enableTrigger();  
        mode = MODE_TEST; 
        resetBuffer();
        failCnt = 0;
        
        return true;

      case COMMAND_ERROR:
        sendErrorMessage();
        failCnt = 0;
        #ifdef __RTMC_DEBUG__
          Serial2.write(COMMAND_ERROR);
          #endif
        return true;

      case COMMAND_RESET:   
        reset();
        return true;
      
      default: 
        return false;
    }
  }

  else if(messageLength == 2)  // Input / Output command
  {
    switch(messageBuffer[0]) 
    {
      case COMMAND_OUTPUT:
        sendOutputInfo(messageBuffer[1]); 
        failCnt = 0;
        #ifdef __RTMC_DEBUG__
          Serial2.write(COMMAND_OUTPUT);
          #endif
        return true;

      case COMMAND_INPUT:  
        sendInputInfo(messageBuffer[1]);  
        failCnt = 0;
        #ifdef __RTMC_DEBUG__
          Serial2.write(COMMAND_INPUT);
          #endif
        return true;

      default:
        return false;
    }
  }

  // return false if no valid command has been received
  return false;
  }

void Node::receiveData(){
  char dataBuffer[MAX_MESSAGE_LENGTH] = {0};
  memset(dataBuffer, 0, sizeof(dataBuffer));
  
  int count = 0;
  int8_t messageLength = recvData(dataBuffer, numReceiveBytes);
  if(messageLength > 0) 
  {
    for(uint8_t k=0; k<numInputs; k++)
    {
      memcpy(input[k].data, dataBuffer+count, sizeof(input[k].data));
      count += sizeof(input[k].data);
    }   
    dataValid = true;
    failCnt = 0;
  }
  else
  {
    dataValid = false;
    failCnt++;
  }
  }

void Node::reset(){
  resetReason = RESET_WATCHDOG;
  // https://raspberrypi.stackexchange.com/questions/132439/pi-pico-software-reset-using-the-c-sdk
  watchdog_enable(1, 1);  // getestet, funktioniert
  while(1);
}

void Node::clearResetReason(){
  resetReason = RESET_RUN_PIN;
}


void loop() {
  // DO NOT MODIFY OR ADD TO "loop()"
  if(mode == MODE_IDLE){
    if(node.available() >= 3)   // 1 byte length, 1 byte command, 1 byte CRC
    {
      node.receive();
    }
  }
  else
  {
    switch(state)
    {
      case STATE_TRIGGERED:       readSensorData(); state = STATE_DATA_READ; break;
      case STATE_DATA_READ:       node.receive(); break;
      case STATE_DATA_RECEIVED:   processInputData(); state = STATE_DATA_PROCESSED; break; 
      case STATE_DATA_PROCESSED:  node.transmitData(); break;
      case STATE_IDLE:            break;
      case STATE_FAILSAFE:        failsafe(); state = STATE_IDLE; break;
      default:                    break;
    }
  } 
  RTMCloop();
  // NO, REALLY, DON'T ADD ANYTHING HERE!
}

#endif