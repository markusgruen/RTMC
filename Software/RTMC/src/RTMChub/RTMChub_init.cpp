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

#ifdef __IMXRT1062__
#include "RTMChub/RTMChub_init.h"
#include "RTMChub/RTMChub_main.h"
#include "RTMChub/RTMChub_pinmapping.h"
#include "common/node.h"  // for "dTypes" in getOutputInfo and getInputInfos

#define TIMEOUT_MESSAGE "\" took too long to setup"


void RTMCinit::gpio(){
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(NODE_RESET_PIN, OUTPUT);
  pinMode(MAIN_RESET_PIN, INPUT);
  pinMode(MODE_PIN, INPUT);
  pinMode(START_BUTTON_PIN, INPUT);
  pinMode(START_LED_PIN, OUTPUT);
  pinMode(TX_EN_PIN, INPUT);

  digitalWrite(TRIGGER_PIN, LOW); // RX enable
  digitalWrite(NODE_RESET_PIN, LOW); // no reset
  digitalWrite(START_LED_PIN, HIGH); // Start Button LED off
}

void RTMCinit::getMode(){
  int value = analogRead(MODE_PIN);
 
  switch(value){
    case 599 ... 683:
      settings.mode = SERIAL_MONITOR;
      settings.separator = ',';
      settings.decimalPoint = '.';
      break;
    case 684 ... 769:
      settings.mode = SD_ENGLISH;
      settings.separator = ',';
      settings.decimalPoint = '.';
      break;
    case 770 ... 854:
      settings.mode = SD_GERMAN;
      settings.separator = ';';
      settings.decimalPoint = ',';
      break;
    case 855 ... 940:
      settings.mode = SERIAL_ENGLISH;
      settings.separator = ',';
      settings.decimalPoint = '.';
      break;
    case 941 ...1023:
      settings.mode = SERIAL_GERMAN;
      settings.separator = ';';
      settings.decimalPoint = ',';
      break;
    default: 
      Serial.println("");
      Serial.println(F("wrong mode selected. Reset required"));      
      while(1);
      break;
  }  
  if(debug)
  {
    switch(settings.mode){
      case SERIAL_GERMAN:
        Serial.println(F("[DEBUG] mode: SERIAL_GERMAN")); break;
      case SERIAL_ENGLISH:
        Serial.println(F("[DEBUG] mode: SERIAL_ENGLISH")); break;
      case SD_GERMAN:
        Serial.println(F("[DEBUG] mode: SD_GERMAN")); break;
      case SD_ENGLISH:
        Serial.println(F("[DEBUG] mode: SD_ENGLISH")); break;
      case SERIAL_MONITOR:
        Serial.println(F("[DEBUG] mode: SERIAL_MONITOR")); break;
    }
  }
}

void RTMCinit::resetNodes(){
  digitalWrite(NODE_RESET_PIN, HIGH);
  delay(200);
  digitalWrite(NODE_RESET_PIN, LOW);
}

void RTMCinit::getNodeInfo(){
  if(debug)
  {
    Serial.println(F("[DEBUG] Checking available nodes (~ 5 sec)"));
  }

  char response[MAX_MESSAGE_LENGTH] = {0};
  int8_t numBytesReceived = 0;

  RXenable();
  delay(10);
  resetBuffers();

  Serial.println("Scanning for nodes - this takes <5 seconds ------>|");
  for(int i=0; i<10; i++) 
  {
    Serial.write('.');
    delay(TIMEOUT_MILLIS_NODE_RESET/10);
  }

  if(debug) Serial.write('\n');
  for(int i=0; i<NUM_NODES; i++)
  {
    numBytesReceived = node[i].recvMessage(response, sizeof(response)); 
    
    if(debug)
    {
      Serial.print(F("[DEBUG] Node "));
      Serial.print(i);
      Serial.print(F(": numBytesReceived = "));
      Serial.print(numBytesReceived);
    }

    if(numBytesReceived > 0)
    {
      activeNodes.push_back(i); // add "i" to the list of active Nodes

      node[i].numOutputs = (uint8_t) response[0];
      node[i].numInputs = (uint8_t) response[1];
      node[i].numReceiveBytes = node[i].numOutputs*sizeof(Output::data) + 1;  // +1 for checksum
      strlcpy(node[i].name, response+2, sizeof(node[i].name));
      node[i].dataValid = true;

      if(debug)
      {
        Serial.print(F("; numOutputs = "));
        Serial.print(node[i].numOutputs);
        Serial.print(F("; numInputs = "));
        Serial.print(node[i].numInputs);
        Serial.print(F("; name = "));
        Serial.println(node[i].name);
      }
    }
    else if(numBytesReceived < -1) 
    {
      snprintf(node[i].error, sizeof(node[i].error)-1, "Node info errorcode %d", numBytesReceived);
    }
    else if(debug)
    {
        Serial.println();
    }

    // clear the response-buffer
    memset(response, 0, sizeof(response));
  }
}

void RTMCinit::getNodeError(){
  if(debug)
  {
    Serial.println(F("[DEBUG] Getting Node error messages"));
  }

  char response[MAX_MESSAGE_LENGTH] = {0};
  int8_t numBytesReceived = 0;

  for(uint8_t i: activeNodes) {
    TXenable();    
    sendCommand(node[i], COMMAND_ERROR);
    RXenable();

    // continue printing dots while waiting for the Error response
    
    for(int j=0; j<40; j++) 
    {
      if(node[i].available())
      {
        delay(1); // dirty delay to allow the complete message to arrive
        break;
      }
      if(!debug) Serial.write('.');  // to keep the debug-output tidy
      delay(TIMEOUT_MILLIS_NODE_INIT/40);
    }

    numBytesReceived = node[i].recvMessage(response, sizeof(response));
    if(debug)
    {
      Serial.print(F("[DEBUG] Node "));
      Serial.print(i);
      Serial.print(F(": numBytesReceived = "));
      Serial.print(numBytesReceived);
      Serial.print(" --- ");
      if(numBytesReceived == 0)
      {
        Serial.println(F("no error"));
      }
      else if (numBytesReceived > 0) 
      {
        Serial.println(response);
      }
    }
    if(numBytesReceived > 0) {
      strlcpy(node[i].error, response, sizeof(node[i].error));
      node[i].dataValid = false;
    }
    else if(numBytesReceived < 0) {
      if(debug)
      {
        Serial.println(" Node did not answer");
      }
      strlcpy(node[i].error, "\"", sizeof(node[i].error));
      strlcpy(node[i].error+1, node[i].name, sizeof(node[i].error)-strlen(TIMEOUT_MESSAGE)-1);
      strlcpy(node[i].error+strlen(node[i].error), TIMEOUT_MESSAGE, sizeof(node[i].error)-strlen(node[i].error));
      node[i].dataValid = false;
    }
    memset(response, 0, sizeof(response));
  }
  removeInvalidNodes(debug);
}

void RTMCinit::getOutputInfos(){
  if(debug)
  {
    Serial.println(F("[DEBUG] Getting Node output infos"));
  }

  char response[MAX_MESSAGE_LENGTH] = {0};
  int8_t numBytesReceived = 0;

  for(uint8_t i: activeNodes) {
    for(int k=0; k<node[i].numOutputs; k++){
      TXenable();    
      sendCommand(node[i], COMMAND_OUTPUT, k);
      RXenable();

      numBytesReceived = node[i].recvMessage_wait(response, sizeof(response), TIMEOUT_MILLIS_NODE_RESPONSE);

      if(debug)
      {
        Serial.print(F("[DEBUG] Node "));
        Serial.print(i);
        Serial.print(F(": numBytesReceived = "));
        Serial.print(numBytesReceived);
      }
      if(numBytesReceived > 0) {
        node[i].output[k].dType     = (dTypes) response[0];
        node[i].output[k].precision = (uint8_t) response[1];
        strlcpy(node[i].output[k].name, response+2, sizeof(node[i].output[k].name));
        if(node[i].output[k].dType == NOT_SET) {  // Sensor initialization fails. Then the device will still answer, but the output type will not be set
          node[i].dataValid = false;
          snprintf(node[i].error, sizeof(node[i].error)-1, "Node info errorcode %d", ERRORCODE_DTYPE_NOT_SET);
        }

        if(debug)
        {
          Serial.print(F("; signal = "));
          Serial.print(node[i].output[k].name);
          Serial.print(F("; dataType = "));
          Serial.print(node[i].output[k].dType);
          Serial.print(F("; precision = "));
          Serial.println(node[i].output[k].precision);
        }
      }
      else {
        if(debug)
        {
          Serial.println(" Node did not answer");
        }
        snprintf(node[i].error, sizeof(node[i].error)-1, "Node info errorcode %d", numBytesReceived);        
        node[i].dataValid = false;
        memset(response, 0, sizeof(response));
        break;  // this node will be delete, go to the next node
      }
      memset(response, 0, sizeof(response));
    }
  }
  removeInvalidNodes(debug);
}

void RTMCinit::getInputInfos(){
  if(debug)
  {
    Serial.println(F("[DEBUG] Getting Node input infos"));
  }

  char response[MAX_MESSAGE_LENGTH] = {0};
  int8_t numBytesReceived = 0;

  for(uint8_t i: activeNodes) {
    for(int k=0; k<node[i].numInputs; k++){
      TXenable();    
      sendCommand(node[i], COMMAND_INPUT, k);
      RXenable();

      numBytesReceived = node[i].recvMessage_wait(response, sizeof(response), TIMEOUT_MILLIS_NODE_RESPONSE);

      if(debug)
      {
        Serial.print(F("[DEBUG] Node "));
        Serial.print(i);
        Serial.print(F(": numBytesReceived = "));
        Serial.print(numBytesReceived);
      }
      if(numBytesReceived > 0) {
        node[i].input[k].dType = (dTypes) response[0];
        strlcpy(node[i].input[k].name, response+1, sizeof(node[i].input[k].name));
        /*switch(node[i].input[k].dType) {
          case INT:     node[i].numSendBytes += SIZEOF_INT; break;
          case FLOAT:   node[i].numSendBytes += SIZEOF_FLOAT; break;
          case NOT_SET: node[i].numSendBytes += 0; break;
          case INVALID: node[i].numSendBytes += 0; break;
          default:      node[i].numSendBytes += 0; break;
        }*/

        if(debug)
        {
          Serial.print(F("; signal = "));
          Serial.print(node[i].input[k].name);
          Serial.print(F("; dataType = "));
          Serial.println(node[i].input[k].dType);
        }
      }
      else {
        if(debug)
        {
          Serial.println(" Node did not answer");
        }
        snprintf(node[i].error, sizeof(node[i].error)-1, "Node info errorcode %d", numBytesReceived);        
        node[i].dataValid = false;
        memset(response, 0, sizeof(response));
        break;  // this node will be delete, go to the next node
      }
      memset(response, 0, sizeof(response));
    }
  }
  removeInvalidNodes(debug);
}

#endif