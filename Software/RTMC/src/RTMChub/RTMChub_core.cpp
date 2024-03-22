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
#include "RTMChub/RTMChub_core.h"
#include "RTMChub/RTMChub_main.h"  // for "state"
#include <algorithm>  // for "find"


// init the static variables
Times RTMCcore::systemTimes = {0, 0, 0, 0, 0, 0, 0, 0};
Settings RTMCcore::settings;
std::vector<int> RTMCcore::activeNodes;


void RTMCcore::sendCommand(char _command){
  delayMicroseconds(WAIT_RXTX);

  for(int i: activeNodes){
    node[i].sendMessage(&_command,1);
  }
  for(int i: activeNodes){
    node[i].flush(); // wait for data to be sent
  }    

  delayMicroseconds(WAIT_RXTX);
}

void RTMCcore::sendCommand(Node _node, char _command){
  delayMicroseconds(WAIT_RXTX);
  _node.sendMessage(&_command,1);
  _node.flush();
  delayMicroseconds(WAIT_RXTX);
}

void RTMCcore::sendCommand(Node _node, char command, uint8_t _byte){
  char message[3];
  message[0] = command;
  message[1] = _byte;
  message[2] = '\0';

  delayMicroseconds(WAIT_RXTX);
  _node.sendMessage(message, 2);
  _node.flush();
  delayMicroseconds(WAIT_RXTX);
}

void RTMCcore::receiveData(){
  int8_t numBytesReceived = 0;
  uint8_t numBytesProcessed = 0;
  char dataStream[MAX_MESSAGE_LENGTH];

  for(unsigned int i: activeNodes) {
    // read data from serial-buffer only if there is enough data available
    if(node[i].available() >= node[i].numReceiveBytes){ 
      node[i].resetting = false;

      // TODO Node's variante mit ohne timeout hier einfügen!
      numBytesReceived = node[i].recvData_wait(dataStream, node[i].numReceiveBytes, 2);  // timeout must be >1 otherwise the while-loop won't start reliably
      if(numBytesReceived > 0) {
        numBytesProcessed = 0;
        for(uint8_t k=0; k<node[i].numOutputs; k++){
          memcpy(node[i].output[k].data, dataStream+numBytesProcessed, sizeof(node[i].output[k].data));
          numBytesProcessed += sizeof(node[i].output[k].data);
        }
        memset(dataStream, 0, MAX_MESSAGE_LENGTH);
        node[i].dataValid = true;
        node[i].failCnt = 0;
      }
      else{
        node[i].dataValid = false;
        node[i].failCnt++;
      }
    }
    else if(!node[i].resetting){
      node[i].resetting = false;
      node[i].dataValid = false;      
      node[i].failCnt++;
    }

    if(node[i].failCnt >= node[i].maxFailCnt){
      sendCommand(node[i], COMMAND_RESET);
      node[i].resetting = true;
      node[i].failCnt = 0;
      #ifdef __RTMC_DEBUG__
        Serial.print(F("[DEBUG] Resetting node "));
        Serial.println(i);
        #endif
    }
  }
  state = STATE_DATA_READ;
}

void RTMCcore::transmitData(){
  for(uint8_t i: activeNodes) {
    transmitData(node[i]);
  }
  
  state = STATE_IDLE;
}

void RTMCcore::transmitData(Node _node){
  // Ich gehe davon aus, dass MAX_MESSAGE_LENGTH-1 immer groß genug ist: 63 > 8*4 <-- passt!
  // Dies ist die schnellste Methode, "dataStream" zu beschreiben (getestet gegen über start = sprintf(start... wie hier beschrieben: https://www.mikrocontroller.net/topic/93067 
  char dataStream[MAX_MESSAGE_LENGTH] = {0};
  int numBytesProcessed = 0;
  for(unsigned int j=0; j < _node.numInputs; j++) {
    for(uint8_t k=0; k<sizeof(_node.input[j].data); k++){
      dataStream[numBytesProcessed++] = _node.input[j].data[k];
    }
  }
  _node.sendData(dataStream, numBytesProcessed);
}

void RTMCcore::resetBuffers(){
  for(int i=0; i<NUM_NODES; i++){
    node[i].resetBuffer();
  }
}

void RTMCcore::removeInvalidNodes(bool debug){
  for(uint8_t i=0; i<NUM_NODES; i++){
    if((find(activeNodes.begin(),activeNodes.end(), i) != activeNodes.end()) && // if "i" is in active Nodes AND ...
        node[i].dataValid == false ){ 

        activeNodes.erase(find(activeNodes.begin(),activeNodes.end(), i));
      if(debug) {
        Serial.print("[DEBUG] removing node ");
        Serial.println(i);
      }
    }
  }
}

#endif