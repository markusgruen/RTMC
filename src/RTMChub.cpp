#ifdef __IMXRT1062__

#include "RTMChub.h"
#include "util/RTMCutil.h"
#include <Arduino.h>
#include <RTClib.h>
#include <algorithm>   // std::max_element(first, last)

constexpr float VERSION{0.3};

// Real Time Clock
RTC_DS3231 rtc;
DateTime now;

// Nodes
Node node[NUM_NODES] = {&Serial1, &Serial2, &Serial3, &Serial4, &Serial5, &Serial6, &Serial7, &Serial8};
std::vector<int> activeNodes;

RTMCcore core;
RTMCinit init;
RTMCmeasure measure;
RTMCprint print;
RTMCutil util;


// Timer for trigger signal
IntervalTimer trigger;    // IntervalTimer is a built-in function by Teensy, which generates interrupt-based events
IntervalTimer receiver;

// consts
const unsigned long MIN_PULSE_TIME = 10; // [µs] additional time pulse "systemTimes.lowT"
const unsigned long TIME_OFFSET_PERCENTAGE = 2; // [%] additional time for response time

// 
volatile IRQinfo irqInfo = {0, 0, 0, true, false};  // triggerTime, previousTime, startTime, firstTrigger, startButton
volatile enum SystemStates state = STATE_IDLE;


void RTMCcore::sendCommand(char command){
  delayMicroseconds(WAIT_RXTX);

  for(int i: activeNodes){
    node[i].sendMessage(&command,1);
  }
  for(int i: activeNodes){
    node[i].flush(); // wait for data to be sent
  }    

  delayMicroseconds(WAIT_RXTX);
  }

void RTMCcore::sendCommand(Node _node, char command){
  delayMicroseconds(WAIT_RXTX);
  _node.sendMessage(&command,1);
  _node.flush();
  delayMicroseconds(WAIT_RXTX);
  }

void RTMCcore::sendCommand(Node _node, char command, uint8_t i){
  char message[3];
  message[0] = command;
  message[1] = i;
  message[2] = '\0';

  delayMicroseconds(WAIT_RXTX);
  _node.sendMessage(message, 2);
  _node.flush();
  delayMicroseconds(WAIT_RXTX);
  }

void RTMCcore::receiveData(){
  // TODO more commenting
  int8_t ret = 0;
  uint8_t cnt = 0;
  char dataStream[MAX_MESSAGE_LENGTH];

  for(unsigned int i: activeNodes) {
    // read data from serial-buffer only if there is enough data available
    if(node[i].available() >= node[i].numReceiveBytes){ 
      node[i].resetting = false;

      // TODO Node's variante mit ohne timeout hier einfügen!
      ret = node[i].recvData(dataStream, node[i].numReceiveBytes, 2);  // timeout must be >1 otherwise the while-loop won't start reliably
      if(ret > 0) {
        cnt = 0;
        for(uint8_t k=0; k<node[i].numOutputs; k++){
          memcpy(node[i].output[k].data, dataStream+cnt, sizeof(node[i].output[k].data));
          cnt += sizeof(node[i].output[k].data);
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
  /*char dataStream[NUM_NODES][MAX_MESSAGE_LENGTH] = {0};
  int cnt;
  for(unsigned int i: activeNodes) {
    cnt = 0;
    for(unsigned int j=0; j < node[i].numInputs; j++) {
      for(uint8_t k=0; k<sizeof(node[i].input[j].data); k++){
        dataStream[i][cnt++] = node[i].input[j].data[k];
      }
    }
    node[i].sendData(dataStream[i], cnt);
  }*/
  for(unsigned int i: activeNodes) {
    transmitData(node[i]);
  }
  
  state = STATE_IDLE;
  }

void RTMCcore::transmitData(Node _node){
  // Ich gehe davon aus, dass MAX_MESSAGE_LENGTH-1 immer groß genug ist: 63 > 8*4 <-- passt!
  // Dies ist die schnellste Methode, "dataStream" zu beschreiben (getestet gegen über start = sprintf(start... wie hier beschrieben: https://www.mikrocontroller.net/topic/93067 
  char dataStream[MAX_MESSAGE_LENGTH] = {0};
  int cnt = 0;
  for(unsigned int j=0; j < _node.numInputs; j++) {
    for(uint8_t k=0; k<sizeof(_node.input[j].data); k++){
      dataStream[cnt++] = _node.input[j].data[k];
    }
  }
  _node.sendData(dataStream, cnt);
  }

void RTMCcore::resetBuffers(){
  for(int i=0; i<NUM_NODES; i++){
    node[i].resetBuffer();
  }
  }

void RTMCcore::removeInvalidNodes(){
  for(uint8_t i=0; i<NUM_NODES; i++){
    if((find(activeNodes.begin(),activeNodes.end(), i) != activeNodes.end()) && // if "i" is in active Nodes AND ...
        node[i].dataValid == false ){ 

        activeNodes.erase(find(activeNodes.begin(),activeNodes.end(), i));
      #ifdef __RTMC_DEBUG__
          Serial.print("[DEBUG] removing node ");
          Serial.println(i);
          #endif
    }
  }
}

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
      mode = SERIAL_MONITOR;
      csv.separator = ',';
      csv.decimalPoint = '.';
      break;
    case 684 ... 769:
      mode = SD_ENGLISH;
      csv.separator = ',';
      csv.decimalPoint = '.';
      break;
    case 770 ... 854:
      mode = SD_GERMAN;
      csv.separator = ';';
      csv.decimalPoint = ',';
      break;
    case 855 ... 940:
      mode = SERIAL_ENGLISH;
      csv.separator = ',';
      csv.decimalPoint = '.';
      break;
    case 941 ...1023:
      mode = SERIAL_GERMAN;
      csv.separator = ';';
      csv.decimalPoint = ',';
      break;
    default: 
      Serial.println("");
      Serial.println(F("wrong mode selected. Reset required"));      
      while(1);
      break;
  }  
  #ifdef __RTMC_DEBUG__  
    switch(mode){
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
    #endif
  }

void RTMCinit::resetNodes(){
  #ifdef __RTMC_DEBUG__
    Serial.println(F("[DEBUG] Resetting nodes"));
    #endif

  digitalWrite(NODE_RESET_PIN, HIGH);
  delay(200);
  digitalWrite(NODE_RESET_PIN, LOW);
  }

void RTMCinit::getNodeInfo(){
  #ifdef __RTMC_DEBUG__
    Serial.println(F("[DEBUG] Checking available nodes (~ 5 sec)"));
    #endif

  char response[MAX_MESSAGE_LENGTH] = {0};
  int8_t ret = 0;

  RXenable();
  delay(10);
  resetBuffers();

  Serial.println("Scanning for nodes - this takes ~4 seconds ------>|");
  for(int i=0; i<50; i++) {
    Serial.write('.');
    delay(TIMEOUT_NODE_RESET/50);
  }
  Serial.write('\n');
  //util_delayAndBlinkLED(TIMEOUT_NODE_RESET, START_LED_PIN);  // give the nodes some time to start after the reset

  for(int i=0; i<NUM_NODES; i++) {
    ret = node[i].recvMessage(response, sizeof(response)); 
    
    #ifdef __RTMC_DEBUG__
      Serial.print(F("[DEBUG] Node "));
      Serial.print(i);
      Serial.print(F(": ret = "));
      Serial.print(ret);
      #endif

    if(ret > 0) {
      activeNodes.push_back(i); // add "i" to the list of active Nodes

      node[i].numOutputs = (uint8_t) response[0];
      node[i].numInputs = (uint8_t) response[1];
      node[i].numReceiveBytes = node[i].numOutputs*sizeof(InOut::data) + 1;  // +1 for checksum
      strlcpy(node[i].name, response+2, sizeof(node[i].name));
      node[i].dataValid = true;

      #ifdef __RTMC_DEBUG__
        Serial.print(F("; numOutputs = "));
        Serial.print(node[i].numOutputs);
        Serial.print(F("; numInputs = "));
        Serial.print(node[i].numInputs);
        Serial.print(F("; name = "));
        Serial.println(node[i].name);
        #endif
    }
    else if(ret < -1) {
      snprintf(node[i].error, sizeof(node[i].error)-1, "Node info errorcode %d", ret);
    }
    #ifdef __RTMC_DEBUG__
      else{Serial.println();}
      #endif

    // clear the response-buffer
    memset(response, 0, sizeof(response));
  }
  }

void RTMCinit::getNodeError(){
  #ifdef __RTMC_DEBUG__
    Serial.println(F("[DEBUG] Getting Node error messages"));
    #endif

  char response[MAX_MESSAGE_LENGTH] = {0};
  int8_t ret = 0;

  for(uint8_t i: activeNodes) {
    TXenable();    
    sendCommand(node[i], COMMAND_ERROR);
    RXenable();

    ret = node[i].recvMessage(response, sizeof(response), TIMEOUT_NODE_RESPONSE);

    #ifdef __RTMC_DEBUG__
      Serial.print(F("[DEBUG] Node "));
      Serial.print(i);
      Serial.print(F(": ret = "));
      Serial.print(ret);
      Serial.print(" --- ");
      if(ret == 0) Serial.println(F("no error"));
      else if (ret > 0) Serial.println(response);
      #endif
    if(ret > 0) {
      strlcpy(node[i].error, response, sizeof(node[i].error));
      node[i].dataValid = false;
    }
    else if(ret < 0) {
      #ifdef __RTMC_DEBUG__
        Serial.println(" Node did not answer");
        #endif
      snprintf(node[i].error, sizeof(node[i].error)-1, "Node info errorcode %d", ret);        
      node[i].dataValid = false;
    }
    memset(response, 0, sizeof(response));
  }
  removeInvalidNodes();
  }

void RTMCinit::getOutputInfos(){
  #ifdef __RTMC_DEBUG__
    Serial.println(F("[DEBUG] Getting Node output infos"));
    #endif

  char response[MAX_MESSAGE_LENGTH] = {0};
  int8_t ret = 0;

  for(uint8_t i: activeNodes) {
    for(int k=0; k<node[i].numOutputs; k++){
      TXenable();    
      sendCommand(node[i], COMMAND_OUTPUT, k);
      RXenable();

      ret = node[i].recvMessage(response, sizeof(response), TIMEOUT_NODE_RESPONSE);

      #ifdef __RTMC_DEBUG__
        Serial.print(F("[DEBUG] Node "));
        Serial.print(i);
        Serial.print(F(": ret = "));
        Serial.print(ret);
        #endif
      if(ret > 0) {
        node[i].output[k].dType     = (uint8_t) response[0];
        node[i].output[k].precision = (uint8_t) response[1];
        strlcpy(node[i].output[k].name, response+2, sizeof(node[i].output[k].name));
        if(node[i].output[k].dType == NOT_SET) {  // Sensor initialization fails. Then the device will still answer, but the output type will not be set
          node[i].dataValid = false;
          snprintf(node[i].error, sizeof(node[i].error)-1, "Node info errorcode %d", ERRORCODE_DTYPE_NOT_SET);
        }

        #ifdef __RTMC_DEBUG__
          Serial.print(F("; signal = "));
          Serial.print(node[i].output[k].name);
          Serial.print(F("; dataType = "));
          Serial.print(node[i].output[k].dType);
          Serial.print(F("; precision = "));
          Serial.println(node[i].output[k].precision);
          #endif
      }
      else {
        #ifdef __RTMC_DEBUG__
          Serial.println(" Node did not answer");
          #endif
        snprintf(node[i].error, sizeof(node[i].error)-1, "Node info errorcode %d", ret);        
        node[i].dataValid = false;
        memset(response, 0, sizeof(response));
        break;  // this node will be delete, go to the next node
      }
      memset(response, 0, sizeof(response));
    }
  }
  removeInvalidNodes();
  }

void RTMCinit::getInputInfos(){
  #ifdef __RTMC_DEBUG__
    Serial.println(F("[DEBUG] Getting Node input infos"));
    #endif

  char response[MAX_MESSAGE_LENGTH] = {0};
  int8_t ret = 0;

  for(uint8_t i: activeNodes) {
    for(int k=0; k<node[i].numInputs; k++){
      TXenable();    
      sendCommand(node[i], COMMAND_INPUT, k);
      RXenable();

      ret = node[i].recvMessage(response, sizeof(response), TIMEOUT_NODE_RESPONSE);

      #ifdef __RTMC_DEBUG__
        Serial.print(F("[DEBUG] Node "));
        Serial.print(i);
        Serial.print(F(": ret = "));
        Serial.print(ret);
        #endif
      if(ret > 0) {
        node[i].input[k].dType = (uint8_t) response[0];
        strlcpy(node[i].input[k].name, response+1, sizeof(node[i].input[k].name));
        /*switch(node[i].input[k].dType) {
          case INT:   node[i].numSendBytes += SIZEOF_INT; break;
          case FLOAT: node[i].numSendBytes += SIZEOF_FLOAT; break;
          case NOT_SET: node[i].numSendBytes += 0; break;
          case INVALID: node[i].numSendBytes += 0; break;
          default: node[i].numSendBytes += 0; break;
        }*/

        #ifdef __RTMC_DEBUG__
          Serial.print(F("; signal = "));
          Serial.print(node[i].input[k].name);
          Serial.print(F("; dataType = "));
          Serial.println(node[i].input[k].dType);
          #endif
      }
      else {
        #ifdef __RTMC_DEBUG__
          Serial.println(" Node did not answer");
          #endif
        snprintf(node[i].error, sizeof(node[i].error)-1, "Node info errorcode %d", ret);        
        node[i].dataValid = false;
        memset(response, 0, sizeof(response));
        break;  // this node will be delete, go to the next node
      }
      memset(response, 0, sizeof(response));
    }
  }
  removeInvalidNodes();
}

void RTMCmeasure::nodes(){
  long transmitTime[NUM_NODES][MEASURE_CYCLES] = {0};
  long responseTime[NUM_NODES][MEASURE_CYCLES] = {0};
  long receiveTime[NUM_NODES][MEASURE_CYCLES] = {0};
  
  #ifdef __RTMC_DEBUG__
      Serial.println(F("[DEBUG] Measuring node response times"));
      #endif

  // ensure data sent during response measurement is zero
  for(uint8_t i: activeNodes) {
    for(int k=0; k<node[i].numInputs; k++){
      node[i].input[k].write(0);
    }
  }
  TXenable();
  core.resetBuffers();

  for(unsigned int i: activeNodes) {
    #ifdef __RTMC_DEBUG__
      Serial.print(F("[DEBUG] Node: "));
      Serial.print(i);
      Serial.print(F(": "));
      #endif

    sendCommand(node[i], COMMAND_TEST); 
    delay(10);

    for(int k=0; k<MEASURE_CYCLES; k++){
      transmitTime[i][k] = measureTransmitTime(i);
      responseTime[i][k] = measureResponseTime(i);
      receiveTime[i][k] = measureReceiveTime(i);

      if(responseTime[i][k] < 0 || receiveTime[i][k] < 0){
        node[i].dataValid = false;  // flag node as invalid
        snprintf(node[i].error, sizeof(node[i].error)-1, "Node took too long to response");
      }

      #ifdef __RTMC_DEBUG__
        Serial.print(responseTime[i][k]);
        Serial.print("  ");
        #endif
    }

    TXenable();
    sendCommand(node[i], COMMAND_STOP); 
    delay(10);

    node[i].responseTime = *std::max_element(responseTime[i], responseTime[i]+MEASURE_CYCLES);  
    node[i].transmitTime = *std::max_element(transmitTime[i], transmitTime[i]+MEASURE_CYCLES);  
    node[i].receiveTime = *std::max_element(receiveTime[i], receiveTime[i]+MEASURE_CYCLES);  

  #ifdef __RTMC_DEBUG__
    Serial.println();
    Serial.print(F("[DEBUG] transmit time: "));
    Serial.print(node[i].transmitTime);
    Serial.print(F(" us, response time: "));
    Serial.print(node[i].responseTime);
    Serial.print(F(" us, receive time: "));
    Serial.print(node[i].receiveTime);
    Serial.println(F(" us"));
    #endif

  }
  removeInvalidNodes();
  }

long RTMCmeasure::measureTransmitTime(uint8_t i){
  unsigned long startMicros = micros();
  
  TXenable();
  delayMicroseconds(10);  // TODO

  transmitData(node[i]); // transmit Data
  node[i].flush();
  
  delayMicroseconds(10);   // TODO  // make sure data is received on the node's end
  return micros() - startMicros;
  }

long RTMCmeasure::measureResponseTime(uint8_t i){
  unsigned long startMicros = micros();
  unsigned long responseTime = 0;
  
  RXenable();  // rising edge on TRIGGER_PIN triggers nodes's measurement
  // wait until expected number of bytes has been received or timeout
  while((micros() - startMicros) < 1000*TIMEOUT_NODE_RESPONSE) {
    if(node[i].available() == node[i].numReceiveBytes) {
      responseTime = micros()-startMicros;
      break;
    }
  }
  if((micros() - startMicros) < 1000*TIMEOUT_NODE_RESPONSE){
    return responseTime;
  }
  else {
    #ifdef __RTMC_DEBUG__
      Serial.print(F(": Timeout"));
      #endif
    return -1;
  }
  }

long RTMCmeasure::measureReceiveTime(uint8_t i){
  int cnt = 0;
  int8_t ret = 0;
  char dummyData[MAX_MESSAGE_LENGTH];
  unsigned long startMicros = micros();
 
  ret = node[i].recvData(dummyData, node[i].numReceiveBytes, TIMEOUT_SERIAL_RECEIVE);
  if(ret == node[i].numReceiveBytes-1){  // -1, because last byte is checksum
    cnt = 0;
    for(uint8_t k=0; k<node[i].numOutputs; k++){
      memcpy(node[i].output[k].data, dummyData+cnt, sizeof(node[i].output[k].data));
      cnt += sizeof(node[i].output[k].data);
    }
    memset(dummyData, 0, sizeof(dummyData));
    return micros()-startMicros;
  }
  else{
    #ifdef __RTMC_DEBUG__
      Serial.print(F("Node "));
      Serial.print(i);
      Serial.print(F(": Timeout - ret: "));
      Serial.println(ret);
      #endif
    return -1;
  }
  }

void RTMCmeasure::processTime(){
  unsigned long startMicros = micros();
  processData();
  systemTimes.processT = micros()-startMicros;
  
  #ifdef __RTMC_DEBUG__
    Serial.print(F("[DEBUG] Measuring process time [us]: "));
    Serial.println(systemTimes.processT);
    #endif
  }


void RTMCmeasure::printTime(){
  // printing to Serial is very fast. About 0.3 us per character
  // Assumption: 10 characters for each value --> 3 us per value that gets printed out
  // this will over-estimate the required time, but printing time will
  // still be much shorter than the response time
  //
  // otherwise actually measuring the printTime will lead to an actual printout of 
  // random data to Serial. This cannot be deleted and will show up on the screen
  // and on the saved data.
  if(mode == SERIAL_GERMAN || mode == SERIAL_ENGLISH || mode == SERIAL_MONITOR) {
    uint8_t numTotalValues = 0;

    for(uint8_t i: activeNodes) {
      numTotalValues += node[i].numOutputs;
    }
    systemTimes.printT = 3*numTotalValues + 3; // +3us for printout of "time"
  }  
  else{
    // TODO
    // measure printout to SD
    systemTimes.printT = 0;
  }
}

void RTMCprint::version(){
  Serial.print(F("RTMC by Markus Grün - Version: "));
  Serial.println(VERSION, 2);
  }

void RTMCprint::info(long samplingTime){
  char infoStr[256] = {0};

  snprintf(infoStr, sizeof(infoStr)-1, "node No.%c response time [ms]%c node name%c num Outputs%c num Inputs%c value names%c ...", csv.separator, csv.separator, csv.separator, csv.separator, csv.separator, csv.separator);
  Serial.println(infoStr);

  //uint8_t k = 0;
  for(uint8_t i=0; i<NUM_NODES; i++){
    Serial.print(i);
    Serial.write(csv.separator);
    Serial.write(' ');
    if(find(activeNodes.begin(), activeNodes.end(), i) != activeNodes.end()) {  // if "i" is in activeNodes
      Serial.print(node[i].responseTime/1000.0, 4);
      Serial.write(csv.separator);
      Serial.write(' ');

      Serial.print(node[i].name); 
      Serial.write(csv.separator);
      Serial.write(' ');

      Serial.print(node[i].numOutputs);
      Serial.write(csv.separator);
      Serial.write(' ');

      Serial.print(node[i].numInputs);
      for(int j=0; j<node[i].numOutputs; j++){
        Serial.write(csv.separator);
        Serial.write(' ');
        Serial.print(node[i].output[j].name);
      }
      for(int j=0; j<node[i].numInputs; j++){
        Serial.write(csv.separator);
        switch(node[i].input[j].dType){
          case INT:   Serial.print(" int "); break;
          case FLOAT: Serial.print(" float "); break;
          default: Serial.print(" Error: only 'int' or 'float' supported ");
        }
        Serial.print(node[i].input[j].name);
      }
      Serial.println();
      //k++;
    }
    else if(node[i].error[0] != '\0') {
      Serial.print(F("ERROR: "));
      Serial.println(node[i].error);
    }
    else {
      Serial.println(F("not connected"));
    }
  }

  Serial.print(F("min. sampling time: "));
  Serial.print(samplingTime/1000000.0, 6);
  Serial.print(F(" s, or "));
  Serial.print(1000000.0/samplingTime,1);
  Serial.println(F(" Hz."));
  }


void RTMCprint::samplingTime(long samplingTime){
  Serial.print("Sampling Time [s]");
  Serial.write(csv.separator);
  Serial.write(' ');
  Serial.println(samplingTime/1000000.0, 6);
  }

void RTMCprint::startTime(){
  char dateStr[24] = {0};
  char timeStr[24] = {0};
  now = rtc.now();

  if(mode == SERIAL_GERMAN || mode == SD_GERMAN){
    snprintf(dateStr, sizeof(dateStr)-1, "%02d.%02d.%04d", now.day(), now.month(), now.year());
    snprintf(timeStr, sizeof(timeStr)-1, "%02d:%02d:%02d", now.DSThour(), now.minute(), now.second());
  }    
  else{
    snprintf(dateStr, sizeof(dateStr)-1, "%04d/%d/%d", now.year(), now.month(), now.day());
    snprintf(timeStr, sizeof(timeStr)-1, "%02d:%02d:%02d", now.DSThour(), now.minute(), now.second());
  }
  Serial.write(dateStr, strlen(dateStr));
  Serial.write(csv.separator);
  Serial.write(' ');
  Serial.write(timeStr, strlen(timeStr));
  Serial.println();
  Serial.println();}

void RTMCprint::header(){
  // 1st line: print node names
  Serial.write(csv.separator);
  Serial.write(' ');
  for(unsigned int i: activeNodes) {
    for(int k=0; k<node[i].numOutputs; k++){
      Serial.print(node[i].name);
      Serial.write(csv.separator);
      Serial.write(' ');
    }
  }
  Serial.println();

  // 2nd line: print value names
  Serial.print(F("time [s]"));
  Serial.write(csv.separator);
  Serial.write(' ');
  for(unsigned int i: activeNodes) {
    for(int k=0; k<node[i].numOutputs; k++){
      Serial.print(node[i].output[k].name);
      Serial.write(csv.separator);
      Serial.write(' ');
    }
  }
  Serial.println();
  }

void RTMCprint::data(){
  // TODO refactor
  // Serial.print(float, precision) ist die schnellste Möglichkeit auf dem teensy.
  // Alternativen wie dtostrf in einen buffer und dann sprintf(buffer)
  // oder direkt sprintf("%.3f", float) sind langsamer.
  // Insbesondere STRCAT ist langsam, aber es geht besser. Siehe: https://www.mikrocontroller.net/topic/93067 
  // Trotzdem bleibt Serial.print(float, prec) die schnellste Lösung
  //
  // Deswegen ist das Schreiben mit Punkt als Dezimal-Separator schneller als
  // wenn das Komma als Dezimal-Separator genutzt wird.

  noInterrupts();
  float time = (irqInfo.previousTime - irqInfo.startTime) / 1000000.0;
  interrupts();
  
    
  if(csv.decimalPoint == '.') {
    Serial.print(time, 5);    // die Anzahl an Nachkommastellen dynamisch von der Frequenz abhängig machen?
    Serial.write(csv.separator);
    Serial.write(' ');

    for(unsigned int i: activeNodes){
      if(node[i].dataValid){
        for(int k=0; k<node[i].numOutputs; k++){
          if(node[i].output[k].dType == INT) {  // if "int"
            Serial.print(node[i].output[k].readInt());
          }
          else {
            Serial.print(node[i].output[k].readFloat(), node[i].output[k].precision);
          }
          Serial.write(csv.separator);
          Serial.write(' ');
        }
      }
      else {
        for(int k=0; k<node[i].numOutputs; k++){
          Serial.print(csv.nanString);
          Serial.write(csv.separator);
          Serial.write(' ');
        }
      }
    }
  }


  else{
    // Vorgehen aus https://www.mikrocontroller.net/topic/93067 umgesetzt
    char out[(NUM_NODES*MAX_OUTPUTS + 1)*MAX_MESSAGE_LENGTH +1    + 10];  // +1 for time; +1 for '\n'
    char *startPtr = out;
    *startPtr = '\0';

    startPtr += sprintf(startPtr, "%.5f%c ", time, csv.separator);

    // von chagGPT vorgeschlagen. Löst das Warnungs-Problem
    if (startPtr - out >= 8) {
      out[startPtr - out - 8] = csv.decimalPoint;
    }
    //*(startPtr-8) = csv.decimalPoint;  // @Matthias: Eine Idee, wie ich die Warnung "warning: array subscript -8 is outside array bounds of 'char [4171]' [-Warray-bounds]" wegbekomme?

    for(unsigned int i: activeNodes){
      if(node[i].dataValid){
        for(int k=0; k<node[i].numOutputs; k++){
          if(node[i].output[k].dType == INT) {  // if "int"
            startPtr += sprintf(startPtr, "%d%c ",   node[i].output[k].readInt(), csv.separator);
          }
          else{
            switch(node[i].output[k].precision) {
              case 0: startPtr += sprintf(startPtr, "%.0f%c ", node[i].output[k].readFloat(), csv.separator); *(startPtr-3) = csv.decimalPoint; break;
              case 1: startPtr += sprintf(startPtr, "%.1f%c ", node[i].output[k].readFloat(), csv.separator); *(startPtr-4) = csv.decimalPoint; break;
              case 2: startPtr += sprintf(startPtr, "%.2f%c ", node[i].output[k].readFloat(), csv.separator); *(startPtr-5) = csv.decimalPoint; break;
              case 3: startPtr += sprintf(startPtr, "%.3f%c ", node[i].output[k].readFloat(), csv.separator); *(startPtr-6) = csv.decimalPoint; break;
              case 4: startPtr += sprintf(startPtr, "%.4f%c ", node[i].output[k].readFloat(), csv.separator); *(startPtr-7) = csv.decimalPoint; break;
              case 5: startPtr += sprintf(startPtr, "%.5f%c ", node[i].output[k].readFloat(), csv.separator); *(startPtr-8) = csv.decimalPoint; break;
              case 6: startPtr += sprintf(startPtr, "%.6f%c ", node[i].output[k].readFloat(), csv.separator); *(startPtr-9) = csv.decimalPoint; break;
            }
          }
        }
      }
      else {
        for(unsigned int k=0; k<node[i].numOutputs; k++){
          startPtr += sprintf(startPtr, "%s%c ", csv.nanString, csv.separator);
        }
      }
    }
    Serial.write(out);
  }
  Serial.println();
  state = STATE_IDLE;
}

RTMChub::RTMChub(){

  systemTimes = {0, 0, 0, 0, 0, 0, 0, 0};

  init.gpio();

  analogWriteResolution(15);
  analogWriteFrequency(START_LED_PIN, 4577.64);

  attachInterrupt(digitalPinToInterrupt(START_BUTTON_PIN), startButtonIRQ, FALLING);
  attachInterrupt(digitalPinToInterrupt(MAIN_RESET_PIN), restart, FALLING);

  strlcpy(csv.nanString, "NaN", sizeof(csv.nanString));
  }

void RTMChub::begin(){
  // start the RTC
  rtc.begin();

  // start all Node's UARTs
  for(int i=0; i<NUM_NODES; i++){
    node[i].begin(BAUDRATE_NODE);
  }

  Serial.begin(BAUDRATE_HOST);
  while(!Serial);

  init.getMode();
  print.version();
  
  init.resetNodes();
  init.getNodeInfo();
  init.getNodeError();

  init.getOutputInfos();
  init.getNodeError();

  init.getInputInfos();
  init.getNodeError();

  measure.nodes();
  measure.processTime();
  measure.printTime();
  calculateSampleTime();

  print.info(systemTimes.samplingT);
  }

void RTMChub::start(){
  RXenable(); // to make the green LED light up again...

  Serial.println(F("press the START button, press ENTER, or type 'Tx.xx' to start the measurement with a different sampling time [s]\r\n"));
  waitForStartCommand();

  // determine, after how many cycles a node is restarted (because it failed to answer the trigger signal)
  for(unsigned int i: activeNodes) {
    if( (1.2*node[i].responseTime / systemTimes.samplingT) < 5 ){  // TODO remove magic numbers
      node[i].maxFailCnt = 5;
    }
    else{
      node[i].maxFailCnt = 1.2*node[i].responseTime / systemTimes.samplingT;
    } 
    #ifdef __RTMC_DEBUG__
      for(unsigned int i: activeNodes) {
        Serial.print(F("[DEBUG] node "));
        Serial.print(i);
        Serial.print(F(" maxFailCnt: "));
        Serial.println(node[i].maxFailCnt);
      }
      #endif 
  }

  // switch on the START-LED
  pinMode(START_LED_PIN, OUTPUT); // must be repeated, to re-enable digitalWrite (after using analogWrite)
  // START_LED_ON;

  print.samplingTime(systemTimes.samplingT);
  print.startTime();
  print.header();
  resetBuffers();
  state = STATE_IDLE;

  // prepare all variables and start the timers for trigger and receiving
  TXenable();
  sendCommand(COMMAND_MEASURE);
  delay(20); // give the nodes some time to process the command
  noInterrupts()
    irqInfo.triggerTime = 0;
    irqInfo.previousTime = 0;
    irqInfo.startTime = 0;
    irqInfo.firstTrigger = true;
  interrupts();

  trigger.begin(triggerIRQ, systemTimes.samplingT);
  triggerIRQ(); // immediatly call the triggerIRQ, to have the first measurement right after pressing the start button
  delayMicroseconds(systemTimes.samplingT - systemTimes.lowT);  // pulse length -- not systemTimes.highT, to allow user to overwrite samplingT
  receiver.begin(receiverIRQ, systemTimes.samplingT);
  receiverIRQ(); // immediatly call the receiverIRQ, to have the first measurement right after pressing the start button
  }

void RTMChub::calculateSampleTime(){
  unsigned long maxTransmitTime = 0;
  unsigned long maxResponseTime = 0;
  unsigned long sumReceiveTime = 0;

  for(unsigned int i: activeNodes){
    maxTransmitTime = std::max(maxTransmitTime, node[i].transmitTime);
    maxResponseTime = std::max(maxResponseTime, node[i].responseTime);
    sumReceiveTime += node[i].receiveTime;
  }

  systemTimes.transmitT = maxTransmitTime * (100 + TIME_OFFSET_PERCENTAGE)/100;
  systemTimes.responseT = maxResponseTime * (100 + TIME_OFFSET_PERCENTAGE)/100;
  systemTimes.receiveT = sumReceiveTime * (100 + TIME_OFFSET_PERCENTAGE)/100;
  
  systemTimes.lowT = systemTimes.receiveT + systemTimes.processT + systemTimes.transmitT;
  systemTimes.highT = std::max(systemTimes.responseT, systemTimes.printT);

  if(systemTimes.lowT < MIN_PULSE_TIME){
     systemTimes.samplingT = MIN_PULSE_TIME + systemTimes.highT;
  }
  else {
    systemTimes.samplingT = systemTimes.lowT + systemTimes.highT;
  }

  #ifdef __RTMC_DEBUG__
    Serial.print(F("[DEBUG] lowTime: "));
    Serial.println(systemTimes.lowT);
    Serial.print(F("[DEBUG] highTime: "));
    Serial.println(systemTimes.highT);
    #endif
  }

void RTMChub::waitForStartCommand(){
  char inputBuffer[16] = {0};
  int numReceivedBytes = -1;
  bool validInput = false;

  while(1) {
    while(!Serial.available() && !irqInfo.startButton){ // wait for user input
      //util_pulseLED(START_LED_PIN);  
    }
    numReceivedBytes = util.recvWithEndMarker(inputBuffer, sizeof(inputBuffer)); // read user input
    if(numReceivedBytes == 0 || irqInfo.startButton){  // "ENTER" or Startbutton
      irqInfo.startButton = false;
      break;  // break the while loop to proceed
    }
    else if(numReceivedBytes > 0){  // user has provided some input
      switch(inputBuffer[0]){
        case 't': case 'T':
          if(atof(inputBuffer+1) > 0){
            // expand the highTime / lowTimes equally, keeping the original ratio
            float ratio = (float)systemTimes.highT / (float)systemTimes.samplingT;
            systemTimes.samplingT = 1000000 * atof(inputBuffer+1); 
            systemTimes.highT = ratio * systemTimes.samplingT;
            validInput = true;
          }          
          else{
            validInput = false;
          }
          break;
        default:
          validInput = false;
          break;
      }
      if(validInput){ // check for valid input
        break; // break the while-loop to proceed
      }
    }
  }
  }

void RTMChub::checkForStopCommand(){
  char inputBuffer[16] = {0};

  if(Serial.available() || irqInfo.startButton){
    util.recvWithEndMarker(inputBuffer, sizeof(inputBuffer)); // read user input (empty the receive buffer)
    if(inputBuffer[0] == 0 || irqInfo.startButton){
      irqInfo.startButton = false;
      //START_LED_OFF;
      trigger.end();
      receiver.end();
      printData();
      
      TXenable();
      sendCommand(COMMAND_STOP);
      Serial.println("Measurement finished.\r\n");
      delay(100); // give the nodes some time to process the command
      start();
    }
  }
  }





void RTMChub::restart(){
  // Magic number to restart the Teensy. Info from the prjc.com forum. Seems to be working...
  SCB_AIRCR = 0x05FA0004; 
  }
void RTMChub::receiveData(){
  core.receiveData();
  }
void RTMChub::transmitData(){
  core.transmitData();
  }
void RTMChub::printData(){
  print.data();
}


void TXenable(){
  digitalWriteFast(TRIGGER_PIN, HIGH);
  }
void RXenable(){
  digitalWriteFast(TRIGGER_PIN, LOW);
  }
void triggerIRQ(void){
  RXenable();
  irqInfo.triggerTime = micros();
  if(irqInfo.firstTrigger){
    irqInfo.startTime = irqInfo.triggerTime;
    state = STATE_IDLE; // don't evoke RTMCloop_printData();
    irqInfo.firstTrigger = false;
  }
  else{
    state = STATE_TRIGGER_SET;
  }
  }
void receiverIRQ(void){
  TXenable();
  irqInfo.previousTime = irqInfo.triggerTime;
  state = STATE_RECEIVE;
  }
void startButtonIRQ(void){
  delay(100); // debounce really badly
  while(!digitalRead(START_BUTTON_PIN)); // wait for user to release the button
  irqInfo.startButton = true; 
}

void loop() {
  // DO NOT ADD OR CHANGE ANYTHING IN "loop()"


  //..._____                    _____________________________________
  //        |                  |                                     |
  //        |__________________|                                     |__...
  //        ^      lowT        ^                highT                ^
  //   RECEIVE_IRQ         TRIGGER_IRQ                          RECEIVE_IRQ
  //
  // lowT:  MAIN: read data, process data, send data  // max(node.readTime) + max(node.processTime) + max(node.sendTime)
  //        NODE: -
  // highT: MAIN: print data of previous cycle        // max( max(node.responseTime), base.printTime )
  //        NODE: perform measurement / read data, process data, send data

  switch (state) {
    case STATE_TRIGGER_SET:     rtmcHub.printData(); break;
    case STATE_RECEIVE:         rtmcHub.receiveData(); break;
    case STATE_DATA_READ:       processData(); state = STATE_DATA_PROCESSED; break;
    case STATE_DATA_PROCESSED:  rtmcHub.transmitData(); break;
    case STATE_IDLE:            rtmcHub.checkForStopCommand(); break;
    default:              rtmcHub.checkForStopCommand(); break;
  }
}


#endif