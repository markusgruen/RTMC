#ifdef ARDUINO_ARCH_RP2040

#include "RTMCnode.h"
#include <Wire.h>  // to be able to map the SDA/SCL pins correctly via Wire.setSDA() and Wire.setSCL();

constexpr float VERSION{0.3};
//constexpr unsigned long RECEIVE_TIMEOUT{1000 * 1000 * numReceiveBytes*10 / BAUDRATE_NODE + 20};  

bool __uninitialized_ram(resetReason);  // resetReason will remain in RAM even during restart via watchdog!

volatile enum SystemStates state = STATE_IDLE;
enum Modes mode = MODE_IDLE;


// volatile int out = 0;  // was macht out?

Node::Node(const char* _name) : BaseNode(&Serial1){
  strlcpy(name, _name, sizeof(name));
  }
void Node::begin(){

  pinMode(TRIGGER_PIN, INPUT);
  pinMode(TX_EN_PIN, INPUT);

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);

  numReceiveBytes = numInputs*sizeof(InOut::data) + 1;  // +1 for checksum
  receiveTimeout = 1000 * 1000 * numReceiveBytes*10 / BAUDRATE_NODE + 20;  // [us] timeout when waiting for serial transfer of data. Assuming 10 bits per byte; +20 is for unexpected overhead
  maxFailCnt = MAX_FAIL_COUNT;

  RTMC::begin(BAUDRATE_NODE);

  // establish Serial only when NO re-start via watchdog took place
  if(resetReason == RESET_RUN_PIN){
    serialStartTime = millis();
    Serial.begin(BAUDRATE_HOST);
    #ifdef __RTMC_DEBUG__
      Serial2.begin(115200);
      delay(100);
      Serial2.print(F("Start"));
      #endif
    state = STATE_IDLE;
  }
  else{ // if reset via watchdog, immediatly go into Measure-mode
    resetBuffer();
    state = STATE_IDLE;
    enableTrigger();
  }
  
  clearResetReason();
  }

void Node::start(){
  // TODO add code for standalone measurement
  if(state == STATE_IDLE){
    while(millis()-serialStartTime < TIMEOUT_NODE_RESET-100){  // -100, damit noch Zeit bleibt, den Namen zu senden, bevor RTMCmain aufgibt
      if(Serial) { 
        Serial.print(F("RTMCnode by Markus Grün, Version: "));
        Serial.println(VERSION);
        
        hasSerial = true;
        break;
      }
    }
    sendName();
    readSensorData();  // read data, to get the dTypes of the output channels
    //disableTrigger();
    enableTrigger();
    state = STATE_IDLE;  // reset the state to IDLE (readData set the state to "DATA_READ")
  }
  }

void Node::sendName(){
  char message[MAX_MESSAGE_LENGTH -1] = {0};  // -1 to leave room for the checksum
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

void Node::sendErrorInfo(){
  while(digitalReadFast(TRIGGER_PIN) == 0);  // wait until the Master releases the Trigger
  sendMessage(node.error, strlen(node.error));
  }

void Node::transmitData(){
  // Ich gehe davon aus, dass MAX_MESSAGE_LENGTH-1 immer groß genug ist: 63 > 8*4 <-- passt!
  // Dies ist die schnellste Methode, "dataStream" zu beschreiben (getestet gegen über start = sprintf(start... wie hier beschrieben: https://www.mikrocontroller.net/topic/93067 
  char dataStream[MAX_MESSAGE_LENGTH] = {0};
  int cnt = 0;
  for(int i=0; i < numOutputs; i++) {
    for(uint8_t k=0; k<sizeof(output[i].data); k++){
      dataStream[cnt++] = output[i].data[k];
    }
  }
  while(digitalReadFast(TRIGGER_PIN) == 0);  // wait until the Master releases the Trigger;
  sendData(dataStream, cnt);
  enableTrigger();

  state = STATE_IDLE;  
  }

void Node::receive(){ 
  // node with inputs
  if(numInputs > 0) {
    if(available() >= numReceiveBytes){
      receiveData();
      state = STATE_DATA_RECEIVED;
    }
    else if(available() == 3 || available() == 4){
      if(receiveCommand()) {
        state = STATE_IDLE;
      }
      else {
        state = STATE_DATA_RECEIVED;
      }
    }
    else {
      failCnt++;
      state = STATE_DATA_RECEIVED;
    }

    // evaluate failCnt
    if(mode == MODE_RUN && failCnt >= maxFailCnt){
      state = STATE_FAILSAFE;
    }
  }

  // node without inputs
  else {
    if(available() > 0) {
      receiveCommand();
      state = STATE_IDLE;
    }
    else{
      state = STATE_DATA_PROCESSED;
    }
  }
  }

bool Node::receiveCommand(){
  char messageBuffer[MAX_MESSAGE_LENGTH] = {0};
  memset(messageBuffer, 0, sizeof(messageBuffer));

  int8_t ret = recvMessage(messageBuffer, sizeof(messageBuffer));
  if(ret == 1) { 
    switch(messageBuffer[0]) {
      case COMMAND_MEASURE: 
        enableTrigger();
        mode = MODE_RUN;
        failCnt = 0;
        #ifdef __RTMC_DEBUG__
          Serial2.print("M");
          #endif
        return true;
 
      case COMMAND_STOP:
        disableTrigger();
        mode = MODE_IDLE;
        failCnt = 0;
        #ifdef __RTMC_DEBUG__
          Serial2.print("S");
          #endif
        return true;

      case COMMAND_TEST:
        enableTrigger();  
        mode = MODE_TEST; 
        resetBuffer();
        failCnt = 0;
        #ifdef __RTMC_DEBUG__
          Serial2.print("T");
          #endif 
        return true;

      case COMMAND_ERROR:
        sendErrorInfo();
        failCnt = 0;
        #ifdef __RTMC_DEBUG__
          Serial2.print("E");
          #endif
        return true;

      case COMMAND_RESET:   
        reset();
        return true;
      
      default: 
        return false;
    }
  }
  else if(ret == 2) { // Input / Output command
    switch(messageBuffer[0]) {
      case COMMAND_OUTPUT:
        sendOutputInfo(messageBuffer[1]); 
        failCnt = 0;
        #ifdef __RTMC_DEBUG__
          Serial2.print("O");
          #endif
        return true;

      case COMMAND_INPUT:  
        sendInputInfo(messageBuffer[1]);  
        failCnt = 0;
        #ifdef __RTMC_DEBUG__
          Serial2.print("I");
          #endif
        return true;

      default:
        return false;
    }
  }
  else if (numReceiveBytes > 0){
    failCnt++;
  }
  return false;
  }

void Node::receiveData(){
  char dataBuffer[MAX_MESSAGE_LENGTH] = {0};
  memset(dataBuffer, 0, sizeof(dataBuffer));
  
  int cnt = 0;
  //int8_t ret = recvData(dataBuffer, numReceiveBytes, 4);  // timeout must be >2 otherwise the while-loop won't start reliably
  int8_t ret = recvData(dataBuffer, numReceiveBytes);
  if(ret > 0) {
    for(uint8_t k=0; k<numInputs; k++){
      memcpy(input[k].data, dataBuffer+cnt, sizeof(input[k].data));
      cnt += sizeof(input[k].data);
    }   
    dataValid = true;
    failCnt = 0;
  }
  else {
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

void enableTrigger(){
  attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN), triggerIRQ, RISING);
  }

void disableTrigger(){
  detachInterrupt(digitalPinToInterrupt(TRIGGER_PIN));
  }

void triggerIRQ(void) {
  disableTrigger();
  state = STATE_TRIGGERED;
  }

void clearResetReason(){
  resetReason = RESET_RUN_PIN;
}

void loop() {
  // DO NOT MODIFY OR ADD TO "loop()"
  if(mode == MODE_IDLE){
    if(node.available() >= 3) {
      node.receive();
    }
  }
  else{
    switch(state){
      case STATE_TRIGGERED:       readSensorData(); state = STATE_DATA_READ; break;
      case STATE_DATA_READ:       node.receive(); break;
      case STATE_DATA_RECEIVED:   processInputData(); state = STATE_DATA_PROCESSED; break; 
      case STATE_DATA_PROCESSED:  node.transmitData(); break;
      case STATE_IDLE:            break;
      case STATE_FAILSAFE:        failsafe(); state = STATE_IDLE; break;
      default:                    break;
    }
  } 
  // NO, REALLY, DON'T ADD ANYTHING HERE!
}



#endif