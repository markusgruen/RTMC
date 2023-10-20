#ifndef RTMCHUB_H
#define RTMCHUB_H

#ifndef __IMXRT1062__
  #error "#error "Make sure to select 'Teensy 4.1' from Tools/Board""
#else 

#include <vector>
#include "common/RTMC.h"  // for BaseNode


// #define __RTMC_DEBUG__


// Pinmapping
#define TRIGGER_PIN        3
#define TX_EN_PIN         27
#define NODE_RESET_PIN     2
#define MAIN_RESET_PIN    38
#define MODE_PIN          23
#define START_BUTTON_PIN  22 
#define START_LED_PIN      4      

// constants
#define NUM_NODES          8
#define MEASURE_CYCLES    10


#define WAIT_RXTX                  5 // [µs]  // wait this long after pulling the trigger-signal low to send a command over the RS485 and after sending before pulling the trigger high again
#define TIMEOUT_NODE_RESPONSE  4000  // [ms]  // once a node is connected, this is the time RTMChub waits for it response, when a request is issued. If no response is received after this time, the node will be disabled

// functions
// #define START_LED_ON    digitalWrite(START_LED_PIN, LOW)
// #define START_LED_OFF   do{digitalWrite(START_LED_PIN, HIGH); delay(100);} while(0)


enum Mode{SERIAL_GERMAN, SERIAL_ENGLISH, SD_GERMAN, SD_ENGLISH, SERIAL_MONITOR};
enum SystemStates {STATE_IDLE, 
                   STATE_TRIGGER_SET, 
                   //STATE_TIME_TAKEN, 
                   STATE_RECEIVE, 
                   STATE_DATA_READ, 
                   STATE_DATA_PROCESSED};
extern volatile enum SystemStates state;

struct Times{unsigned long samplingT;
             unsigned long highT;
             unsigned long lowT;
             unsigned long responseT;
             unsigned long receiveT;
             unsigned long processT;
             unsigned long transmitT;
             unsigned long printT;
};

struct IRQinfo{uint64_t triggerTime;
               uint64_t previousTime;
               uint64_t startTime;
               bool firstTrigger;
               bool startButton;
};

class Node : public BaseNode{
  public:
    Node(HardwareSerial* port) : BaseNode(port){};

    //uint8_t numSendBytes = 0;
    unsigned long responseTime = 0;  // [µs]
    unsigned long transmitTime = 0;  // [µs]
    unsigned long receiveTime = 0;  // [µs]
    bool resetting = false;

};
extern Node node[NUM_NODES];


class RTMCcore{
  public:
    void sendCommand(char command);
    void sendCommand(Node _node, char command);
    void sendCommand(Node _node, char command, uint8_t i);
    void receiveData();
    void transmitData();
    void transmitData(Node _node);
    void resetBuffers();
    void removeInvalidNodes();

    Mode mode;
    Times systemTimes;
    
    struct CSV{
      char separator = ',';
      char decimalPoint = '.';
      char nanString[5];
    }csv;
};

class RTMCinit : public RTMCcore{
  public:
    void gpio();
    void getMode();
    void resetNodes();
    void getNodeInfo();
    void getNodeError();
    void getOutputInfos();
    void getInputInfos();
};

class RTMCprint : public RTMCcore{
  public:
    void data();
    void info(long samplingTime);
    void version();
    void header();
    void samplingTime(long samplingTime);
    void startTime();
};

class RTMCmeasure : public RTMCprint{
  public:
    void nodes();
    void responseTime();
    void RxTxTime();
    void processTime();
    void printTime();
  private:
    // long measureTransmitTime(Node _node);
    // long measureResponseTimeNew(Node _node);
    // long measureReceiveTime(Node _node);
    long measureTransmitTime(uint8_t i);
    long measureResponseTime(uint8_t i);
    long measureReceiveTime(uint8_t i);

    long receiveTime();
    long measureTransmitTime();

};

class RTMChub : public RTMCcore{
  public:
    RTMChub();
    RTMCmeasure measure;
    void begin();
    void start();
    void checkForStopCommand();
    void printData();
    void receiveData();
    void transmitData();

  private:
    void calculateSampleTime();
    void waitForStartCommand();
    static void restart();
};

extern RTMChub rtmcHub;

void RXenable();
void TXenable();
void triggerIRQ(void);
void receiverIRQ(void);
void startButtonIRQ(void);

void extern processData(void);

#endif
#endif
