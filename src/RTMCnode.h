#ifndef RTMCNODE_H
#define RTMCNODE_H

// #define __RTMC_DEBUG__

#include "common/RTMC.h"
#ifndef ARDUINO_ARCH_RP2040
  #error "Make sure to select 'Raspberry Pi Pico' from Tools/Board"
#else

// extern volatile int out;  // was macht out?

#define TRIGGER_PIN     22
#define TX_EN_PIN       27
#define SDA_PIN         12
#define SCL_PIN         13

#define RESET_WATCHDOG false
#define RESET_RUN_PIN  true

#define MAX_FAIL_COUNT  5


enum SystemStates{STATE_IDLE,
                  STATE_TRIGGERED,
                  STATE_DATA_RECEIVED,
                  STATE_DATA_PROCESSED,
                  STATE_DATA_READ,
                  STATE_FAILSAFE,
                  };
extern volatile SystemStates state;

enum Modes{MODE_IDLE,
           MODE_TEST,
           MODE_RUN,
           };
extern Modes mode;


class Node : public BaseNode{ 
  public:
    Node(const char *_name);
    
    void begin();
    void start();
    void receive();
    void transmitData();
    void reset();  

  private:
    void sendName();
    void sendOutputInfo(uint8_t idx);  
    void sendInputInfo(uint8_t idx);
    void sendErrorInfo();
    void receiveData();
    bool receiveCommand();
    long serialStartTime = 0;
    long receiveTimeout = 0;
    bool hasSerial = false;
    uint8_t inputDataStream[64];  // TODO
    uint8_t inputDataCnt;
};

extern Node node;

void triggerIRQ();
void enableTrigger();
void disableTrigger();
void clearResetReason();
void readSensorData();
void processInputData();

void loop();
void failsafe();

#endif
#endif
