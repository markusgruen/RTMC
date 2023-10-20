#include "RTMCutil.h"
#include <Arduino.h>

RTMCutil::RTMCutil(){};

int RTMCutil::recvWithEndMarker(char* _str, unsigned int _size) {
  static byte ndx = 0;
  int count = 0;
  char rc;
  bool newData = false;

  while (Serial.available() > 0) {
    rc = Serial.read();

    if (rc != '\r' && rc != '\n') {
      _str[ndx] = rc;
      ndx++;
      if (ndx >= _size) {
        ndx = _size - 1;
      }
    } else {
      _str[ndx] = '\0';                           // terminate the string
      if (rc == '\r' && Serial.peek() == '\n') {  // read out line feed in case of \r\n as endmarker
        rc = Serial.read();                       // remove '\n' from the buffer
      }
      count = ndx;
      ndx = 0;
      newData = true;
    }
  }
  if (newData) {
    return count;
  } else {
    return -1;
  }
}
