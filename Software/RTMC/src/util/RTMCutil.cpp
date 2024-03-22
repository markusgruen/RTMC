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

#include "RTMCutil.h"
#include <Arduino.h>


int recvWithEndMarker(char* _dest, unsigned int _size) {
  static byte index = 0;
  int count = 0;
  char receivedByte;
  bool newData = false;

  while (Serial.available() > 0) {
    receivedByte = Serial.read();

    if (receivedByte != '\r' && receivedByte != '\n') {
      _dest[index] = receivedByte;
      index++;
      if (index >= _size) {
        index = _size - 1;
      }
    } else {
      _dest[index] = '\0';   // terminate the string
      if (receivedByte == '\r' && Serial.peek() == '\n') {  // read out line feed in case of \r\n as endmarker
        receivedByte = Serial.read();  // remove '\n' from the buffer
      }
      count = index;
      index = 0;
      newData = true;
    }
  }
  if (newData) {
    return count;
  } else {
    return -1;
  }
}
