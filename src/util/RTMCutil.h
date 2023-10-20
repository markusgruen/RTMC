#ifndef RTMCUTIL_H
#define RTMCUTIL_H

class RTMCutil {
public:
  RTMCutil();
  int recvWithEndMarker(char* _str, unsigned int _size);
};



#endif