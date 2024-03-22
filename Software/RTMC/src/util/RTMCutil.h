/*
This is the library for the RTMChub's utility functions. It contains functions 
to receive data from the user's serial
 
MIT License

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


#ifndef RTMCUTIL_H
#define RTMCUTIL_H


/*!
 * @brief Reads the Serial's receive buffer and writes it to "_dest"
 * @param _dest Pointer to the char array where the received data will
 * be copied to
 * @param _size The char array's size
 * @return Returns -1 while no line-end-marker ('\n' or '\r') 
 * has yet been received, or returns the number of received bytes.
 */  
int recvWithEndMarker(char* _dest, unsigned int _size);



#endif