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

#include "node.h"


void BaseNode::addOutput(const char* _name, uint8_t _precision){
  if(numOutputs < MAX_OUTPUTS)
  {
    strlcpy(output[numOutputs].name, _name, sizeof(output[numOutputs].name));
    output[numOutputs].precision = _precision;
    numOutputs++;
  }
  else
  {
    strlcpy(error, "too many outputs - max.number of outputs = 8", sizeof(error));
  }
}
  
void BaseNode::addInput(const char* _name, dTypes _dType){
  if(numInputs < MAX_INPUTS)
  {
    strlcpy(input[numInputs].name, _name, sizeof(input[numInputs].name));
    input[numInputs].dType = _dType;
    numInputs++;
  }
  else
  {
    strlcpy(error, "too many inputs - max.number of inputs = 7", sizeof(error));
  }
}

void BaseNode::addInput(const char* _name, String _typeString){
  if(numInputs < MAX_INPUTS)
  {
    String typeString = _typeString;
    typeString.toUpperCase();

    strlcpy(input[numInputs].name, _name, sizeof(input[numInputs].name));
    if(typeString == "INT")
    {
      input[numInputs].dType = INT;
    }
    else if(typeString == "FLOAT")
    {
      input[numInputs].dType = FLOAT;
    }
    else
    {
      strlcpy(error, "input value type not supported", sizeof(error));
    }
    
    numInputs++;
  }
  else
  {
    strlcpy(error, "too many inputs - max.number of inputs = 7", sizeof(error));
  }
}

