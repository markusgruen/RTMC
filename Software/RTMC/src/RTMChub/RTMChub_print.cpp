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
#include "RTMChub/RTMChub_print.h"
#include "RTMChub/RTMChub_main.h"  // for "state"
#include <algorithm>  // for "find"


void RTMCprint::version(float version){
  Serial.print(F("RTMChub -- Version: "));
  Serial.println(version, 2);
}

void RTMCprint::info(long samplingTime_us){
  char infoStr[256] = {0};

  snprintf(infoStr, sizeof(infoStr)-1, "\nnode No.%c response time [ms]%c node name%c num Outputs%c num Inputs%c value names%c ...", settings.separator, settings.separator, settings.separator, settings.separator, settings.separator, settings.separator);
  Serial.println(infoStr);

  for(uint8_t i=0; i<NUM_NODES; i++){
    Serial.print(i);
    Serial.write(settings.separator);
    Serial.write(' ');
    if(find(activeNodes.begin(), activeNodes.end(), i) != activeNodes.end()) {  // if "i" is in activeNodes
      Serial.print(node[i].responseTime_us/1000.0, 4);
      Serial.write(settings.separator);
      Serial.write(' ');

      Serial.print(node[i].name); 
      Serial.write(settings.separator);
      Serial.write(' ');

      Serial.print(node[i].numOutputs);
      Serial.write(settings.separator);
      Serial.write(' ');

      Serial.print(node[i].numInputs);
      for(int j=0; j<node[i].numOutputs; j++){
        Serial.write(settings.separator);
        Serial.write(' ');
        Serial.print(node[i].output[j].name);
      }
      for(int j=0; j<node[i].numInputs; j++){
        Serial.write(settings.separator);
        switch(node[i].input[j].dType){
          case INT:   Serial.print(" int "); break;
          case FLOAT: Serial.print(" float "); break;
          default: Serial.print(" Error: only 'int' or 'float' supported ");
        }
        Serial.print(node[i].input[j].name);
      }
      Serial.println();
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
  Serial.print(samplingTime_us/1000000.0, 6);
  Serial.print(F(" s, or "));
  Serial.print(1000000.0/samplingTime_us,1);
  Serial.println(F(" Hz."));
}

void RTMCprint::samplingTime(long samplingTime_us){
  Serial.print("Sampling Time [s]");
  Serial.write(settings.separator);
  Serial.write(' ');
  Serial.println(samplingTime_us/1000000.0, 6);
}

void RTMCprint::startTime(DateTime now){
  char dateStr[24] = {0};
  char timeStr[24] = {0};

  if(settings.mode == SERIAL_GERMAN || settings.mode == SD_GERMAN){
    snprintf(dateStr, sizeof(dateStr)-1, "%02d.%02d.%04d", now.day(), now.month(), now.year());
    snprintf(timeStr, sizeof(timeStr)-1, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  }    
  else{
    snprintf(dateStr, sizeof(dateStr)-1, "%04d/%d/%d", now.year(), now.month(), now.day());
    snprintf(timeStr, sizeof(timeStr)-1, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  }
  Serial.write(dateStr, strlen(dateStr));
  Serial.write(settings.separator);
  Serial.write(' ');
  Serial.write(timeStr, strlen(timeStr));
  Serial.println();
  Serial.println();
}

void RTMCprint::header(){
  // 1st line: print node names
  Serial.write(settings.separator);
  Serial.write(' ');
  for(unsigned int i: activeNodes) {
    for(int k=0; k<node[i].numOutputs; k++){
      Serial.print(node[i].name);
      Serial.write(settings.separator);
      Serial.write(' ');
    }
  }
  Serial.println();

  // 2nd line: print value names
  Serial.print(F("time [s]"));
  Serial.write(settings.separator);
  Serial.write(' ');
  for(unsigned int i: activeNodes) {
    for(int k=0; k<node[i].numOutputs; k++){
      Serial.print(node[i].output[k].name);
      Serial.write(settings.separator);
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
  float time = (irqHandle.previousTime_us - irqHandle.startTime_us) / 1000000.0;
  interrupts();
  
    
  if(settings.decimalPoint == '.') {
    Serial.print(time, 5);    // die Anzahl an Nachkommastellen dynamisch von der Frequenz abhängig machen?
    Serial.write(settings.separator);
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
          Serial.write(settings.separator);
          Serial.write(' ');
        }
      }
      else {
        for(int k=0; k<node[i].numOutputs; k++){
          Serial.print(settings.nanString);
          Serial.write(settings.separator);
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

    startPtr += sprintf(startPtr, "%.5f%c ", time, settings.separator);

    if (startPtr - out >= 8) {
      out[startPtr - out - 8] = settings.decimalPoint;
    }

    for(unsigned int i: activeNodes){
      if(node[i].dataValid){
        for(int k=0; k<node[i].numOutputs; k++){
          if(node[i].output[k].dType == INT) {  // if "int"
            startPtr += sprintf(startPtr, "%d%c ",   node[i].output[k].readInt(), settings.separator);
          }
          else{
            switch(node[i].output[k].precision) {
              case 0: startPtr += sprintf(startPtr, "%.0f%c ", node[i].output[k].readFloat(), settings.separator); *(startPtr-3) = settings.decimalPoint; break;
              case 1: startPtr += sprintf(startPtr, "%.1f%c ", node[i].output[k].readFloat(), settings.separator); *(startPtr-4) = settings.decimalPoint; break;
              case 2: startPtr += sprintf(startPtr, "%.2f%c ", node[i].output[k].readFloat(), settings.separator); *(startPtr-5) = settings.decimalPoint; break;
              case 3: startPtr += sprintf(startPtr, "%.3f%c ", node[i].output[k].readFloat(), settings.separator); *(startPtr-6) = settings.decimalPoint; break;
              case 4: startPtr += sprintf(startPtr, "%.4f%c ", node[i].output[k].readFloat(), settings.separator); *(startPtr-7) = settings.decimalPoint; break;
              case 5: startPtr += sprintf(startPtr, "%.5f%c ", node[i].output[k].readFloat(), settings.separator); *(startPtr-8) = settings.decimalPoint; break;
              case 6: startPtr += sprintf(startPtr, "%.6f%c ", node[i].output[k].readFloat(), settings.separator); *(startPtr-9) = settings.decimalPoint; break;
              default: startPtr += sprintf(startPtr, "%.6f%c ", node[i].output[k].readFloat(), settings.separator); *(startPtr-9) = settings.decimalPoint; break;
            }
          }
        }
      }
      else {
        for(unsigned int k=0; k<node[i].numOutputs; k++){
          startPtr += sprintf(startPtr, "%s%c ", settings.nanString, settings.separator);
        }
      }
    }
    Serial.write(out);
  }
  Serial.println();
  state = STATE_IDLE;
}

#endif