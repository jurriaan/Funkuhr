/**
 * Example sketch for the Funkuhr Arduino library
 */
#include "Funkuhr.h"
void callback(Dcf77Time time) {
  Serial.print("GOT SYNC:");   
  Serial.print(time.year);
  Serial.print('-');
  Serial.print(time.month);
  Serial.print('-');
  Serial.print(time.day);
  Serial.print(' ');
  Serial.print(time.hour);
  Serial.print(':');
  Serial.print(time.min);
  Serial.print('+');
  Serial.print(time.zone);
  Serial.print(' ');
}

Funkuhr dcf;

uint8_t curSec;

void setup(void) 
{
  Serial.begin(115200);
  dcf.init(callback);
}

unsigned long long oldbuff = 0;
void loop() 
{
  unsigned long long buff = dcf.getBuffer();
  if(buff != oldbuff) {
  for(int i =0; i < 59;i++) {
    Serial.print((buff >> i & 1)>0? '1':'0');
  }
  Serial.println();
  oldbuff= buff;
  }
  delay(1000);
}