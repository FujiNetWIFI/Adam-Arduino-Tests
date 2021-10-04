#include <Arduino.h>

#define TXD2 21
#define RXD2 22

void setup() 
{
  Serial.begin(921600);
  Serial1.begin(62500,SERIAL_8N1,RXD2,TXD2,true);
}

void loop() 
{
  while (Serial1.available())
    Serial.printf("%02X ",Serial1.read());
}