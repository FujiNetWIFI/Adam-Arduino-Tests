#include <Arduino.h>

void setup() 
{
  pinMode(21,OUTPUT);
}

void loop() 
{
  digitalWrite(21,!digitalRead(21));
  ets_delay_us(16);
}