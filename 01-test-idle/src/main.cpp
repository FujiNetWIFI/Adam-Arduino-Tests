#include <Arduino.h>

#define RXD2 22
#define TXD2 21

int i = 0;

unsigned long idleStart;
unsigned long idleCurrent;

void adamNet_rx()
{
}

void setup()
{
  // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  pinMode(TXD2, OUTPUT);
  Serial.begin(921600);
  Serial.println("Serial Txd is on pin: " + String(TX));
  Serial.println("Serial Rxd is on pin: " + String(RX));
  idleStart = millis();
}

void loop()
{
  unsigned long start, current;
  bool isIdle = false;

  do 
  {
    while (digitalRead(RXD2) == HIGH) { yield(); }

    start = micros();
    while ((digitalRead(RXD2) == LOW) && (isIdle == false))
    {
      current = micros();
      if ((current - start) > 2000)
      {
        isIdle = true;
        Serial.printf("Bus is idle.");
      }
    }
  } 
  while (isIdle == false);
}