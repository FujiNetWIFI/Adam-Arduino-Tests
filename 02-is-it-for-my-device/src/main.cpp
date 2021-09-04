#include <Arduino.h>

#define RXD2 22
#define TXD2 21

unsigned long idleStart;
unsigned long idleCurrent;
bool isIdle = false;

void WaitForIdle()
{
  if (Serial2.available())
    isIdle = false;

  if (isIdle == true)
    return;

  idleStart = idleCurrent = millis();

  while (idleCurrent - idleStart < 2)
  {
    // Wait for idle.
    while (Serial2.available()) { Serial2.read(); }
    idleCurrent = millis();
  }

  isIdle = true;
}

void setup() 
{
  pinMode(TXD2, OUTPUT);
  Serial.begin(921600);
  Serial2.begin(62500, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial Txd is on pin: " + String(TX));
  Serial.println("Serial Rxd is on pin: " + String(RX));
  Serial.printf("Baud rate is %lu\n", Serial2.baudRate());
  idleStart = idleCurrent = millis();
  while (Serial2.available()) { Serial2.read(); Serial.printf("."); }
}

void loop()
{
  if (Serial2.available())
  {
    unsigned char b = Serial2.read() & 0x0F;

    if (b == 8)
    {
      Serial.printf("TAPE\n");
    }

    while (Serial2.available()) { Serial2.read(); }
  }

  WaitForIdle();
}