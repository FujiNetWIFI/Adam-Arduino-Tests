#include <Arduino.h>

#define RXD2 22
#define TXD2 21

int i = 0;
unsigned char data_payload[65535];

unsigned long idleStart;
unsigned long idleCurrent;
bool isIdle = false;

void adamNet_rx()
{
}

void setup()
{
  // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  pinMode(TXD2, OUTPUT);
  Serial.begin(921600);
  Serial2.begin(62500, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial Txd is on pin: " + String(TX));
  Serial.println("Serial Rxd is on pin: " + String(RX));
  Serial.printf("Baud rate is %lu\n", Serial2.baudRate());
  idleStart = millis();
  while (Serial2.available()) { Serial2.read(); Serial.printf("."); }
}

void loop()
{
  if (Serial2.available())
  {
    if (isIdle == true)
    {
      Serial.printf("AdamNet is doing something.\n");
    }

    isIdle = false;

    while (Serial2.available()) { Serial2.read(); } // drain

    idleStart = idleCurrent = millis();
  }
  else if (idleCurrent - idleStart > 10)
  {
    if (isIdle == false)
    {
      Serial.printf("AdamNet is idle.\n");
      isIdle = true;
    }
  }
  idleCurrent = millis();
}