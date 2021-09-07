#include <Arduino.h>

#define RXD2 22
#define TXD2 21

void WaitForIdle()
{
  unsigned long start;
  unsigned long current;
  bool isIdle;
  
  do
  {
  
    while (Serial2.available()) 
    {
      Serial2.read(); // Drain
    }

    start = micros();

    while (!Serial2.available())
    {
      current = micros();

     if (current - start > 2000)
      isIdle = true;
    }
  } while (isIdle == false);
}

void setup()
{
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(RXD2,INPUT);
  pinMode(TXD2,OUTPUT);
  digitalWrite(2, 1);
  digitalWrite(4, 1);
  digitalWrite(13, 1);
  Serial.begin(921600);
  Serial2.begin(62500,SERIAL_8N1,RXD2,TXD2,false);
  Serial.printf("\n\n\n");
  Serial.println("Serial Txd is on pin: " + String(TX));
  Serial.println("Serial Rxd is on pin: " + String(RX));
}

unsigned char getSingleByte()
{
  char tmp[1];
  Serial2.readBytes(tmp,1);
  return tmp[0];
}

void loop()
{
  while (Serial2.available())
  {
    unsigned char b = Serial2.read(); // Get Node ID and Command
    Serial.printf("%02X ",b);
    if ((b & 0x0f) == 4) // For me?
    {
      Serial.printf("For me. command is %02x \n",(b >> 4));
      while (Serial2.available())
      {
        Serial.printf("%02X ",Serial2.read());
      }
    }
    WaitForIdle();
  }
}