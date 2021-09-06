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
  digitalWrite(TXD2,HIGH); // so we don't contend the adamnet.
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
  if (Serial2.available())
  {
    unsigned char b = Serial2.read();
    unsigned char dev = b & 0x0F;
    
    if (dev == 0x04)
    {
      unsigned char cmd = b >> 4;

      if (cmd == 0x0D)
      {
        unsigned char transferCommand[9];

        Serial.printf("CMD: Ready\n");
        getSingleByte();
        Serial2.readBytes(transferCommand,9);
        if ((transferCommand[0]>>4) == 0x06)
        {
          unsigned long wantedBlock = ( (unsigned long)(transferCommand[6] << 24) | (unsigned long)(transferCommand[5] << 16) | (unsigned long)(transferCommand[4] << 8) | (unsigned long)(transferCommand[3]));
          unsigned short shortBlock = (unsigned long)wantedBlock;
          Serial2.printf("Requested block %04x",shortBlock);
        }
      }

    }

    WaitForIdle();
  }
}