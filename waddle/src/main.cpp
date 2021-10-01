#include <Arduino.h>

#define TXD2 21
#define RXD2 22

void setup() 
{
  pinMode(RXD2,INPUT_PULLDOWN);
  pinMode(TXD2,OUTPUT);
  digitalWrite(TXD2,LOW);
}

void adamnet_send(byte b)
{
  // Send start bit
  digitalWrite(TXD2, HIGH);
  ets_delay_us(16);

  // Send data bits
  for (int i=0; i<8; i++)
  {
    if ((b & 0x01) == 0x01)
      digitalWrite(TXD2,LOW);
    else 
      digitalWrite(TXD2,HIGH);
    
    b >>= 1;

    ets_delay_us(16);
  }

  // Send stop bit.
  digitalWrite(TXD2,LOW);
  ets_delay_us(16);
}

void loop() 
{
}