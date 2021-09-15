#include <Arduino.h>

#define TXD2 21
#define RXD2 22

/**
 * @brief wait for bus idle state
 * This works by waiting for a transition to LOW,
 * and subsequently sampling bit cells
 * until we see 10 contiguous LOW signals
 * indicating that the bus is now idle.
 */
void wait_for_idle()
{
  byte cellCount=0;

  while (digitalRead(RXD2) == HIGH) 
    yield();

  ets_delay_us(8);

  while (cellCount < 10)
  {
    if (digitalRead(RXD2) == LOW)
      cellCount++;
    else
      cellCount = 0;
    
    ets_delay_us(16); // Delay to next bit cell.
  }
}

byte adamnet_recv()
{
  byte incomingByte = 0;

  // Sample the start bit
  while (digitalRead(RXD2) == LOW)
    yield();
  
  ets_delay_us(24);

  // Sample the data bits
  for (unsigned char i = 8; i > 0; --i)
  {
    incomingByte >>= 1;
    if (digitalRead(RXD2) == LOW)
      incomingByte |= 0x80;
    else
      incomingByte |= 0x00;
    ets_delay_us(16);
  }

  // Sample the stop bit
  ets_delay_us(16);

  return incomingByte;
}

void adamnet_recv_bytes(byte *b, int len)
{
  for (int i = 0; i < len; i++)
    b[i] = adamnet_recv();
}

void adamnet_send(byte b)
{
  // Send start bit
  digitalWrite(TXD2, LOW);
  ets_delay_us(16);

  // Send data bits
  for (byte mask = 0x01; mask > 0; mask <<= 1)
  {
    if (b & mask)
      digitalWrite(TXD2, LOW);
    else
      digitalWrite(TXD2, HIGH);
    ets_delay_us(16);
  }

  // Send stop bit.
  digitalWrite(TXD2, HIGH);
  ets_delay_us(16);
}

void adamnet_send_bytes(byte *b, int len)
{
  for (int i = 0; i < len; i++)
    adamnet_send(b[i]);
}

void setup() 
{
  Serial.begin(921600);
  
  pinMode(RXD2,INPUT);
  pinMode(TXD2,OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(TXD2, HIGH); // So the AdamNet doesn't contend.
  digitalWrite(2, HIGH);
  digitalWrite(4, HIGH);
  digitalWrite(13, HIGH);

  Serial.printf("\n\n#FujiNet AdamNet test #8 - better idle.\n\n");
}

void loop() 
{
  byte b = adamnet_recv(); 
  byte d = b & 0x0F;
  byte c = b >> 4;

  Serial.printf("DEV: %02X - ",d);

  switch (c)
  {
    case 0x00:
      Serial.printf("RESET\n");
      break;
    case 0x01:
      Serial.printf("STATUS\n");
      break;
    case 0x02:
      Serial.printf("ACK\n");
      break;
    case 0x03:
      Serial.printf("CTS\n");
      break;
    case 0x04:
      Serial.printf("RECEIVE\n");
      break;    
    case 0x05:
      Serial.printf("CANCEL\n");
      break;    
    case 0x06:
      Serial.printf("SEND\n");
      break;    
    case 0x07:
      Serial.printf("NACK\n");
      break;    
    case 0x08:
      Serial.printf("RESPONSE STATUS\n");
      break;    
    case 0x09:
      Serial.printf("RESPONSE ACK\n");
      break;
    case 0x0A:
      Serial.printf("RESPONSE CANCEL\n");
      break;    
    case 0x0B:
      Serial.printf("RESPONSE DATA\n");
      break;    
    case 0x0C:
      Serial.printf("RESPONSE NACK\n");
      break;    
    case 0x0D:
      Serial.printf("READY\n");
      break;
    default:
      Serial.printf("Unknown %02X\n",c);    
  }
  
  wait_for_idle();
}