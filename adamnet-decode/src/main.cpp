#include <Arduino.h>

#define TXD2 21
#define RXD2 22

#define COMMAND_CONTROL_RESET 0x00
#define COMMAND_CONTROL_STATUS 0x01
#define COMMAND_CONTROL_ACK 0x02
#define COMMAND_CONTROL_CTS 0x03
#define COMMAND_CONTROL_RECEIVE 0x04
#define COMMAND_CONTROL_CANCEL 0x05
#define COMMAND_DATA_SEND 0x06
#define COMMAND_CONTROL_NAK 0x07
#define COMMAND_CONTROL_READY 0x0D

#define RESPONSE_CONTROL_STATUS 0x08
#define RESPONSE_CONTROL_ACK 0x09
#define RESPONSE_CONTROL_CANCEL 0x0A
#define RESPONSE_DATA_SEND 0x0B
#define RESPONSE_CONTROL_NAK 0x0C

/**
 * @brief wait for bus idle state
 * This works by waiting for a transition to LOW,
 * and subsequently sampling bit cells
 * until we see 10 contiguous LOW signals
 * indicating that the bus is now idle.
 */
void wait_for_idle()
{
  byte cellCount = 0;

  while (digitalRead(RXD2) == HIGH)
    yield();

  ets_delay_us(8); // go to middle of bit cell

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

void setup()
{
  Serial.begin(921600);

  pinMode(RXD2, INPUT);
  pinMode(TXD2, OUTPUT);

  Serial.printf("\n\n#FujiNet AdamNet test #10 - Protocol Decode\n\n");
}

void command_control_reset()
{
  Serial.printf("COMMAND.CONTROL.RESET\n");
}

void command_control_status()
{
  Serial.printf("COMMAND.CONTROL.STATUS\n");
}

void command_control_ack()
{
  Serial.printf("COMMAND.CONTROL.ACK\n");
}

void command_control_cts()
{
  Serial.printf("COMMAND.CONTROL.CTS\n");
}

void command_control_receive()
{
  Serial.printf("COMMAND.CONTROL.RECEIVE\n");
}

void command_control_cancel()
{
  Serial.printf("COMMAND_CONTROL_CANCEL\n");
}

void command_data_send()
{
  short s = (adamnet_recv() << 8) | adamnet_recv();
  Serial.printf("Max msg size: %u bytes\n\n",s);
  for (short i = 0;i<s;i++)
    Serial.printf("%02X ",adamnet_recv());
  Serial.printf("Checksum: %02X",adamnet_recv());
}

void command_control_nak()
{
  Serial.printf("COMMAND.CONTROL.NAK\n");
}

void response_control_status()
{
  short l = (adamnet_recv() << 8) | adamnet_recv();
  Serial.printf("RESPONSE.CONTROL.STATUS\n");
  Serial.printf("Max msg size: %u bytes\n",l);
  Serial.printf("Transmit code: %s",(adamnet_recv() == 1) ? "BLOCK MODE\n" : "CHARACTER MODE\n");
  Serial.printf("Device Dependent Status: %02x\n",adamnet_recv());
  Serial.printf("Checksum: %02x\n",adamnet_recv());
}

void response_control_ack()
{
  Serial.printf("RESPONSE.CONTROL.ACK\n");
}

void response_control_cancel()
{
  Serial.printf("RESPONSE.CONTROL.CANCEL\n");
}

void response_data_send()
{
  short s = (adamnet_recv() << 8) | adamnet_recv();
  Serial.printf("RESPONSE.DATA.SEND\n");
  Serial.printf("Max msg size: %u bytes\n\n",s);
  for (short i = 0;i<s;i++)
    Serial.printf("%02X ",adamnet_recv());
  Serial.printf("Checksum: %02X",adamnet_recv());  
}

void response_control_nak()
{
  Serial.printf("RESPONSE.CONTROL.NAK\n");
}

void command_control_ready()
{
  Serial.printf("COMMAND.CONTROL_READY\n");
}

void loop()
{
  byte b = adamnet_recv();
  byte d = b & 0x0F;
  byte c = b >> 4;

  Serial.printf("--- PACKET BEGIN ---\n");

  Serial.printf(" DEVICE: %x\n",d);
  Serial.printf("COMMAND: %x\n\n",c);

  switch (c)
  {
  case COMMAND_CONTROL_RESET:
    command_control_reset();
  case COMMAND_CONTROL_STATUS:
    command_control_status();
    break;
  case COMMAND_CONTROL_ACK:
    command_control_ack();
    break;
  case COMMAND_CONTROL_CTS:
    command_control_cts();
    break;
  case COMMAND_CONTROL_RECEIVE:
    command_control_receive();
    break;
  case COMMAND_CONTROL_CANCEL:
    command_control_cancel();
    break;
  case COMMAND_DATA_SEND:
    command_data_send();
    break;
  case COMMAND_CONTROL_NAK:
    command_control_nak();
    break;
  case RESPONSE_CONTROL_STATUS:
    response_control_status();
    break;
  case RESPONSE_CONTROL_ACK:
    response_control_ack();
    break;
  case RESPONSE_CONTROL_CANCEL:
    response_control_cancel();
    break;
  case RESPONSE_DATA_SEND:
    response_data_send();
    break;
  case RESPONSE_CONTROL_NAK:
    response_control_nak();
    break;
  case COMMAND_CONTROL_READY:
    command_control_ready();
    break;
  default:
    Serial.printf("Unhandled Command: %02X\n", c);
    break;
  }

  Serial.printf("--- PACKET END ---\n");

  wait_for_idle();
}