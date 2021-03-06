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

byte status[6] =
    {
        0x84,       // response.control.status
        0x00, 0x04, // length, 4 bytes (big endian)
        0x01,       // Device mode = block mode
        0x40,       // Device dependent status byte
        0x45        // Checksum
};

byte testBlock[1028] =
    {
        0xB4,       // response.data.send
        0x04, 0x00, // Length, 1024 bytes (big endian)

        0x78, 0x32, 0x6f, 0xfd, 0xcd, 0x16, 0xc8, 0x21, 0x76, 0xc8, 0x11, 0x89,
        0x18, 0x01, 0x0e, 0x00, 0xcd, 0x1a, 0xfd, 0xc3, 0x13, 0xc8, 0x01, 0x00,
        0x00, 0xcd, 0x20, 0xfd, 0x01, 0xe0, 0x01, 0xcd, 0x20, 0xfd, 0x01, 0x05,
        0x07, 0xcd, 0x20, 0xfd, 0x3e, 0x00, 0x21, 0x00, 0x1b, 0xcd, 0x29, 0xfd,
        0x3e, 0x01, 0x21, 0x00, 0x38, 0xcd, 0x29, 0xfd, 0x3e, 0x02, 0x21, 0x00,
        0x18, 0xcd, 0x29, 0xfd, 0x3e, 0x03, 0x21, 0x00, 0x00, 0xcd, 0x29, 0xfd,
        0x3e, 0x04, 0x21, 0x00, 0x20, 0xcd, 0x29, 0xfd, 0xcd, 0x38, 0xfd, 0x21,
        0x00, 0x20, 0x3e, 0xf0, 0x11, 0x10, 0x00, 0xcd, 0x26, 0xfd, 0x21, 0x10,
        0x20, 0x3e, 0x7f, 0x11, 0x10, 0x00, 0xcd, 0x26, 0xfd, 0x01, 0x17, 0x1f,
        0x11, 0x00, 0x00, 0x21, 0x00, 0x38, 0xcd, 0x36, 0xfc, 0xc9, 0x42, 0x4c,
        0x4f, 0x43, 0x4b, 0x20, 0x30, 0x20, 0x4c, 0x6f, 0x61, 0x64, 0x65, 0x64,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,

        0x04 // Checksum

};

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
  Serial.printf("\n");
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

  Serial.printf("%02X ",incomingByte);

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
  digitalWrite(TXD2, HIGH);
  ets_delay_us(16);

  // Send the data bits
  for (byte mask = 0x01; mask>0; mask<<=1)
  {
    if (b & mask)
    {
      digitalWrite(TXD2,LOW);
    }
    else
    {
      digitalWrite(TXD2,HIGH);
    }
    ets_delay_us(16);
  }

  // Send stop bit.
  digitalWrite(TXD2,LOW);
  ets_delay_us(16);
}

void adamnet_send_bytes(byte *b, int len)
{
  for (int i = 0; i < len; i++)
    adamnet_send(b[i]);
}

void command_control_reset()
{
  // Let's not do anything for now.
}

void command_control_status()
{
  Serial.printf("command.control.status()\n");
  ets_delay_us(150);
  adamnet_send_bytes(status,sizeof(status));
}

void command_control_ack()
{
  Serial.printf("command.control.ack()\n");
}

void command_control_cts()
{
  Serial.printf("command.control.clr() - Sending test block!  ");
  adamnet_send_bytes(testBlock,sizeof(testBlock));
}

void command_control_receive()
{
  Serial.printf("command.control.receive()\n");
  ets_delay_us(150);
  adamnet_send(0x94); // RESPONSE.CONTROL.ACK for DEVICE 4, Tell Adam the block is ready.
}

void command_control_cancel()
{
  Serial.printf("command.control.cancel() - ADAM asked us to cancel\n");
  ets_delay_us(150);
  adamnet_send(0x94); // I'm assuming we should send back an ack. TODO: Will check the master 6801.
}

void command_data_send()
{
  byte transferData[9];
  unsigned long requestedBlock=0;
  Serial.printf("command.data.send() - Getting transfer data - ");
  ets_delay_us(150);
  adamnet_send(0x94); // Send ack to ADAM, letting it know we got the send command.
  adamnet_recv_bytes(transferData,sizeof(transferData)); // Get requested block payload
  requestedBlock = (transferData[6] << 24) | (transferData[5] << 16) | (transferData[4] << 8) | transferData[3];
  Serial.printf("Requested block: %lu\n", requestedBlock);
}

void command_control_nak()
{
  Serial.printf("command.control.nak()\n");
}

void command_control_ready()
{
  ets_delay_us(150); // Wait so adam doesn't miss
  adamnet_send(0x94); // Ack.
}

void setup()
{
  Serial.begin(921600);

  pinMode(RXD2, INPUT);
  pinMode(TXD2, OUTPUT);

  Serial.printf("\n\n#FujiNet AdamNet test #9 - boot attempt #3.\n\n");
}

void loop()
{
  byte b = adamnet_recv();
  byte d = b & 0x0F;
  byte c = b >> 4;

  if (d == 4)
  {
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
    case COMMAND_CONTROL_READY:
      command_control_ready();
      break;
    default:
      Serial.printf("Unhandled Command: %02X\n",c);
      wait_for_idle();
      break;
    }
  }
  else
    wait_for_idle();
}