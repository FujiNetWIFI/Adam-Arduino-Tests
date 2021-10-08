#include <Arduino.h>
#include <SPI.h>
#include <SPIFFS.h>

#define TXD2 21
#define RXD2 22

#define IDLE_TIMEOUT 150
#define IDLE_TIME 150

#define DISK_DRIVE_1 0x04

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

byte block[1024];
unsigned long blocknum=0;
File f;

byte adamnet_checksum(byte *buf, unsigned short len)
{
  byte checksum = 0x00;

  for (unsigned short i = 0; i < len; i++)
    checksum ^= buf[i];

  return checksum;
}

void wait_for_idle()
{
  bool isIdle = false;
  unsigned long start, current, dur;

  do
  {
    // Wait for serial line to quiet down.
    while (Serial1.available())
      Serial1.flush(false);

    start = current = micros();

    while ((!Serial1.available()) && (isIdle == false))
    {
      current = micros();
      dur = current - start;
      if (dur > IDLE_TIMEOUT)
        isIdle = true;
    }
  } while (isIdle == false);
}

byte adamnet_recv()
{
  while (!Serial1.available())
    yield();

  return Serial1.read();
}

unsigned short adamnet_recv_length()
{
  unsigned short s = 0;

  s = adamnet_recv() << 8;
  s |= adamnet_recv();

  return s;
}

void adamnet_send(byte b)
{
  byte c;
  Serial1.write(b);

  while (!Serial1.available())
    yield();

  c = Serial1.read();

  if (c != b)
  {
    Serial.printf("ERR! Expected %02X got %02X. Waiting 150us\n",b,c);
  }
}

void adamnet_send_bytes(byte *b, int len)
{
  for (int i = 0; i < len; i++)
    adamnet_send(b[i]);
}

bool is_it_for_me(byte b)
{
  return (b & 0x0f) == DISK_DRIVE_1;
}

void command_control_status()
{
  ets_delay_us(150);
  adamnet_send_bytes(status, sizeof(status));
}

void command_control_cts()
{
  ets_delay_us(150);
  adamnet_send(0xB4);
  adamnet_send(0x04);
  adamnet_send(0x00);
  adamnet_send_bytes(block, sizeof(block));
  adamnet_send(adamnet_checksum(block,sizeof(block)));
}

void command_control_receive()
{
  ets_delay_us(150);
  adamnet_send(0x94); // Indicate we received the receive request.
}

void command_data_send()
{
  short s = adamnet_recv_length();
  byte x[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  for (short i = 0; i < s; i++)
    x[i] = adamnet_recv();


  blocknum = x[3] << 24 | x[2] << 16 | x[1] << 8 | x[0];

  ets_delay_us(300);
  adamnet_send(0x94); // Acknowledge that we got the block.
}

void command_control_ready()
{
  f.seek(blocknum*1024);
  f.readBytes((char *)block,sizeof(block));
  // ets_delay_us(150); // wait a complete byte length before responding
  adamnet_send(0x94); // Acknowledge to adam that we are ready.
}

void process_packet(byte c)
{
  switch (c)
  {
  case COMMAND_CONTROL_STATUS: // Adam asking for status
    command_control_status();
    break;
  case COMMAND_CONTROL_CTS: // Adam saying clear to send
    command_control_cts();
    break;
  case COMMAND_CONTROL_RECEIVE: // Adam says it wants to receive
    command_control_receive();
    break;
  case COMMAND_DATA_SEND: // Adam asks us to send a specific block
    command_data_send();
    break;
  case COMMAND_CONTROL_READY: // Adam says it's ready.
    command_control_ready();
    break;
  }
}

void setup()
{
  pinMode(RXD2,INPUT_PULLDOWN);
  pinMode(TXD2,OUTPUT);
  SPIFFS.begin();
  f = SPIFFS.open("/boot.ddp");
  Serial.begin(921600);
  Serial1.begin(62500, SERIAL_8N1, RXD2, TXD2, true);
  Serial.printf("FujiNet Test #11 - Starting over.\n\n");
}

void loop()
{
  byte b = adamnet_recv();
  byte c = b >> 4;

  if (is_it_for_me(b))
    process_packet(c);
  else
    wait_for_idle();
}