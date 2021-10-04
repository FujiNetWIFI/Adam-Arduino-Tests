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

bool initialized = false;
bool responseAcknowledged = false;

fs::File f;

byte block[1024];

byte status[6] =
    {
        0x84,       // response.control.status
        0x00, 0x04, // length, 4 bytes (big endian)
        0x01,       // Device mode = block mode
        0x40,       // Device dependent status byte
        0x45        // Checksum
};

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

byte adamnet_checksum(byte *buf, unsigned short len)
{
    byte checksum = 0x00;

    for (unsigned short i = 0; i < len; i++)
        checksum ^= buf[i];

    return checksum;
}

byte adamnet_recv()
{
  while (!Serial1.available())
    yield();

  return Serial1.read();
}

unsigned long adamnet_recv_block()
{
  unsigned long b=0;

  b |= adamnet_recv();
  b |= adamnet_recv() << 8;
  b |= adamnet_recv() << 16;
  b |= adamnet_recv() << 24;

  return b & 0xFFFF; // Mask off the top 16 bits.
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
  Serial1.write(b);

  Serial.printf("T:%02X ",b);
  while (!Serial1.available())
    yield();

  Serial1.read();
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
  ets_delay_us(IDLE_TIME);
  adamnet_send_bytes(status, sizeof(status));
  responseAcknowledged = false;
}

void command_control_ack()
{
  responseAcknowledged = true;
}

void command_control_cts()
{
  byte ck = adamnet_checksum(block,sizeof(block));

  ets_delay_us(IDLE_TIME);
  
  // Send RESPONSE.DATA.SEND
  adamnet_send(0xB4);

  // Send Message length, currently fixed size at 1024 bytes
  adamnet_send(0x00);
  adamnet_send(0x04);

  // Send block data
  adamnet_send_bytes(block, sizeof(block));

  // Send checksum
  adamnet_send(ck);
}

void command_control_receive()
{
  ets_delay_us(IDLE_TIME);
  adamnet_send(0x94); // Acknowledge the receive
}

void command_control_cancel()
{
  ets_delay_us(IDLE_TIME);
  adamnet_send(0x94); // Acknowledge
}

void command_data_send()
{
  short s = adamnet_recv_length();
  unsigned long b = adamnet_recv_block();
  
  adamnet_recv(); // Grab reserved block. Do not use.
  adamnet_recv(); // Grab checksum, but do not use it.

  ets_delay_us(IDLE_TIME);

  if (!f.seek(b * sizeof(block)))
  {
    Serial.printf("Could not seek to block %lu - Sending NAK\n",b);
    adamnet_send(0xC4); // NAK
    return;
  }

  if (f.readBytes((char *)block,sizeof(block)) != sizeof(block))
  {
    Serial.printf("Could not read block %lu - Sending NAK",b);
    adamnet_send(0xC4); // NAK
    return;
  }

  adamnet_send(0x94); // ACK
  Serial.printf("Block #%lu\n",b);
}

void command_control_nak()
{
  // Are we supposed to ack here that we got a nak?
}

void command_control_ready()
{
  ets_delay_us(IDLE_TIME);
  adamnet_send(0x94); // ACK
}

void process_packet(byte c)
{
  switch(c)
  {
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
  }
}

void setup() 
{
  Serial.begin(921600);
  Serial1.begin(62500,SERIAL_8N1,RXD2,TXD2,true);

  Serial.printf("\n\n#ColecoAdam #FujiNet Test #13 - Boot Donkey Kong Jr.\n");

  if (!SPIFFS.begin())
  {
    Serial.printf("Could not open SPIFFS Storage. Aborting.\n");
    return;
  }
  else
    Serial.printf("SPIFFS Storage opened.\n");

  f = SPIFFS.open("/boot.ddp");

  if (!f)
  {
    Serial.printf("Could not open file boot.ddp in SPIFFS Storage. Aborting.\n");
    return;
  }
  else
    Serial.printf("Opened file /boot.ddp for read\n#FUJINET Ready.\n");
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