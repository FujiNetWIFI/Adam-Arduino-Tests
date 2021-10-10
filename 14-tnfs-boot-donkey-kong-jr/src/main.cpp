#include <Arduino.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WiFiUdp.h>

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

#define TNFS_SERVER "atari-apps.irata.online"
#define TNFS_PORT 16384

bool initialized = false;
bool responseAcknowledged = false;

byte tnfs_fd;
WiFiUDP UDP;
byte block[1024];
unsigned long blocknum;
unsigned long seekedblocknum;

byte status[6] =
    {
        0x84,       // response.control.status
        0x00, 0x04, // length, 4 bytes (big endian)
        0x01,       // Device mode = block mode
        0x40,       // Device dependent status byte
        0x45        // Checksum
};

union
{
  struct
  {
    byte session_idl;
    byte session_idh;
    byte retryCount;
    byte command;
    byte data[508];
  };
  byte rawData[512];
} tnfsPacket;

/**
 * Mount the TNFS server
 */
void tnfs_mount()
{
  int start = millis();
  int dur = millis() - start;

  memset(tnfsPacket.rawData, 0, sizeof(tnfsPacket.rawData));
  tnfsPacket.session_idl = 0;
  tnfsPacket.session_idh = 0;
  tnfsPacket.retryCount = 0;
  tnfsPacket.command = 0;
  tnfsPacket.data[0] = 0x01; // vers
  tnfsPacket.data[1] = 0x00; // "  "
  tnfsPacket.data[2] = 0x2F; // /
  tnfsPacket.data[3] = 0x00; // nul
  tnfsPacket.data[4] = 0x00; // no username
  tnfsPacket.data[5] = 0x00; // no password

  UDP.beginPacket(TNFS_SERVER, TNFS_PORT);
  UDP.write(tnfsPacket.rawData, 10);
  UDP.endPacket();

  while (dur < 5000)
  {
    if (UDP.parsePacket())
    {
      int l = UDP.read(tnfsPacket.rawData, 512);
      if (tnfsPacket.data[0] == 0x00)
      {
        // Successful
        Serial.printf("Mounted\n");
        return;
      }
      else
      {
        // Error
        return;
      }
    }
  }
  // Otherwise we timed out.
}

/**
 * Open 'autorun.atr'
 */
void tnfs_open()
{
  int start = millis();
  int dur = millis() - start;
  tnfsPacket.retryCount++;   // increase sequence #
  tnfsPacket.command = 0x29; // OPEN
  tnfsPacket.data[0] = 0x01; // R/O
  tnfsPacket.data[1] = 0x00; //
  tnfsPacket.data[2] = 0x00; // Flags
  tnfsPacket.data[3] = 0x00; //
  tnfsPacket.data[4] = '/';  // Filename start
  tnfsPacket.data[5] = 'b';
  tnfsPacket.data[6] = 'o';
  tnfsPacket.data[7] = 'o';
  tnfsPacket.data[8] = 't';
  tnfsPacket.data[9] = '.';
  tnfsPacket.data[10] = 'd';
  tnfsPacket.data[11] = 'd';
  tnfsPacket.data[12] = 'p';
  tnfsPacket.data[13] = 0x00; // NUL terminated
  tnfsPacket.data[14] = 0x00; // no username
  tnfsPacket.data[15] = 0x00; // no password

  UDP.beginPacket(TNFS_SERVER, TNFS_PORT);
  UDP.write(tnfsPacket.rawData, 19 + 4);
  UDP.endPacket();

  while (dur < 5000)
  {
    if (UDP.parsePacket())
    {
      int l = UDP.read(tnfsPacket.rawData, 512);
      if (tnfsPacket.data[0] == 0x00)
      {
        // Successful
        tnfs_fd = tnfsPacket.data[1];
        Serial.printf("opened, FD %u\n", tnfs_fd);
        return;
      }
      else
      {
        // unsuccessful
        return;
      }
    }
  }
  // Otherwise, we timed out.
}

/**
 * TNFS read
 */
void tnfs_read()
{
  int start = millis();
  int dur = millis() - start;
  tnfsPacket.retryCount++;      // Increase sequence
  tnfsPacket.command = 0x21;    // READ
  tnfsPacket.data[0] = tnfs_fd; // returned file descriptor
  tnfsPacket.data[1] = 0x00;    // 256 bytes
  tnfsPacket.data[2] = 0x01;    //

  UDP.beginPacket(TNFS_SERVER, TNFS_PORT);
  UDP.write(tnfsPacket.rawData, 4 + 3);
  UDP.endPacket();

  while (dur < 5000)
  {
    if (UDP.parsePacket())
    {
      int l = UDP.read(tnfsPacket.rawData, sizeof(tnfsPacket.rawData));
      if (tnfsPacket.data[0] == 0x00)
      {
        // Successful
        Serial.printf("Read 256 bytes\n");
        return;
      }
      else
      {
        // Error
        return;
      }
    }
  }
}

/**
 * TNFS seek
 */
void tnfs_seek(long offset)
{
  int start = millis();
  int dur = millis() - start;
  byte offsetVal[4];

  // This may be sending the bytes in the wrong endian, pls check. Easiest way is to flip the indices.
  offsetVal[0] = (int)((offset & 0xFF000000) >> 24);
  offsetVal[1] = (int)((offset & 0x00FF0000) >> 16);
  offsetVal[2] = (int)((offset & 0x0000FF00) >> 8);
  offsetVal[3] = (int)((offset & 0X000000FF));

  tnfsPacket.retryCount++;
  tnfsPacket.command = 0x25; // LSEEK
  tnfsPacket.data[0] = tnfs_fd;
  tnfsPacket.data[1] = 0x00; // SEEK_SET
  tnfsPacket.data[2] = offsetVal[3];
  tnfsPacket.data[3] = offsetVal[2];
  tnfsPacket.data[4] = offsetVal[1];
  tnfsPacket.data[5] = offsetVal[0];

  UDP.beginPacket(TNFS_SERVER, TNFS_PORT);
  UDP.write(tnfsPacket.rawData, 6 + 4);
  UDP.endPacket();

  while (dur < 5000)
  {
    if (UDP.parsePacket())
    {
      int l = UDP.read(tnfsPacket.rawData, sizeof(tnfsPacket.rawData));

      if (tnfsPacket.data[0] == 0)
      {
        // Success.
        Serial.printf("seek to: %lu\n", offset);
        return;
      }
      else
      {
        // Error.
        return;
      }
    }
  }
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

unsigned short adamnet_recv_length()
{
  unsigned short s = 0;

  s = adamnet_recv() << 8;
  s |= adamnet_recv();

  return s;
}

void adamnet_send(byte b)
{
  while (!Serial2.availableForWrite())
    yield();

  Serial2.write(b);
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
  byte c = adamnet_checksum(block, sizeof(block));

  Serial.printf("Sending block...%lu\n", blocknum);

  ets_delay_us(150);
  adamnet_send(0xB4);
  adamnet_send(0x04);
  adamnet_send(0x00);
  adamnet_send_bytes(block, sizeof(block));
  adamnet_send(c);
}

void command_control_receive()
{
  if (blocknum != seekedblocknum)
  {
    tnfs_seek(blocknum * 1024);

    // recombine 1024 byte block from 4 TNFS packets.
    tnfs_read();
    memcpy(&block[0], &tnfsPacket.data[3], 256);
    tnfs_read();
    memcpy(&block[256], &tnfsPacket.data[3], 256);
    tnfs_read();
    memcpy(&block[512], &tnfsPacket.data[3], 256);
    tnfs_read();
    memcpy(&block[768], &tnfsPacket.data[3], 256);
    seekedblocknum = blocknum;
  }

  ets_delay_us(80);
  adamnet_send(0x94); // Indicate we received the receive request.
}

void command_data_send()
{
  short s = adamnet_recv_length();
  byte x[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  for (short i = 0; i < s; i++)
    x[i] = adamnet_recv();

  blocknum = x[3] << 24 | x[2] << 16 | x[1] << 8 | x[0];

  Serial.printf("Req %lu\n", blocknum);

  ets_delay_us(150);
  adamnet_send(0x94); // Acknowledge that we got the block.
}

void command_control_ready()
{
  ets_delay_us(200);
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
  Serial.begin(921600);
  Serial1.begin(62500, SERIAL_8N1, RXD2, TXD2, true);
  Serial2.begin(62500, SERIAL_8N1, -1, TXD2, true);

  Serial.printf("\n\n#ColecoAdam #FujiNet Test #14 - TNFS Boot Donkey Kong Jr.\n");

  Serial.printf("Connecting to Network...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.scanNetworks();
  // WiFi.begin("iPhone", "ac1xtzatqu8ri");
  WiFi.begin("iPhone-FFWG81EPN72Q","3016520488");

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.printf("wifi status: %u\n", WiFi.status());
    delay(10);
  }

  Serial.printf("Connected!\n\n");

  UDP.begin(16384);
  tnfs_mount();
  tnfs_open();

  tnfs_read();
  memcpy(&block[0], &tnfsPacket.data[3], 256);
  tnfs_read();
  memcpy(&block[256], &tnfsPacket.data[3], 256);
  tnfs_read();
  memcpy(&block[512], &tnfsPacket.data[3], 256);
  tnfs_read();
  memcpy(&block[768], &tnfsPacket.data[3], 256);

  Serial.printf("Read first block.\n");

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