#include <Arduino.h>

#define RXD1 10 // some random pin
#define TXD1 21

#define RXD2 22
#define TXD2 20 // Some random pin

char tapeString[64];

void setup()
{
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(2, 1);
  digitalWrite(4, 1);
  digitalWrite(13, 1);
  Serial.begin(921600);
  Serial1.begin(62500, SERIAL_8N1, RXD1, TXD1, false);
  Serial2.begin(62500, SERIAL_8N1, RXD2, TXD2, true);

  Serial.printf("\n\n\n#FujiNet Test #5 - Status decode\n\n\n");
}

void wait_for_idle()
{
  bool isIdle = false;
  unsigned long start, current;

  do
  {

    while (Serial2.available())
    {
      Serial2.read();
    } // Drain

    start = micros();

    while ((!Serial2.available()) && (isIdle == false))
    {
      current = micros();
      if ((current - start) > 900)
        isIdle = true;
    }

  } while (isIdle == false);
}

unsigned char wait_ack()
{
  char ret[1];
  Serial2.readBytes(ret, 1);
  return ret[0];
}

const char *tape_status(unsigned char statusByte)
{
  memset(tapeString, 0, sizeof(tapeString));

  strcat(tapeString, "Tape 1: ");
  switch (statusByte & 0x0F)
  {
  case 0:
    strcat(tapeString, "Inserted and OK. ");
    break;
  case 1:
    strcat(tapeString,"CRC Error. ");
    break;
  case 2:
    strcat(tapeString,"Missing Block. ");
    break;
  case 3:
    strcat(tapeString,"Empty Drive. ");
    break;
  case 4:
    strcat(tapeString, "Missing Drive. ");
    break;
  case 5:
    strcat(tapeString,"Write Protect tab in place. ");
    break;
  case 7:
    strcat(tapeString,"Drive Error. ");
    break;
  default:
    strcat(tapeString, "Unknown. ");
    break;
  }

  strcat(tapeString, "Tape 2: ");
  switch (statusByte >> 4)
  {
  case 0:
    strcat(tapeString, "Inserted and OK. ");
    break;
  case 1:
    strcat(tapeString,"CRC Error. ");
    break;
  case 2:
    strcat(tapeString,"Missing Block. ");
    break;
  case 3:
    strcat(tapeString,"Empty Drive. ");
    break;
  case 4:
    strcat(tapeString, "Missing Drive. ");
    break;
  case 5:
    strcat(tapeString,"Write Protect tab in place. ");
    break;
  case 7:
    strcat(tapeString,"Drive Error. ");
    break;
  default:
    strcat(tapeString, "Unknown. ");
    break;
  }

  return tapeString;
}

void command_control_status()
{
  union _status
  {
    struct _field
    {
      unsigned char addr;
      unsigned short length;
      unsigned char transmitMode;
      unsigned char statusByte;
      unsigned char checksum;
    } field;
    unsigned char rawData[6];
  } statusResponse;

  unsigned char status[6];
  unsigned char ack;

  Serial2.readBytes(statusResponse.rawData, sizeof(statusResponse));
  ack = wait_ack();

  // Flip the status endianness
  statusResponse.field.length = statusResponse.rawData[2] << 8 | statusResponse.rawData[1];

  Serial.printf("Status response:\n");
  Serial.printf("Max message size: %u bytes.\n",statusResponse.field.length);
  Serial.printf("Source Address: %02X\n", statusResponse.field.addr & 0x0F);
  Serial.printf("Transmit mode: %s\n", (statusResponse.field.transmitMode == 0 ? "Character" : "Block"));
  Serial.printf("Device dependent Status: %02X : %s\n", statusResponse.field.statusByte, tape_status(statusResponse.field.statusByte));
  Serial.printf("Checksum: %02X\n", statusResponse.field.checksum);

  Serial.printf("\n150us later - Ack byte from Adam: %02X\n", ack);
}

void command_control_ready()
{
  // char transferCommand[9];

  // Serial.printf("command.control.ready\n");
  // ets_delay_us(150);
  // Serial.printf("Ack: %02X\n", wait_ack());
  // Serial.readBytes(transferCommand, 9);
  // Serial.printf("Transfer Command: %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", transferCommand[0], transferCommand[1], transferCommand[2], transferCommand[3], transferCommand[4], transferCommand[5], transferCommand[6], transferCommand[7], transferCommand[8]);

  // if (transferCommand[0] == 0x68)
  // {
  //   unsigned long wantedBlock = transferCommand[6] << 24 | transferCommand[5] << 16 | transferCommand[4] << 8 | transferCommand[3];
  //   Serial.printf("Requested Block %lu\n", wantedBlock);
  // }
}

void command_data_send()
{
  Serial.printf("Command.data.send\n");
}

void command(unsigned char cmd)
{
  switch (cmd)
  {
  case 0x01:
    command_control_status();
    break;
  case 0x0D:
    command_control_ready();
    break;
  }
}

void loop()
{
  if (Serial2.available())
  {
    unsigned char b = Serial2.read();
    unsigned char dev = b & 0x0F;
    unsigned char cmd = b >> 4;

    if (dev == 8)
    {
      command(cmd);
    }

    wait_for_idle();
  }
}