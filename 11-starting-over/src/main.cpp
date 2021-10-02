#include <Arduino.h>

#define TXD2 21
#define RXD2 22

#define IDLE_TIMEOUT 150

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

void command_control_reset()
{
  // Serial.printf("COMMAND.CONTROL.RESET\n");
}

void command_control_status()
{
  // Serial.printf("COMMAND.CONTROL.STATUS\n");
}

void command_control_ack()
{
  // Serial.printf("COMMAND.CONTROL.ACK\n");
}

void command_control_cts()
{
  // Serial.printf("COMMAND.CONTROL.CTS\n");
}

void command_control_receive()
{
  // Serial.printf("COMMAND.CONTROL.RECEIVE\n");
}

void command_control_cancel()
{
  // Serial.printf("COMMAND_CONTROL_CANCEL\n");
}

void command_data_send()
{
  short s = adamnet_recv_length();
  byte x[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  unsigned long block = 0;

  Serial.printf("Max msg size: %u bytes\n\n", s);

  for (short i = 0; i < s; i++)
    x[i] = adamnet_recv();

  Serial.printf("Checksum: %02X\n\n", adamnet_recv());

  block = x[3] << 24 | x[2] << 16 | x[1] << 8 | x[0];

  Serial.printf("Req block: %08lx\n", block);
}

void command_control_nak()
{
  // Serial.printf("COMMAND.CONTROL.NAK\n");
}

void response_control_status()
{
  short l = adamnet_recv_length();
  Serial.printf("RESPONSE.CONTROL.STATUS\n");
  Serial.printf("Max msg size: %u bytes\n", l);
  Serial.printf("Transmit code: %s", (adamnet_recv() == 1) ? "BLOCK MODE\n" : "CHARACTER MODE\n");
  Serial.printf("Device Dependent Status: %02x\n", adamnet_recv());
  Serial.printf("Checksum: %02x\n\n", adamnet_recv());
}

void response_control_ack()
{
  // Serial.printf("RESPONSE.CONTROL.ACK\n");
}

void response_control_cancel()
{
  // Serial.printf("RESPONSE.CONTROL.CANCEL\n");
}

void response_data_send()
{
  short s = adamnet_recv_length();
  Serial.printf("RESPONSE.DATA.SEND\n");
  Serial.printf("Max msg size: %u bytes\n\n", s);
  for (short i = 0; i < s; i++)
    Serial.printf("%02X ", adamnet_recv());
  Serial.printf("Checksum: %02X\n\n", adamnet_recv());
}

void response_control_nak()
{
  // Serial.printf("RESPONSE.CONTROL.NAK\n");
}

void command_control_ready()
{
  // Serial.printf("COMMAND.CONTROL_READY\n");
}

void setup()
{
  Serial.begin(921600);
  Serial1.begin(62500, SERIAL_8N1, RXD2, -1, true);
  Serial.printf("FujiNet Test #11 - Starting over.\n\n");
}

void loop()
{
  if (!Serial1.available())
    return;

  byte b = Serial1.read();
  byte c = b >> 4;

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

  if (c != COMMAND_CONTROL_CTS)
    wait_for_idle();
}