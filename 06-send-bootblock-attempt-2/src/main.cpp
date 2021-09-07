#include <Arduino.h>

#define RXD1 10 // some random pin
#define TXD1 21

#define RXD2 22
#define TXD2 20 // Some random pin

byte get_ack()
{
  while (!Serial2.available())
  {
    yield();
  }
  return (byte)Serial2.read();
}
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

  Serial.printf("\n\n\n#FujiNet Test #5 - Is it for me (for real?)\n\n\n");
}

void command_control_ready()
{
  Serial.printf("command_control_ready\n");
}

void command_control_status()
{
  byte status[6];
  byte ack;

  Serial2.read(status, 6);
  ack = get_ack();

  Serial.printf("Status bytes sent to Adam: %02X %02X %02X %02X %02X %02X\n", status[0], status[1], status[2], status[3], status[4], status[5]);
  Serial.printf("Ack from Adam: %02X\n", ack);
}

void command(unsigned char cmd)
{
  switch (cmd)
  {
  case 0x0D: // COMMAND.CONTROL.READY
    command_control_ready();
    break;
  case 0x01: // COMMAND.CONTROL.STATUS
    command_control_status();
    break;
  default:
    Serial.printf("Unknown command: %02X\n", cmd);
    break;
  }
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

/**
 * Is this packet for me?
 * @param node ID to check
 * @param cmd pointer to where we store the resulting command byte
 * @return true if this is for us, false if not.
 */
bool is_it_for_me(unsigned char dev, unsigned char *cmd)
{
  unsigned char b = Serial2.read();
  unsigned char d = b & 0x0F;

  Serial.printf("B: %02X D: %02X", b, d);

  *cmd = b >> 4;

  return d == dev;
}

void loop()
{
  unsigned char cmd;

  if (Serial2.available())
    if (is_it_for_me(8, &cmd))
      command(cmd);

  wait_for_idle();
}