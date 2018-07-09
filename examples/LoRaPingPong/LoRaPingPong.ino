/*
  LoRaPingPong.ino

  Provides an example to use the shield as a simple LoRa radio.
  Data are exchange between two boards. The first receiving the PING message
  becomes slave and will just send a PONG answer.

*/

#include "LoRaRadio.h"

#define SEND_PERIOD_MS 1000 //send period every second.

// Serial port use to communicate with the USI shield.
// By default, use D0 (Rx) and D1(Tx).
// For Nucleo64, see "Known limitations" chapter in the README.md
HardwareSerial SerialLora(D0, D1);

// Messages to exchange
uint8_t PingMsg[] = "PING";
uint8_t PongMsg[] = "PONG";

bool isMaster = true;
bool next = true;
int timer = 0;

void setup()
{
  Serial.begin(9600);
  Serial.println("-- LoRa Ping Pong sketch --");

  while(!loraRadio.begin(&SerialLora)) {
    Serial.println("LoRa module not ready");
    delay(1000);
  }

  Serial.println("LoRa module ready\n");
}

void loop()
{
  uint8_t rcvData[64];

  if((isMaster == true) && (next == true)) {
    next = false;
    loraRadio.write(PingMsg, 4);
    timer = millis();
  }

  if(loraRadio.read(rcvData) > 0) {
    if(memcmp(rcvData, PongMsg, 4) == 0) {
      Serial.println((char *)PongMsg);
    } else if(memcmp(rcvData, PingMsg, 4) == 0) {
      isMaster = false;
      loraRadio.write(PongMsg, 4);
      Serial.println((char *)PingMsg);
    }
    memset(rcvData, 0, 5);
  }

  if(((millis() - timer) >= SEND_PERIOD_MS) && (isMaster == true)){
    next = true;
  }
}
