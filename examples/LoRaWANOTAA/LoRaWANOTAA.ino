/*
  LoRaOTAA.ino

  Establish a connection on a LoRaWAN network in OTAA mode.
  Send a join request and wait the join accept.
  Exchange data to/from the network.

  Extract of the LoRaWAN standard:

    For Over The Air Activation, end-devices must follow a join procedure prior to participating in
    data exchanges with the network server. An end-device has to go through a new join
    procedure every time it has lost the session context information.

    The join procedure requires the end-device to be personalized with the following information
    before its starts the join procedure: a globally unique end-device identifier (DevEUI), the
    application identifier (AppEUI), and an AES-128 key (AppKey).

  USI will burn the unique IEEE EUI64 at factory.

*/

#include "LoRaWANNode.h"

#define FRAME_DELAY 300000  // in ms. Every 5 minutes by default.

// Serial port use to communicate with the USI shield.
// By default, use D0 (Rx) and D1(Tx).
// For Nucleo64, see "Known limitations" chapter in the README.md
HardwareSerial SerialLora(D0, D1);

// AppKey and AppEUI.
const char appKey[] = "0123456789abcdef0123456789abcdef";
const char appEUI[] = "0101010101010101";

// Data send
char frameTx[] = "Hello world!";

void setup()
{
  Serial.begin(9600);
  Serial.println("-- LoRaWAN OTAA sketch --");

  // Enable the USI module and set the radio band.
  while(!loraNode.begin(&SerialLora, LORA_BAND_EU_868)) {
    Serial.println("Lora module not ready");
    delay(1000);
  }

  // Send a join request and wait the join accept
  while(!loraNode.joinOTAA(appKey, appEUI)) {
    Serial.println("joinOTAA failed!!");
    delay(1000);
  }

  Serial.println("Lora module ready, join accepted.\n");

  String str = "Device EUI: ";
  loraNode.getDevEUI(&str);
  Serial.println(str);
  str = "Application key: ";
  loraNode.getAppKey(&str);
  Serial.println(str);
  str = "Application EUI: ";
  loraNode.getAppEUI(&str);
  Serial.println(str);
}

void loop()
{
  receive();
  transmit();
  delay(FRAME_DELAY);
}

void receive(void) {
  uint8_t frameRx[64];
  uint8_t len;
  uint8_t port;

  // Check if data received from a gateway
  if(loraNode.receiveFrame(frameRx, &len, &port)) {
    uint8_t n = 0;
    Serial.print("frame received: 0x");
    while(len > 0) {
      Serial.print(frameRx[n], HEX);
      Serial.print(',');
      len--;
      n++;
    }
    Serial.print(" on port "); Serial.println(port);
  } else {
    Serial.println("No data");
  }
}

void transmit(void) {
  // Send unconfirmed data to a gateway (port 1 by default)
  int status = loraNode.sendFrame(frameTx, sizeof(frameTx), UNCONFIRMED);
  if(status == LORA_SEND_ERROR) {
    Serial.println("Send frame failed!!!");
  } else if(status == LORA_SEND_DELAYED) {
    Serial.println("Module busy or duty cycle");
  } else {
    Serial.println("Frame sent");
  }
}
