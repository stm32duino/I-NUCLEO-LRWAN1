/*
  LoRaABP.ino

  Establish a connection on a LoRaWAN network in ABP mode.
  Exchange data to/from the network.

  Extract of the LoRaWAN standard:

    Activation by personalization directly ties an end-device to a specific network by-passing the join request
    - join accept procedure.

    Activating an end-device by personalization means that the DevAddr and the two session
    keys NwkSKey and AppSKey are directly stored into the end-device instead of the DevEUI,
    AppEUI and the AppKey. The end-device is equipped with the required information for
    participating in a specific LoRa network when started.

*/

#include "LoRaWANNode.h"

#define FRAME_DELAY 300000  // in ms. Every 5 minutes by default.

// Serial port use to communicate with the USI shield.
// By default, use D0 (Rx) and D1(Tx).
// For Nucleo64, see "Known limitations" chapter in the README.md
HardwareSerial SerialLora(D0, D1);

// Device address, network & application keys
const char devAddr[] = "ef00cb01";
const char nwkSKey[] = "abcdef0123456789abcdef0123456789";
const char appSKey[] = "0123456789abcdef0123456789abcdef";

// Data send
char frameTx[] = "Hello world!";

void setup()
{
  Serial.begin(9600);
  Serial.println("-- LoRaWAN ABP sketch --");

  // Enable the USI module and set the radio band.
  while(!loraNode.begin(&SerialLora, LORA_BAND_EU_868)) {
    Serial.println("Lora module not ready");
    delay(1000);
  }

  // Set the network keys of the module
  while(!loraNode.joinABP(devAddr, nwkSKey, appSKey)) {
    Serial.println("joinABP failed!!");
    delay(1000);
  }

  Serial.println("Lora module ready\n");

  String str = "Device address: ";
  loraNode.getDevAddr(&str);
  Serial.println(str);
  str = "NwkSKey: ";
  loraNode.getNwkSKey(&str);
  Serial.println(str);
  str = "AppSKey: ";
  loraNode.getAppSKey(&str);
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
