/*
  getInfo.ino

  Display several information about the module. The devEUI of the USI module
  can't be modified. It is useful to read it to configure the module in a
  LoRaWAN network when asked by the network server.
*/

#include "LoRaWANNode.h"

// Serial port use to communicate with the USI shield.
// By default, use D0 (Rx) and D1(Tx).
HardwareSerial SerialLora(D0, D1);

void setup()
{
  Serial.begin(115200);
  Serial.println("-- Get Info sketch --");

  // Enable the USI module and set the radio band.
  while (!loraNode.begin(&SerialLora, LORA_BAND_EU_868)) {
    Serial.println("Lora module not ready");
    delay(1000);
  }

  // Get the DevEUI
  String str = "Firmware version: ";
  loraNode.getFWVersion(&str);
  Serial.println(str);

  str = "LoRa stack version: ";
  loraNode.getVersion(&str);
  Serial.println(str);

  str = "Unique DevEUI: 0x";
  loraNode.getDevEUI(&str);
  Serial.println(str);

  str = "Application EUI: 0x";
  loraNode.getAppEUI(&str);
  Serial.println(str);

  str = "Application key: 0x";
  loraNode.getAppKey(&str);
  Serial.println(str);

  str = "Network session Key: 0x";
  loraNode.getNwkSKey(&str);
  Serial.println(str);

  str = "Application session key: 0x";
  loraNode.getAppSKey(&str);
  Serial.println(str);

  str = "Device address: 0x";
  loraNode.getDevAddr(&str);
  Serial.println(str);
}

void loop()
{
  //empty loop
}
