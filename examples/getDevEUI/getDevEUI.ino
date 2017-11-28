/*
  getDevEUI.ino

  Display the unique device EUI of the module. The devEUI of the USI module
  can't be modified. It is useful to read it to configure the module in a
  LoRaWAN network when asked by the network server.

*/


#include "LoRaWANNode.h"

// Serial port use to communicate with the USI shield. Specific to the board Discovery L475 IoT.
HardwareSerial SerialLora(PA_1, PA_0);

void setup()
{
  Serial.begin(9600);
  Serial.println("-- Get DevEUI sketch --");

  // Enable the USI module and set the radio band.
  while(!loraNode.begin(&SerialLora, LORA_BAND_EU_868)) {
    Serial.println("Lora module not ready");
    delay(1000);
  }

  // Get the DevEUI
  String str ="The unique DevEUI is: 0x";
  loraNode.getDevEUI(&str);
  Serial.println(str);
}

void loop()
{
  //empty loop
}
