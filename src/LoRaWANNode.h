/**
  ******************************************************************************
  * @file    LoRaWANNode.h
  * @author  WI6LABS
  * @version V1.0.0
  * @date    28-november-2017
  * @brief   LoRaWAN Arduino API for I-NUCLEO-LRWAN1 shield
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#ifndef __LORAWAN_NODE_H_
#define __LORAWAN_NODE_H_

#include "hw.h"

// LoRa band
#define  LORA_BAND_EU_868                                0U
#define  LORA_BAND_US_915                                1U

#define IS_BAND(band) ((band == LORA_BAND_EU_868)  || (band == LORA_BAND_US_915))

// Lorawan Class
// NOTE: the USI module supports only the class A!!!
#define  LORA_CLASS_A                                0U
#define  LORA_CLASS_B                                1U
#define  LORA_CLASS_C                                2U

#define IS_CLASS(loraClass) ( (loraClass == LORA_CLASS_A)  ||\
                              (loraClass == LORA_CLASS_B)  ||\
                              (loraClass == LORA_CLASS_C) )

// Data acknowledgment
#define  UNCONFIRMED                        0U
#define  CONFIRMED                          1U

// Send frame error flags
#define LORA_SEND_DELAYED   0     // Busy or duty cycle
#define LORA_SEND_ERROR     (-1)  // Send failed

class LoRaWANNodeClass {
public:

  LoRaWANNodeClass();
  // Begin initialization function
  bool begin(HardwareSerial *serialx, uint8_t band, uint8_t loraClass = LORA_CLASS_A);

  bool joinOTAA(const char *appKey, const char *appEui = NULL); //function to configure the otaa parameters
  bool joinABP(const char *devAddr, const char *nwkSKey, const char *appSKey); //function to configure the ABP parameters

  int8_t sendFrame(char frame[], uint8_t length, bool confirmed, uint8_t port = 1);  //function to send a frame
  bool receiveFrame(uint8_t *frame, uint8_t *length, uint8_t *port);

  // Lorawan stack & firmware version
  void getVersion(String *str);
  void getFWVersion(String *str);

  // ABP information
  void getDevAddr(String *str);
  void getNwkSKey(String *str);
  void getAppSKey(String *str);

  // OTAA information
  void getDevEUI(String *str);
  void getAppKey(String *str);
  void getAppEUI(String *str);

  // Set/Get Band
  bool setBand(uint8_t band);
  uint8_t getBand(void);

  // Set/get duty cycle
  bool setDutyCycle(bool state);
  bool getDutyCycle(void);

  // Set/get adaptative data rate
  bool setAdaptativeDataRate(bool state);
  bool getAdaptativeDataRate(void);

  // Set/get data rate (SF and band width)
  bool setDataRate(uint8_t value);
  uint8_t getDataRate(void);

  // Reset USI module
  void reset(void);

  // Set/get network type
  bool setPublicNwkMode(bool value);
  uint8_t getPublicNwkMode(void);

  // Enter/exit the sleep mode
  bool sleep(void);
  bool wakeup(void);

  // Set/get RX1 delay
  // NOTE: Delay time of RX2 window is 1000ms larger than RX1 window.
  bool setRx1Delay(uint32_t ms);
  uint32_t getRx1Delay(void);

  // Set/get Join Accept RX1 delay
  // NOTE: Delay time of Join Accept RX2 window is 1000ms larger than RX1 window.
  bool setJoinRx1Delay(uint32_t ms);
  uint32_t getJoinRx1Delay(void);

private:

};

extern LoRaWANNodeClass loraNode;

#endif //__LORAWAN_NODE_H_
