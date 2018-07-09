/**
  ******************************************************************************
  * @file    LoRaRadio.cpp
  * @author  WI6LABS
  * @version V1.0.0
  * @date    14-December-2017
  * @brief   LoRa radio API header
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

#ifndef __LORA_RADIO_H_
#define __LORA_RADIO_H_

#include "hw.h"

#define RADIO_MIN_FREQ  860000000
#define RADIO_MAX_FREQ  1020000000

#define IS_RADIO_FREQ(x)  (((x) >= RADIO_MIN_FREQ) && ((x) <= RADIO_MAX_FREQ))

#define RADIO_MIN_POWER  5
#define RADIO_MAX_POWER  20

#define IS_RADIO_POWER(x)  (((x) >= RADIO_MIN_POWER) && ((x) <= RADIO_MAX_POWER))

#define RADIO_SF6   6
#define RADIO_SF7   7
#define RADIO_SF8   8
#define RADIO_SF9   9
#define RADIO_SF10  10
#define RADIO_SF11  11
#define RADIO_SF12  12

#define IS_RADIO_SF(x) (((x) == RADIO_SF6)  || ((x) == RADIO_SF7)   ||\
                        ((x) == RADIO_SF8)  || ((x) == RADIO_SF9)   ||\
                        ((x) == RADIO_SF10) || ((x) == RADIO_SF11)  ||\
                        ((x) == RADIO_SF12))

#define RADIO_BW125 0
#define RADIO_BW250 1
#define RADIO_BW500 2

#define IS_RADIO_BW(x) (((x) == RADIO_BW125) || ((x) == RADIO_BW250) ||\
                        ((x) == RADIO_BW500))

#define RADIO_CR4_5 1
#define RADIO_CR4_6 2
#define RADIO_CR4_7 3
#define RADIO_CR4_8 4

#define IS_RADIO_CR(x) (((x) == RADIO_CR4_5) || ((x) == RADIO_CR4_6) ||\
                        ((x) == RADIO_CR4_7) || ((x) == RADIO_CR4_8))

#define RADIO_MIN_PREAMBLE_LENGTH 5
#define RADIO_MAX_PREAMBLE_LENGTH 65535

#define IS_RADIO_PREAMBLE_LENGTH(x)  (((x) >= RADIO_MIN_PREAMBLE_LENGTH) &&\
                                      ((x) <= RADIO_MAX_PREAMBLE_LENGTH))


class LoraRadio {
public:

  LoraRadio();

  // Begin initialization function
  bool begin(HardwareSerial *serialx);
  void end(void);

  bool setFrequency(uint32_t freq);
  bool setTxPower(uint8_t power);
  bool setSpreadingFactor(uint8_t sf);
  bool setBandWidth(uint8_t bw);
  bool setCodingRate(uint8_t cr);
  bool enableCRC(void);
  bool disableCRC(void);
  bool setPreambuleLength(uint32_t length);
  bool enableInvIQ(void);
  bool disableInvIQ(void);

  uint8_t write(uint8_t *buffer, uint8_t len);
  uint8_t read(uint8_t *buffer);

  bool sleep(void);
  bool wakeup(void);

private:
  uint8_t   _power;
  uint32_t  _freq;
  uint8_t   _sf;
  uint8_t   _bw;
  uint8_t   _cr;
  bool      _crc;
  uint16_t  _preamble;
  bool      _invIQ;

  uint8_t enableRx(void);
  uint8_t enableTx(void);
  uint8_t enableIdle(void);

  bool setRadio(void);

  uint8_t parseRcvData(void *pdata);
};

extern LoraRadio loraRadio;

#endif //__LORA_RADIO_H_
