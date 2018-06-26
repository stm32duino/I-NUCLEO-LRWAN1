/**
  ******************************************************************************
  * @file    LoRaRadio.cpp
  * @author  WI6LABS
  * @version V1.0.0
  * @date    14-December-2017
  * @brief   LoRa radio API. This API allows to manage the I-NUCLEO-LRWAN1 module
  *          at the radio level. You will be able to configure the radio
  *          parameters and send/receive frame. In this case the LoRaWAN stack
  *          is not available.
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

/*
  NOTE: This mode doesn't provide any duty cycle management. If you use an EU
  band you must respect the LoRa duty cycle indication.
*/

#include "LoRaRadio.h"
#include "lora_driver.h"

// Module mode
#define IDLE_MODE 0
#define TX_MODE   4
#define RX_MODE   5

LoraRadio::LoraRadio()
{
  _power = 20;
  _freq = 868000000;
  _sf = 7;
  _bw = 0;
  _cr = 1;
  _crc = true;
  _preamble = 8;
  _invIQ = false;
}

/*
 * @brief  Initializes the LoRa module.
 * @param  pointer to the serial instance uses to communicate with the module
 * @retval Error code: false if initialization failed else true
 */
bool LoraRadio::begin(HardwareSerial *serialx)
{
  uint8_t nbTry = 0;
  uint8_t enable = 0;

  if(serialx == NULL) {
    return false;
  }

  // Enable UART
  if(Modem_IO_Init(serialx) != AT_OK) {
    return false;
  }

  LoRa_DumyRequest();

  // Local echo mode must be disabled
  Modem_AT_Cmd(AT_EXCEPT, AT_ATE, &enable);

  // Verbose response must be enabled for the AT parser
  enable = 1;
  if (Modem_AT_Cmd(AT_EXCEPT, AT_VERB, &enable) != AT_OK) {
    AT_VERB_cmd = false;
  }

  // Enable Lora module
  /*
    NOTE: Sometimes if the module is not ready when we call the previous command
    the first answer received by LoraInit() is not "OK" but "AT OK" (echo of the
    command sent). The driver doesn't understand this answer. So we use a loop
    to avoid this issue.
  */
  while(Lora_Init() != MODULE_READY) {
    if((nbTry++) > 3) {
      return false;
    }
  }

  // Set radio configuration
  if(!setRadio()) {
    return false;
  }

  return true;
}

/*
 * @brief  Stop LoRa radio.
 * @param  none
 * @retval none
 */
void LoraRadio::end(void)
{
  enableIdle();
  Modem_IO_DeInit();
}

/*
 * @brief  Set radio frequency.
 * @param  frequency in Hz: 860000000 - 1020000000 Hz. Default is 868000000Hz
 * @retval Error code: false if failed else true
 */
bool LoraRadio::setFrequency(uint32_t frequency)
{
  if(IS_RADIO_FREQ(frequency)) {
    _freq = frequency;
    return setRadio();
  }
  return false;
}

/*
 * @brief  Set radio output power.
 * @param  power in dbm: 5 - 20 dbm. Default is 20dbm.
 * @retval Error code: false if failed else true
 */
bool LoraRadio::setTxPower(uint8_t power)
{
  if(IS_RADIO_POWER(power)) {
    _power = power;
    return setRadio();
  }
  return false;
}

/*
  * @brief  Set radio spreading factor.
  * @param  sf: 6 - 12. Default is 7.
  *         RADIO_SF6  = SF6 (64 symbol/chip rate)
  *         RADIO_SF7  = SF7 (128 symbol/chip rate)
  *         RADIO_SF8  = SF8 (256 symbol/chip rate)
  *         RADIO_SF9  = SF9 (512 symbol/chip rate)
  *         RADIO_SF10 = SF10 (1024 symbol/chip rate)
  *         RADIO_SF11 = SF11 (2048 symbol/chip rate)
  *         RADIO_SF12 = SF12 (4096 symbol/chip rate)
  * @retval Error code: false if failed else true
  */
bool LoraRadio::setSpreadingFactor(uint8_t sf)
{
  if(IS_RADIO_SF(sf)) {
    _sf = sf;
    return setRadio();
  }
  return false;
}

/*
  * @brief  Set radio band width.
  * @param  bw: 0 - 2. Default is 0.
  *         RADIO_BW125  = 125KHz
  *         RADIO_BW250  = 250KHz
  *         RADIO_BW500  = 500KHz
  * @retval Error code: false if failed else true
  */
bool LoraRadio::setBandWidth(uint8_t bw)
{
  if(IS_RADIO_BW(bw)) {
    _bw = bw;
    return setRadio();
  }
  return false;
}

/*
  * @brief  Set radio coding rate.
  * @param  bw: 1 - 4. Default is 1.
  *         RADIO_CR4_5  = 4/5
  *         RADIO_CR4_6  = 4/6
  *         RADIO_CR4_7  = 4/7
  *         RADIO_CR4_8  = 4/8
  * @retval Error code: false if failed else true
  */
bool LoraRadio::setCodingRate(uint8_t cr)
{
  if(IS_RADIO_CR(cr)) {
    _cr = cr;
    return setRadio();
  }
  return false;
}

/*
  * @brief  Enable CRC.
  * @param  none
  * @retval Error code: false if failed else true
  */
bool LoraRadio::enableCRC(void)
{
  _crc = true;
  return setRadio();
}

/*
  * @brief  Disable CRC.
  * @param  none
  * @retval Error code: false if failed else true
  */
bool LoraRadio::disableCRC(void)
{
  _crc = false;
  return setRadio();
}

/*
  * @brief  Set preambule length.
  * @param  length: 5 - 65535. Default is 8.
  * @retval Error code: false if failed else true
  */
bool LoraRadio::setPreambuleLength(uint32_t length)
{
  if(IS_RADIO_PREAMBLE_LENGTH(length)) {
    _preamble = length;
    return setRadio();
  }
  return false;
}

/*
  * @brief  Enable inverted IQ signal.
  * @param  none
  * @retval Error code: false if failed else true
  */
bool LoraRadio::enableInvIQ(void)
{
  _invIQ = true;
  return setRadio();
}

/*
  * @brief  Disable inverted IQ signal.
  * @param  none
  * @retval Error code: false if failed else true
  */
bool LoraRadio::disableInvIQ(void)
{
  _invIQ = false;
  return setRadio();
}

/*
  * @brief  Send data.Data are send in polling mode.
  * @param  buffer: pointer to the bytes to send.
  * @param  len: number of bytes to send. Limited to 64 bytes.
  * @retval number of data sent. 0 if failed.
  */
uint8_t LoraRadio::write(uint8_t *buffer, uint8_t len)
{
  if((buffer != NULL) && (len > 0)) {
    if(len > MAX_PAYLOAD_LENGTH) {
      return 0;
    }
    sSendData_t frame = {1, buffer, len};
    if(enableTx() == AT_OK) {
      if(Modem_AT_Cmd(AT_SET, AT_TXT, &frame) == AT_OK) {
        while(Modem_AT_Cmd(AT_ASYNC_EVENT, AT_TXT, NULL) != AT_OK) {}
        if(enableRx() == AT_OK) {
          return len;
        }
      }
    }
  }

  return 0;
}

/*
  * @brief  Read received data. Data are read in polling mode.
  * @param  buffer: pointer to the bytes read.
  * @retval number of bytes read.
  */
uint8_t LoraRadio::read(uint8_t *buffer)
{
  uint8_t len = 0;
  uint8_t frame[128];

  if(buffer != NULL) {
    if(parseRcvData(frame) == AT_OK) {
      keyCharToInt((char *)frame, buffer, sizeof((char *)frame));
      len = sizeof((char *)buffer);
    }
  }

  return len;
}

/*
  * @brief  Put the LoRa module in sleep mode.
  * @param  none
  * @retval Error code: false if failed else true
  */
bool LoraRadio::sleep(void)
{
  if(Lora_SleepMode() == AT_OK) {
    //delay(500);
    if(Modem_AT_Cmd(AT_ASYNC_EVENT, AT, NULL ) == AT_OK) {
      return true;
    }
  }
  return false;
}

/*
  * @brief  Wake up the LoRa module.
  * @param  none
  * @retval Error code: false if failed else true
  */
bool LoraRadio::wakeup(void)
{
  LoRa_DumyRequest();
  if(Modem_AT_Cmd(AT_ASYNC_EVENT, AT, NULL ) == AT_OK) {
    return true;
  }
  return false;
}

/********************** Private class functions *******************************/

/*
  * @brief  Enable the radio in Rx mode.
  * @param  none
  * @retval Error code
  */
uint8_t LoraRadio::enableRx(void)
{
  uint8_t mode = RX_MODE;
  return Modem_AT_Cmd(AT_SET, AT_DEFMODE, &mode);
}

/*
  * @brief  Enable the radio in Tx mode.
  * @param  none
  * @retval Error code
  */
uint8_t LoraRadio::enableTx(void)
{
  uint8_t mode = TX_MODE;
  return Modem_AT_Cmd(AT_SET, AT_DEFMODE, &mode);
}

/*
  * @brief  Enable the radio in Idle mode.
  * @param  none
  * @retval Error code
  */
uint8_t LoraRadio::enableIdle(void)
{
  uint8_t mode = IDLE_MODE;
  return Modem_AT_Cmd(AT_SET, AT_DEFMODE, &mode);
}

/*
  * @brief  Set the radio parameters.
  * @param  none
  * @retval Error code
  */
bool LoraRadio::setRadio(void)
{
  uint8_t status = false;
  sRadioCtrlSet_t config = {_power, _freq, _sf, _bw, _cr, _crc, _preamble, _invIQ};

  if(enableIdle() == AT_OK) {
    if(Modem_AT_Cmd(AT_SET, AT_RF, &config) == AT_OK) {
      if(enableRx() == AT_OK) {
        status = true;
      }
    }
  }

  return status;
}

/*
  * @brief  Parse the received data.
  * @param  none
  * @retval Error code
  */
uint8_t LoraRadio::parseRcvData(void *pdata)
{
  uint8_t  ResponseComplete = 0;
  int8_t i = 0;
  char *ptrChr;
  ATEerror_t RetCode;
  uint32_t msStart;
  char response[DATA_RX_MAX_BUFF_SIZE];

  if(pdata == NULL) {
    return AT_END_ERROR;
  }

  // Cleanup the response buffer
  memset(response, 0x00, DATA_RX_MAX_BUFF_SIZE);

  while (!ResponseComplete) {
    msStart = millis();
    while (!HW_UART_Modem_IsNewCharReceived()) {
      if((millis() - msStart) > RESPONSE_TIMEOUT) {
        return AT_UART_LINK_ERROR;
      }
    }

    // Process the response
    response[i] = HW_UART_Modem_GetNewChar();

    // Wait up to carriage return OR the line feed marker
    if (/*(response[i] =='\r') || */(response[i] == '\n')) {
      DBG_PRINTF("LoRa radio rcv: %s\r\n", response);

      if (i!= 0) {  // Trap the asynchronous event
        // First statement to get back the return value
        response[i] = '\0';
        ptrChr = strchr(&response[0],'+'); // Skip the '\0''\r'
        if (strncmp(ptrChr, "+RCV", sizeof("+RCV")-1) == 0) {
          RetCode = AT_OK;
          if(AT_VERB_cmd) {
            ptrChr = strrchr(&response[1], ',');
            strcpy((char *)pdata, ptrChr+1);
          } else {
            ptrChr = strchr(&response[1],':');
            strcpy((char *)pdata, ptrChr+2);
		  }
          ResponseComplete = 1;
        } else {
          RetCode = AT_END_ERROR;
        }
        memset(response, 0x00, 16);
        i= -1; // Compensate the next index iteration and restart in [0]
      }
    } else {
      if (i ==  (DATA_RX_MAX_BUFF_SIZE-1)) {
        //  Frame overflow
        i = 0;
        return (AT_TEST_PARAM_OVERFLOW);
      }
    }
    i++;
  }

  return RetCode;
}

LoraRadio loraRadio;
