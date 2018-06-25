/**
  ******************************************************************************
  * @file    LoRaWANNode.cpp
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

#include "LoRaWANNode.h"
#include "lora_driver.h"


LoRaWANNodeClass::LoRaWANNodeClass()
{

}

/*
 * @brief  Initializes the LoRa module
 * @param  pointer to the serial instance uses to communicate with the module
 * @param  LoRaWAN frequency band: 868MHz (EU) or 915MHz (US)
 * @param  LoRaWAN node class: A, B or C. NOTE: Classes B and C are not available.
 * @retval Error code: false if initialization failed else true
 */
bool LoRaWANNodeClass::begin(HardwareSerial *serialx, uint8_t band, uint8_t loraClass)
{
  uint8_t nbTry = 0;
  uint8_t enable = 0;

  if(!IS_BAND(band) || !IS_CLASS(loraClass) || (serialx == NULL)) {
    return 0;
  }

  if(loraClass != LORA_CLASS_A) {
    DBG_PRINTF_CRITICAL("The firmware only supports Class A so far.\r\n");
    return 0;
  }

  // Enable UART
  if(Modem_IO_Init(serialx) != AT_OK) {
    return 0;
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
      return 0;
    }
  }

  // Set band: EU or US.
  if(getBand() != band) {
    if(!setBand(band)) {
      return 0;
    }
  }

  // Enable duty cycle just in case it was disabled previously.
  if(!setDutyCycle(true)) {
    return 0;
  }

  /*  Set class (A, B or C).
      NOTE: Class B & C not supported! This code always returns an error.
      We only keep this code as example or when another class will be supported.
  */
#if 0
  if(Lora_SetClass(loraClass) != AT_OK) {
    printf("Class failed\r\n");
    return 0;
  }
#endif

  // Enable adaptative data rate by default
  if(!setAdaptativeDataRate(true)) {
    return 0;
  }

  return 1;
}

/*
 * @brief  Configures the module in ABP mode.
 * @param  Device address: 4 hexadecimal bytes
 * @param  Network session key: 16 hexadecimal bytes
 * @param  Application session key: 16 hexadecimal bytes
 * @retval Error code: false if failed else true
 */
bool LoRaWANNodeClass::joinABP(const char *devAddr, const char *nwkSKey, const char *appSKey)
{
  uint8_t key[16];

  if((devAddr != NULL) && (nwkSKey != NULL) && (appSKey != NULL)) {
    keyCharToInt(devAddr, key, 4);
    if(LoRa_SetDeviceAddress((key[0] << 24) | (key[1] << 16) | (key[2] << 8) | key[3]) == AT_OK) {
      keyCharToInt(nwkSKey, key, 16);
      if(LoRa_SetKey(AT_NWKSKEY, key) == AT_OK) {
        keyCharToInt(appSKey, key, 16);
        if(LoRa_SetKey(AT_APPSKEY, key) == AT_OK) {
          if(Lora_Join(ABP_JOIN_MODE) == AT_OK) {
            return 1;
          }
        }
      }
    }
  }
  return 0;
}

/*
 * @brief  Configures the module in OTAA mode.
 * @param  Application key: 16 hexadecimal bytes
 * @param  Application EUI: 16 hexadecimal bytes (optional: can be a NULL pointer)
 * @retval Error code: false if failed else true
 */
bool LoRaWANNodeClass::joinOTAA(const char *appKey, const char *appEui)
{
  uint8_t key[16];
  ATEerror_t ret;

  // HACK: remove duty cycle to be able to send again a Join Request
  setDutyCycle(false);

  // Can be optional
  if(appEui != NULL) {
    keyCharToInt(appEui, key, 16);
    if(LoRa_SetAppID(key) != AT_OK) {
      return 0;
    }
  }

  if(appKey != NULL) {
    keyCharToInt(appKey, key, 16);
    if(LoRa_SetKey(AT_APPKEY, key) == AT_OK) {
      ret = Lora_Join(OTAA_JOIN_MODE);
      if(ret == AT_OK) {  // Device joined
        return 1;
      } else if(ret == AT_JOIN_SLEEP_TRANSITION) {
        uint32_t timeout = millis();
        do {
          if((millis() - timeout) > DELAY_FOR_JOIN_STATUS_REQ) {
            return 0;
          }
          ret = Lora_JoinAccept();
        } while(ret != AT_OK);
        // HACK: enable again duty cycle
        setDutyCycle(true);
        return 1;
      }
    }
  }
  return 0;
}

/*
 * @brief  Send a LoRaWAN frame to the network.
 * @param  pointer to the frame to send. Size is limited to 64 bytes.
 * @param  Number of data to send.
 * @param  If true the module requires a acknowledgment from the network.
 * @param  The application port number.
 * @retval LoRa error code.
 */
int8_t LoRaWANNodeClass::sendFrame(char frame[], uint8_t length, bool confirmed, uint8_t port)
{
  if((frame == NULL) || (length == 0)) {
    return LORA_SEND_ERROR;
  }

  // Payload length is limited to 64 bytes.
  if(length > MAX_PAYLOAD_LENGTH) {
    return LORA_SEND_ERROR;
  }

  sSendDataBinary_t dataString = {frame, length, port, confirmed};
  ATEerror_t errorCode = Lora_SendDataBin(&dataString);
  if(errorCode == AT_OK) {
    return length;
  } else if(errorCode == AT_ERROR_WAN_SEND) {
    // Previous frame not sent yet (busy or duty cycle). Wait before to try again.
    return LORA_SEND_DELAYED;
  }
  return LORA_SEND_ERROR;
}

/*
 * @brief  Receives a LoRaWAN frame from the network. Must be called before the
 *         next call of sendFrame(). The receive delay depends on the Rx1 and Rx2
 *         windows.
 * @param  pointer to the received frame.
 * @param  Number of data received.
 * @param  The application port number.
 * @retval Error code: false if failed else true.
 */
bool LoRaWANNodeClass::receiveFrame(uint8_t *frame, uint8_t *length, uint8_t *port)
{
  uint8_t data[128];

  if((frame == NULL) || (length == NULL) || (port == NULL)) {
    return 0;
  }

  sReceivedDataBinary_t structData = {data, 0, 0};
  *length = 0;
  if(Lora_AsyncDownLinkData(&structData) == AT_OK) {
    if(structData.DataSize > 0) {
      *length = structData.DataSize;
      *port = structData.Port;
      keyCharToInt((char *)data, frame, *length);
      return 1;
    }
  }
  return 0;
}

/*
 * @brief  Read the LoRaWAN stack version.
 * @param  pointer to version number (an Arduino string allocated by caller).
 * @retval None.
 */
void LoRaWANNodeClass::getVersion(String *str)
{
  if(str != NULL) {
    char tmp[10] = {'\0'};
    Lora_GetVersion((uint8_t *)tmp);
    str->concat(tmp);
  }
}

/*
 * @brief  Read the firmware version.
 * @param  pointer to version number (an Arduino string allocated by caller).
 * @retval None.
 */
void LoRaWANNodeClass::getFWVersion(String *str)
{
  if(str != NULL) {
    char tmp[10];
    Lora_GetFWVersion((uint8_t *)tmp);
    tmp[9] = '\0';
    str->concat(tmp);
  }
}

/*
 * @brief  Read the device address.
 * @param  pointer to the device address (an Arduino string allocated by caller).
 * @retval None.
 */
void LoRaWANNodeClass::getDevAddr(String *str)
{
  if(str != NULL) {
    uint32_t addr;
    char cAddr[9] = {'\0'};
    char tmp[3];
    LoRa_GetDeviceAddress(&addr);
    keyIntToChar(cAddr, (uint8_t*)&addr, 4);
    // Swap characters
    for(uint8_t i = 0; i < 4; i+=2) {
      strncpy(tmp, &cAddr[i], 2);
      strncpy(&cAddr[i], &cAddr[6-i], 2);
      strncpy(&cAddr[6-i], tmp, 2);
    }
    str->concat(cAddr);
  }
}

/*
 * @brief  Read the network session key.
 * @param  pointer to the NwkSKey (an Arduino string allocated by caller).
 * @retval None.
 */
void LoRaWANNodeClass::getNwkSKey(String *str)
{
  if(str != NULL) {
    uint8_t iKey[16];
    char cKey[33] = {'\0'};
    LoRa_GetKey(AT_NWKSKEY, iKey);
    keyIntToChar(cKey, iKey, 16);
    str->concat(cKey);
  }
}

/*
 * @brief  Read the application session key.
 * @param  pointer to the AppSKey (an Arduino string allocated by caller).
 * @retval None.
 */
void LoRaWANNodeClass::getAppSKey(String *str)
{
  if(str != NULL) {
    uint8_t iKey[16];
    char cKey[33] = {'\0'};
    LoRa_GetKey(AT_APPSKEY, iKey);
    keyIntToChar(cKey, iKey, 16);
    str->concat(cKey);
  }
}

/*
 * @brief  Read the device EUI. The device EUI is unique and burnt at factory.
 * @param  pointer to the devie EUI (an Arduino string allocated by caller).
 * @retval None.
 */
void LoRaWANNodeClass::getDevEUI(String *str)
{
  if(str != NULL) {
    uint8_t iKey[8];
    char cKey[17] = {'\0'};
    LoRa_GetDeviceID(iKey);
    keyIntToChar(cKey, iKey, 8);
    str->concat(cKey);
  }
}

/*
 * @brief  Read the application key.
 * @param  pointer to the AppKey (an Arduino string allocated by caller).
 * @retval None.
 */
void LoRaWANNodeClass::getAppKey(String *str)
{
  if(str != NULL) {
    uint8_t iKey[16];
    char cKey[33] = {'\0'};
    LoRa_GetKey(AT_APPKEY, iKey);
    keyIntToChar(cKey, iKey, 16);
    str->concat(cKey);
  }
}

/*
 * @brief  Read the application EUI.
 * @param  pointer to the appEUI (an Arduino string allocated by caller).
 * @retval None.
 */
void LoRaWANNodeClass::getAppEUI(String *str)
{
  if(str != NULL) {
    uint8_t iKey[8];
    char cKey[17] = {'\0'};
    LoRa_GetAppID(iKey);
    keyIntToChar(cKey, iKey, 8);
    str->concat(cKey);
  }
}

/*
 * @brief  Write the LoRaWAN band. The device will be restarted.
 * @param  frequency band: LORA_BAND_EU_868 or LORA_BAND_US_915
 * @retval Error code: false if failed else true.
 */
bool LoRaWANNodeClass::setBand(uint8_t band)
{
  if(IS_BAND(band)) {
    if(Lora_SetDeviceBand(band) == AT_OK) {
      if(Lora_UpdateConfigTable() == AT_OK) {
        Lora_Reset();
        return 1;
      }
    }
  }
  return 0;
}

/*
 * @brief  Read the LoRaWAN band.
 * @retval Band: LORA_BAND_EU_868 or LORA_BAND_US_915
 */
uint8_t LoRaWANNodeClass::getBand(void)
{
  uint8_t band;
  Lora_GetDeviceBand(&band);
  return (band - 48); //Convert ascii to int
}

/*
 * @brief  Enable/disable the duty cycle. Must be enabled for EU band.
 * @param  true enables duty cycle, false disables it.
 * @retval Error code: false if failed else true.
 */
bool LoRaWANNodeClass::setDutyCycle(bool state)
{
  if(Lora_SetDutyCycle(state) == AT_OK) {
    return 1;
  }
  return 0;
}

/*
 * @brief  Read the duty cycle state.
 * @retval True if enabled else false.
 */
bool LoRaWANNodeClass::getDutyCycle(void)
{
  uint8_t state;
  Lora_GetDutyCycle(&state);
  return (bool)(state - 48); //Convert ascii to int
}

/*
 * @brief  Enable/disable the adaptative data rate.
 * @param  true enables ADR, false disables it.
 * @retval Error code: false if failed else true.
 */
bool LoRaWANNodeClass::setAdaptativeDataRate(bool state)
{
  if(Lora_SetAdaptiveDataRate((uint8_t)state) == AT_OK) {
    return 1;
  }
  return 0;
}

/*
 * @brief  Read the adaptative data rate state.
 * @retval True if enabled else false.
 */
bool LoRaWANNodeClass::getAdaptativeDataRate(void)
{
  uint8_t state;
  Lora_GetAdaptiveDataRate(&state);
  return (bool)(state - 48); //Convert ascii to int
}

/*
 * @brief  Reset the device parameters and restart it.
 */
void LoRaWANNodeClass::reset(void)
{
  Lora_RestoreConfigTable();
  Lora_Reset();
}

/*
 * @brief  Write the data rate.
 * @param  Data rate: 0 to 15.
 * @retval Error code: false if failed else true.
 */
bool LoRaWANNodeClass::setDataRate(uint8_t value)
{
  if(value <= 15) { //DR0 - DR15
    if(Lora_SetDataRate(value) == AT_OK) {
      return 1;
    }
  }
  return 0;
}

/*
 * @brief  Read the data rate.
 * @retval Returns the data rate: 0 to 15. If returns 256 the data rate can't be read.
 */
uint8_t LoRaWANNodeClass::getDataRate(void)
{
  uint8_t value[16]; // 0 (default)

  if(Lora_GetDataRate(value) == AT_OK) {
    return (value[0] - 48); // convert ascii to int
  }
  return 0xFF;
}

/*
 * @brief  Enable/disable the public network mode.
 * @param  True to enable the public network else false to switch in private network.
 * @retval Error code: false if failed else true.
 */
bool LoRaWANNodeClass::setPublicNwkMode(bool value)
{
  // 0 Private network - 1 Public network (default)
  if(Lora_SetPublicNetworkMode(value) == AT_OK) {
    return 1;
  }
  return 0;
}

/*
 * @brief  Read the network mode.
 * @retval Returns the network mode: 0 = private, 1 = public
 *         If returns 256 the mode can't be read.
 */
uint8_t LoRaWANNodeClass::getPublicNwkMode(void)
{
  uint8_t value[16]; // 0 (default)

  if(Lora_GetPublicNetworkMode(value) == AT_OK) {
    return (value[0] - 48); // convert ascii to int
  }
  return 0xFF;
}

/*
 * @brief  Force the device to enter in sleep mode.
 * @retval Error code: false if failed else true.
 */
bool LoRaWANNodeClass::sleep(void)
{
  if(Lora_SleepMode() == AT_OK) {
    delay(500);
    if(Modem_AT_Cmd(AT_ASYNC_EVENT, AT, NULL ) == AT_OK) {
      return 1;
    }
  }
  return 0;
}

/*
 * @brief  Force the device to exit the sleep mode.
 * @retval Error code: false if failed else true.
 */
bool LoRaWANNodeClass::wakeup(void)
{
  LoRa_DumyRequest();
  if(Modem_AT_Cmd(AT_ASYNC_EVENT, AT, NULL ) == AT_OK) {
    return 1;
  }
  return 0;
}

/*
 * @brief  Set the delay time of RX window 1.
 * @Note   Delay time of RX2 window is 1000ms larger than RX1 window.
 * @param  delay in ms. Should be a value of the LoRaWAN.
 * @retval Error code: false if failed else true.
 */
bool LoRaWANNodeClass::setRx1Delay(uint32_t ms)
{
  /* NOTE: RX2 window delay must be set before RX1 window delay if not RX1 delay
  is rejected. */
  if(ms >= getRx1Delay()) {
    if((LoRa_SetDelayRxWind(AT_RX2DL, ms + 1000) == AT_OK) &&
       (LoRa_SetDelayRxWind(AT_RX1DL, ms) == AT_OK)) {
      return 1;
    }
  } else {
    if((LoRa_SetDelayRxWind(AT_RX1DL, ms) == AT_OK) &&
    (LoRa_SetDelayRxWind(AT_RX2DL, ms + 1000) == AT_OK)) {
      return 1;
    }
  }
  return 0;
}

/*
 * @brief  Get the delay time of RX window 1.
 * @retval Returns the delay time of RX window 1 in ms.
 */
uint32_t LoRaWANNodeClass::getRx1Delay(void)
{
  uint32_t value;

  if(LoRa_GetDelayRxWind(AT_RX1DL, &value) == AT_OK) {
    return value;
  }
  return 0xFFFFFFFF;
}

/*
 * @brief  Set the join accept delay time of RX window 1.
 * @Note   Join Accept delay time of RX2 window is 1000ms larger than Join
 *         Accept RX1 window.
 * @param  delay in ms. Should be a value of the LoRaWAN.
 * @retval Error code: false if failed else true.
 */
bool LoRaWANNodeClass::setJoinRx1Delay(uint32_t ms)
{
  /* NOTE: RX2 window delay must be set before RX1 window delay if not RX1 delay
  is rejected. */
  if(ms >= getJoinRx1Delay()) {
    if((LoRa_SetJoinDelayRxWind(AT_JN2DL, ms + 1000) == AT_OK) &&
       (LoRa_SetJoinDelayRxWind(AT_JN1DL, ms) == AT_OK)) {
      return 1;
    }
  } else {
    if((LoRa_SetJoinDelayRxWind(AT_JN1DL, ms) == AT_OK) &&
    (LoRa_SetJoinDelayRxWind(AT_JN2DL, ms + 1000) == AT_OK)) {
      return 1;
    }
  }
  return 0;
}

/*
 * @brief  Get the join accept delay time of RX window 1.
 * @retval Returns the delay time of the join accept RX window 1 in ms.
 */
uint32_t LoRaWANNodeClass::getJoinRx1Delay(void)
{
  uint32_t value;

  if(LoRa_GetJoinDelayRxWind(AT_JN1DL, &value) == AT_OK) {
    return value;
  }
  return 0xFFFFFFFF;
}

LoRaWANNodeClass loraNode;
