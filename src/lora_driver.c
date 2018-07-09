/*******************************************************************************
 * @file    Lora_driver.c based on V1.1.2
 * @author  MCD Application Team
 * @brief   LoRa module API
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "hw.h"
#include "lora_driver.h"
#include "tiny_sscanf.h"

/* External variables --------------------------------------------------------*/
/*
 * Global flag to treat the return value of Lora_GetFWVersion() function
 * which is the only one that is not preceded by '=' charater.
 * This flag is used in the at_cmd_receive(..) function
 */
ATCmd_t gFlagException = AT_END_AT;
/*
 * Depending of firmware version AT+VERB is supported or not.
 * Introduced with fw 2.8, since this, some AT response format have changed.
 * This boolean will allow to handle those format.
 */
bool AT_VERB_cmd = true;

/* Private typedef -----------------------------------------------------------*/

/* Private variable ----------------------------------------------------------*/
/*
 * To get back the device address in 11:22:33:44 format before
 * to be translated into uint32_t type
 */
static uint8_t PtrValueFromDevice[32];
/* In relation with the response size */
static uint8_t PtrTempValueFromDeviceKey[64];
/* Payload size max returned by USI modem */
static uint8_t PtrDataFromNetwork[128];

/* Private define ------------------------------------------------------------*/

/******************************************************************************/
/*                    To put USI modem in sleep mode                          */
/* From USI FW V2.6, modem sleep mode is only supported on ABP Join mode      */
/* From USI FW V3.0, modem sleep mode is supported for ABP and OTAA Join mode */
/******************************************************************************/

/* Private functions ---------------------------------------------------------*/
static void memcpy1( uint8_t *dst, const uint8_t *src, uint16_t size );

/**************************************************************
 * @brief  Check if the LoRa module is working
 * @param  void
 * @retval status of the module (ready or not ready)
**************************************************************/
RetCode_t Lora_Init(void)
{
  ATEerror_t Status;

  /* Check if the module is working */
  Status = Modem_AT_Cmd(AT_CTRL, AT, NULL);

  if (Status == AT_OK) {
    /* Received Ok from module */
    return MODULE_READY;
  } else {
    return MODULE_NO_READY;
  }
}

/**************************************************************
 * @brief  reset of the LoRa module
 * @param  void
 * @retval void
**************************************************************/
void Lora_Reset(void)
{
  /* Reset the lora module */
  Modem_AT_Cmd(AT_CTRL, AT_RESET, NULL);
}

/**************************************************************
 * @brief  Do a request to establish a LoRa Connection with the gateway
 * @param  Mode: by OTAA or by ABP
 * @retval LoRA return code
 * @Nota param is relevant for USI WM_SG_SM_XX modem - Not relevant for MDM32L07X01 modem
**************************************************************/
ATEerror_t Lora_Join(uint8_t Mode)
{
  ATEerror_t Status = AT_END_ERROR;

  /******************************************************************/
  /* In OTAA mode wait JOIN_ACCEPT_DELAY1 cf. LoRaWAN Specification */
  /* MDM32L07X01:                                                   */
  /*      - After Join request waits DELAY_FOR_JOIN_STATUS_REQ      */
  /*      - Then do Join status request to know is nwk joined       */
  /* WM_SG_SM_XX:                                                   */
  /*      - Do the Join request                                     */
  /*      - Then waits synchronous JoinAccept event                 */
  /*      - if timeout raised before JoinAccept event               */
  /*      - then Join request Failed                                */
  /* Nota: Lora_Join() does the join request                        */
  /*       afterwhat                                                */
  /*       Lora_JoinAccept() does the waiting on return event       */
  /******************************************************************/

  switch(Mode) {
    case ABP_JOIN_MODE:
      /* Request a join connection */
      Status = Modem_AT_Cmd(AT_SET, AT_JOIN, &Mode);
      break;
    case OTAA_JOIN_MODE:
      Status = Modem_AT_Cmd(AT_SET, AT_JOIN, &Mode);
      /* HW_EnterSleepMode( );*/
      if(Status == AT_OK)
      {
        Status = AT_JOIN_SLEEP_TRANSITION;  /* to go in low power mode idle loop*/
      }
      break;
    default:
      break;
  }
  return Status;
}

/**************************************************************
 * @brief  Wait for join accept notification either in ABP or OTAA
 * @param  void
 * @retval LoRA return code
 * @Nota this function supports either USI protocol or MDM32L07X01 protocol
**************************************************************/
ATEerror_t Lora_JoinAccept(void)
{
  /* Trap the asynchronous accept event (OTAA mode) coming from USI modem */
  return Modem_AT_Cmd(AT_ASYNC_EVENT, AT_JOIN, NULL);
}

/**************************************************************
 * @brief  Do a request to set the Network join Mode
 * @param  Mode : OTA, ABP
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_SetJoinMode(uint8_t Mode)
{
  /* Set the nwk Join mode */
  return Modem_AT_Cmd(AT_SET, AT_NJM, &Mode);
}

/**************************************************************
 * @brief  Do a request to get the Network join Mode
 * @param  pointer to the Join mode out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetJoinMode(uint8_t *Mode)
{
  /* Get the nwk Join mode */
  return Modem_AT_Cmd(AT_GET, AT_NJM, Mode);
}

/******************* MiB MananagementLora *********************/

/**************************************************************
 * @brief  key configuration
 * @param  KeyType : APPKEY, NWKSKE, APPSKEY
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_SetKey(ATCmd_t KeyType, uint8_t *PtrKey)
{
  /* Set a key type to the LoRa device */
  return Modem_AT_Cmd(AT_SET, KeyType, PtrKey);
}

/**************************************************************
 * @brief  Request the key type configuration
 * @param  KeyType : APPKEY, NWKSKE, APPSKEY
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_GetKey(ATCmd_t KeyType,uint8_t *PtrKey)
{
  ATEerror_t Status;

  /* Get the key type from the LoRa device */
  Status = Modem_AT_Cmd(AT_GET, KeyType, PtrTempValueFromDeviceKey);
  if (Status == 0) {
    uint32_t offset = 0;
    if(!AT_VERB_cmd) {
      offset = AT_FRAME_KEY_OFFSET;
    }
    AT_VSSCANF((char*)PtrTempValueFromDeviceKey+offset, AT_FRAME_KEY,
    &PtrKey[0], &PtrKey[1], &PtrKey[2], &PtrKey[3],
    &PtrKey[4], &PtrKey[5], &PtrKey[6], &PtrKey[7],
    &PtrKey[8], &PtrKey[9], &PtrKey[10], &PtrKey[11],
    &PtrKey[12], &PtrKey[13], &PtrKey[14], &PtrKey[15]);
  }
  return Status;
}

/**************************************************************
 * @brief  Set the Application Identifier
 * @param  pointer to the APPEUI in value
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_SetAppID(uint8_t *PtrAppID)
{
  return Modem_AT_Cmd(AT_SET, AT_APPEUI, PtrAppID);
}

/**************************************************************
 * @brief  Request the Application Identifier
 * @param  pointer to the APPEUI out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_GetAppID(uint8_t *AppEui)
{
  ATEerror_t Status;

  Status = Modem_AT_Cmd(AT_GET, AT_APPEUI, PtrTempValueFromDeviceKey);
  if (Status == 0) {
    if(AT_VERB_cmd) {
      AT_VSSCANF((char*)PtrTempValueFromDeviceKey,
               "%hhx,%hhx,%hhx,%hhx,%hhx,%hhx,%hhx,%hhx",
               &AppEui[0], &AppEui[1], &AppEui[2], &AppEui[3],
               &AppEui[4], &AppEui[5], &AppEui[6], &AppEui[7]);
    } else {
      AT_VSSCANF((char*)PtrTempValueFromDeviceKey,
               "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
               &AppEui[0], &AppEui[1], &AppEui[2], &AppEui[3],
               &AppEui[4], &AppEui[5], &AppEui[6], &AppEui[7]);
    }
  }
  return Status;
}

/**************************************************************
 * @brief  Set the device extended universal indentifier
 * @param  pointer to the DEUI in value
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_SetDeviceID(uint8_t *PtrDeviceID)
{
  return Modem_AT_Cmd(AT_SET, AT_DEUI, PtrDeviceID);
}

/**************************************************************
 * @brief  Request the device extended universal indentifier
 * @param  pointer to the DEUI out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_GetDeviceID(uint8_t *PtrDeviceID)
{
  ATEerror_t Status;

  Status = Modem_AT_Cmd(AT_GET, AT_DEUI, PtrTempValueFromDeviceKey);
  if (Status == 0) {
    if(AT_VERB_cmd) {
      AT_VSSCANF((char*)PtrTempValueFromDeviceKey,
               "%hhx,%hhx,%hhx,%hhx,%hhx,%hhx,%hhx,%hhx",
      &PtrDeviceID[0], &PtrDeviceID[1], &PtrDeviceID[2], &PtrDeviceID[3],
      &PtrDeviceID[4], &PtrDeviceID[5], &PtrDeviceID[6], &PtrDeviceID[7]);
    } else {
      AT_VSSCANF((char*)PtrTempValueFromDeviceKey,
               "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
      &PtrDeviceID[0], &PtrDeviceID[1], &PtrDeviceID[2], &PtrDeviceID[3],
      &PtrDeviceID[4], &PtrDeviceID[5], &PtrDeviceID[6], &PtrDeviceID[7]);
    }
  }
  return Status;
}

/**************************************************************
 * @brief  Set the device address
 * @param  pointer to the DADDR in value
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_SetDeviceAddress(uint32_t DeviceAddr)
{
  return Modem_AT_Cmd(AT_SET, AT_DADDR, &DeviceAddr);
}

/**************************************************************
 * @brief  Request the device address
 * @param  pointer to the DADDR out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_GetDeviceAddress(uint32_t *Value)
{
  ATEerror_t Status;

  Status = Modem_AT_Cmd(AT_GET, AT_DADDR, PtrValueFromDevice);
  if (Status == 0) {
    if(AT_VERB_cmd) {
      AT_VSSCANF((char*)PtrValueFromDevice, "%hhx,%hhx,%hhx,%hhx",
      &((unsigned char *)(Value))[3],
      &((unsigned char *)(Value))[2],
      &((unsigned char *)(Value))[1],
      &((unsigned char *)(Value))[0]);
    } else {
      AT_VSSCANF((char*)PtrValueFromDevice, "%hhx:%hhx:%hhx:%hhx",
      &((unsigned char *)(Value))[3],
      &((unsigned char *)(Value))[2],
      &((unsigned char *)(Value))[1],
      &((unsigned char *)(Value))[0]);
    }
  }
  return Status;
}

/**************************************************************
 * @brief  Set the NetWork ID
 * @param  NWKID in value
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_SetNetworkID(uint32_t NetworkID)
{
  return Modem_AT_Cmd(AT_SET, AT_NWKID, &NetworkID);
}

/**************************************************************
 * @brief  Request the network ID
 * @param  pointer to the NWKID out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_GetNetworkID(uint32_t *Value)
{
  ATEerror_t Status;

  Status = Modem_AT_Cmd(AT_GET, AT_NWKID, PtrValueFromDevice);
  if (Status == 0) {
      AT_VSSCANF((char*)PtrValueFromDevice, "%hhx:%hhx:%hhx:%hhx",
      &((unsigned char *)(Value))[0],
      &((unsigned char *)(Value))[1],
      &((unsigned char *)(Value))[2],
      &((unsigned char *)(Value))[3]);
  }
  return Status;
}

/*******************   Network Management *********************/

/**************************************************************
 * @brief  Do a request to set the adaptive data rate
 * @param  ADR in value 0(off) / 1(on)
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_SetAdaptiveDataRate(uint8_t Rate)
{
  return Modem_AT_Cmd(AT_SET, AT_ADR, &Rate);
}

/**************************************************************
 * @brief  Do a request to get the adaptive data rate
 * @param  pointer to ADR out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetAdaptiveDataRate(uint8_t *Rate)
{
  return Modem_AT_Cmd(AT_GET, AT_ADR, Rate);
}

/**************************************************************
 * @brief  Do a request to set the LoRa Class
 * @param  CLASS in value [0,1,2]
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_SetClass(uint8_t Class)
{
  return Modem_AT_Cmd(AT_SET, AT_CLASS, &Class);
}

/**************************************************************
 * @brief  Do a request to get the LoRa class
 * @param  pointer to CLASS out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetClass(uint8_t *Class)
{
  return Modem_AT_Cmd(AT_GET, AT_CLASS, Class);
}

/**************************************************************
 * @brief  Do a request to set the duty cycle
 * @brief  only used in test mode
 * @param  DCS in value 0(disable) / 1(enable)
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_SetDutyCycle(uint8_t DutyCycle)
{
  return Modem_AT_Cmd(AT_SET, AT_DCS, &DutyCycle);
}

/**************************************************************
 * @brief  Do a request to get the duty cycle
 * @brief  only used in test mode
 * @param  pointer to DCS out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetDutyCycle(uint8_t *DutyCycle)
{
  return Modem_AT_Cmd(AT_GET, AT_DCS, DutyCycle);
}

/**************************************************************
 * @brief  Do a request to set the data Rate
 * @param  DR in value [0,1,2,3,4,5,6,7]
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_SetDataRate(uint8_t DataRate)
{
  return Modem_AT_Cmd(AT_SET, AT_DR, &DataRate);
}

/**************************************************************
 * @brief  Do a request to get the data Rate
 * @param  pointer to DR out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetDataRate(uint8_t *DataRate)
{
  return Modem_AT_Cmd(AT_GET, AT_DR, DataRate);
}

/**************************************************************
 * @brief  Do a request to set the frame counter
 * @param  FrameCounterType : FCD, FCU
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_SetFrameCounter(ATCmd_t FrameCounterType, uint32_t FrameCounternumber)
{
  return Modem_AT_Cmd(AT_SET, FrameCounterType, &FrameCounternumber);
}

/**************************************************************
 * @brief  Request the frame counter number
 * @param  frameCounterType : FCD, FCU
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_GetFrameCounter(ATCmd_t FrameCounterType,uint32_t *FrameCounternumber)
{
  ATEerror_t Status;

  Status = Modem_AT_Cmd(AT_GET, FrameCounterType, PtrValueFromDevice);
  if (Status == 0) {
    AT_VSSCANF((char*)PtrValueFromDevice, "%lu",FrameCounternumber);
  }
  return Status;
}

/**************************************************************
 * @brief  Do a request to set the join accept delay between
 * @brief  the end of the Tx and the join Rx#n window
 * @param  RxWindowType : JN1DL, JN2DL
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_SetJoinDelayRxWind(ATCmd_t RxWindowType, uint32_t JoinDelayInMs)
{
  return Modem_AT_Cmd(AT_SET, RxWindowType, &JoinDelayInMs);
}

/**************************************************************
 * @brief  Do a request to get the join accept delay between
 * @brief  the end of the Tx and the join Rx#n window
 * @param  RxWindowType : JN1DL, JN2DL
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_GetJoinDelayRxWind(ATCmd_t RxWindowType,uint32_t *JoinDelayInMs)
{
  ATEerror_t Status;

  Status = Modem_AT_Cmd(AT_GET, RxWindowType, PtrValueFromDevice);
  if (Status == 0) {
    AT_VSSCANF((char*)PtrValueFromDevice, "%lu",JoinDelayInMs);
  }
  return Status;
}

/**************************************************************
 * @brief  Do a request to set the Public Network mode
 * @param  PNM in value 0(off) / 1(on)
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_SetPublicNetworkMode(uint8_t NetworkMode)
{
  return Modem_AT_Cmd(AT_SET, AT_PNM, &NetworkMode);
}

/**************************************************************
 * @brief  Do a request to get the Public Network mode
 * @param  pointer to PNM out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetPublicNetworkMode(uint8_t *NetworkMode)
{
  return Modem_AT_Cmd(AT_GET, AT_PNM, NetworkMode);
}

/**************************************************************
 * @brief  Do a request to set the delay between the end of the Tx
 * @brief  the end of the Tx and the join Rx#n window
 * @param  RxWindowType : RX1DL, RX2DL
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_SetDelayRxWind(ATCmd_t RxWindowType, uint32_t RxDelayInMs)
{
  return Modem_AT_Cmd(AT_SET, RxWindowType, &RxDelayInMs);
}

/**************************************************************
 * @brief  Do a request to get the delay between the end of the Tx
 * @brief  the end of the Tx and the join Rx#n window
 * @param  RxWindowType : RX1DL, RX2DL
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_GetDelayRxWind(ATCmd_t RxWindowType,uint32_t *RxDelayInMs)
{
  ATEerror_t Status;

  Status = Modem_AT_Cmd(AT_GET, RxWindowType, PtrValueFromDevice);
  if (Status == 0) {
    AT_VSSCANF((char*)PtrValueFromDevice, "%lu",RxDelayInMs);
  }
  return Status;
}

/**************************************************************
 * @brief  Set the frequency of the Rx2 window
 * @param  pointer to the RX2FQ in value
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_SetFreqRxWind2(uint32_t Rx2WindFrequency)
{
  return Modem_AT_Cmd(AT_SET, AT_RX2FQ, &Rx2WindFrequency);
}

/**************************************************************
 * @brief  Request the frequency of the Rx2 window
 * @param  pointer to the RX2FQ out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_GetFreqRxWind2(uint32_t *Rx2WindFrequency)
{
  ATEerror_t Status;

  Status = Modem_AT_Cmd(AT_GET, AT_RX2FQ, PtrValueFromDevice);
  if (Status == 0) {
    AT_VSSCANF((char*)PtrValueFromDevice, "%lu",Rx2WindFrequency);
  }
  return Status;
}


/**************************************************************
 * @brief  Do a request to set the transmit Tx power
 * @param  TXP in value [0,1,2,3,4,5]
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_SetTxPower(uint8_t TransmitTxPower)
{
  return Modem_AT_Cmd(AT_SET, AT_TXP, &TransmitTxPower);
}

/**************************************************************
 * @brief  Do a request to get the transmit Tx Power
 * @param  pointer to TXP out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetTxPower(uint8_t *TransmitTxPower)
{
  return Modem_AT_Cmd(AT_GET, AT_TXP, TransmitTxPower);
}

/**************************************************************
 * @brief  Do a request to set the data Rate of Rx2 window
 * @param  RX2DR in value [0,1,2,3,4,5,6,7]
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_SetDataRateRxWind2(uint8_t DataRateRxWind2)
{
  return Modem_AT_Cmd(AT_SET, AT_RX2DR, &DataRateRxWind2);
}

/**************************************************************
 * @brief  Do a request to get the data Rate of Rx2 window
 * @param  pointer to RX2DR out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetDataRateRxWind2(uint8_t *DataRateRxWind2)
{
  return Modem_AT_Cmd(AT_GET, AT_RX2DR, DataRateRxWind2);
}

/****************** Data Path Management **********************/

/**************************************************************
 * @brief  Send text data to a giving prot number
 * @param  SEND in value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_SendData(sSendDataString_t *PtrStructData)
{
  return Modem_AT_Cmd(AT_SET, AT_SEND, PtrStructData);
}

/**************************************************************
 * @brief  Do a request to get the last data (in raw format)
 * @brief  received by the Slave
 * @param  pointer to RECV out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_ReceivedData(sReceivedDataString_t *PtrStructData)
{
  ATEerror_t Status;
  uint8_t sizebuf;

  Status = Modem_AT_Cmd(AT_GET, AT_RECV, PtrValueFromDevice);
  if (Status == 0) {
    AT_VSSCANF((char*)PtrValueFromDevice, "%d", &(PtrStructData->Port));
    if ((sizebuf=strlen((char*)&PtrValueFromDevice[3])) > DATA_RX_MAX_BUFF_SIZE) {
     /* Shrink the Rx buffer to MAX size */
      sizebuf = DATA_RX_MAX_BUFF_SIZE -1;
    }
    memcpy1(PtrStructData->Buffer, (uint8_t *)&PtrValueFromDevice[3], sizebuf+1);
  }
  return Status;
}

/**************************************************************
 * @brief  Trap an asynchronous event coming from external modem (only USI device)
 * @param  Pointer to RCV out value if any
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_AsyncDownLinkData(sReceivedDataBinary_t *PtrStructData)
{
  ATEerror_t Status;
  uint8_t sizebuf;
  char *ptrChr;

  Status = Modem_AT_Cmd(AT_ASYNC_EVENT, AT_END_AT, PtrDataFromNetwork);
  if (Status == 0) {
    AT_VSSCANF((char*)PtrDataFromNetwork, "%d,%2d",
               &(PtrStructData->Port),&(PtrStructData->DataSize));
    /* Search the last ',' occurence in the return string */
    ptrChr = strrchr((char*)&PtrDataFromNetwork[0], ',')+1;
	if ((sizebuf=strlen((char*)ptrChr)) > DATA_RX_MAX_BUFF_SIZE) {
      /* Shrink the Rx buffer to MAX size */
      sizebuf = DATA_RX_MAX_BUFF_SIZE -1;
    }
    /* Prevent a memory overflow in case of corrupted read */
    if(sizebuf == 0) {
      return AT_TEST_PARAM_OVERFLOW;
    }
    memcpy1(PtrStructData->Buffer, (uint8_t *)ptrChr, sizebuf);
    *(PtrStructData->Buffer+sizebuf) ='\0';
  }
  return Status;
}

/**************************************************************
 * @brief  Send binary data to a giving port number
 * @param  SENDB in value ( USE_MDM32L07X01) SEND in value ( USE_I_NUCLEO_LRWAN1)
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_SendDataBin(sSendDataBinary_t *PtrStructData)
{
  /* Remove all old data in the uart rx buffer to prevent response issue */
  HW_UART_Modem_Flush();
  return Modem_AT_Cmd(AT_SET, AT_SEND, PtrStructData);
}

/**************************************************************
 * @brief  Do a request to get the last data (in binary format)
 * @brief  received by the Slave
 * @param  pointer to RECVB out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_ReceivedDataBin(sReceivedDataBinary_t *PtrStructData)
{
  ATEerror_t Status;
  uint8_t sizebuf;
  uint8_t i;
  char TempBuf[3] ={0};

  Status = Modem_AT_Cmd(AT_GET, AT_RECVB, PtrValueFromDevice);
  if (Status == 0) {
    AT_VSSCANF((char*)PtrValueFromDevice, "%d",&(PtrStructData->Port));

    if ((sizebuf= strlen((char*)&PtrValueFromDevice[3])) > DATA_RX_MAX_BUFF_SIZE) {
      /* Shrink the Rx buffer to MAX size */
      sizebuf = DATA_RX_MAX_BUFF_SIZE;
    }
    for(i=0; i<=((sizebuf/2)-1); i++) {
      TempBuf[0] = PtrValueFromDevice[3+(i*2)];
      TempBuf[1] = PtrValueFromDevice[3+(i*2)+1];
      AT_VSSCANF(TempBuf,"%hhx", &PtrStructData->Buffer[i]);
    }
    PtrStructData->DataSize = i;
  }
  return Status;
}

/**************************************************************
 * @brief  Do a request to set the confirmation mode
 * @param  CFM in value 0(unconfirm) / 1(confirm)
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_SetSendMsgConfirm(uint8_t ConfirmMode)
{
  return Modem_AT_Cmd(AT_SET, AT_CFM, &ConfirmMode);
}

/**************************************************************
 * @brief  Do a request to get the confirmation mode
 * @param  pointer to CFM out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetSendMsgConfirm(uint8_t *ConfirmMode)
{
  return Modem_AT_Cmd(AT_GET, AT_CFM, ConfirmMode);
}

/**************************************************************
 * @brief  Do a request to get the msg status of the last send cmd
 * @param  CFS in value 0(unconfirm) / 1(confirm)
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetSendMsgStatus(uint8_t *MsgStatus)
{
  return Modem_AT_Cmd(AT_GET, AT_CFS, MsgStatus);
}

/**************************************************************
 * @brief  Do a request to get the battery level of the modem (slave)
 * @param  BAT in value
 *              0:    battery connected to external power supply
 *       [1..254]:    1 being at minimum and 254 being at maximum
*             255:    not able to measure the battery level
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetBatLevel(uint32_t *BatLevel)
{
  ATEerror_t Status;

  Status = Modem_AT_Cmd(AT_GET, AT_BAT, PtrValueFromDevice);
  if (Status == 0) {
    AT_VSSCANF((char*)PtrValueFromDevice, "%ld", BatLevel);
  }
  return Status;
}

/**************************************************************
 * @brief  Do a request to get the RSSI of the last received packet
 * @param  RSSI in value [in dbm]
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetRSSI(int32_t *SigStrengthInd)
{
  ATEerror_t Status;

  Status = Modem_AT_Cmd(AT_GET, AT_RSSI, PtrValueFromDevice);
  if (Status == 0) {
    AT_VSSCANF((char*)PtrValueFromDevice, "%ld", SigStrengthInd);
  }
  return (Status);
}


/**************************************************************
 * @brief  Do a request to get the SNR of the last received packet
 * @param  SNR in value [in db]
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetSNR(uint32_t *SigToNoice)
{
  ATEerror_t Status;

  Status = Modem_AT_Cmd(AT_GET, AT_SNR, PtrValueFromDevice);
  if (Status == 0) {
    AT_VSSCANF((char*)PtrValueFromDevice, "%ld", SigToNoice);
  }
  return Status;
}

/**************************************************************
 * @brief  Do a request to get the LoRa stack version of the modem (slave)
 * @param  pointer to VER out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetVersion(uint8_t *PtrVersion)
{
  ATEerror_t Status;
  char *ptrChr;

  Status = Modem_AT_Cmd(AT_GET, AT_VER, PtrValueFromDevice);
  if (Status == 0) {
    if(AT_VERB_cmd) {
      /* Skip fw version and model */
      ptrChr = strchr((char *)&PtrValueFromDevice[0], ',');
      strncpy((char*)PtrVersion, (char *)PtrValueFromDevice,
              ptrChr - (char *)PtrValueFromDevice);
	} else {
      /* Skip "LoRaWAN v" */
      ptrChr = strchr((char *)&PtrValueFromDevice[0], 'v');
      strcpy((char*)PtrVersion,ptrChr+1);
    }
  }
  return Status;
}

/**************************************************************
 * @brief  Do a request to get the firmware version of the modem (slave)
 * @param  pointer to FWVERSION out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetFWVersion(uint8_t *PtrFWVersion)
{
  ATEerror_t Status;
  char *ptrChr;

  gFlagException = AT_FWVERSION;
  Status = Modem_AT_Cmd(AT_GET, AT_FWVERSION, PtrValueFromDevice);
  if (Status == 0) {
    /* Skip "USI Lora Module Firmware V" prefix */
    ptrChr = strchr((char *)&PtrValueFromDevice[0],'V');
    strcpy((char*)PtrFWVersion,ptrChr+1);
  } else {
    /*
     * Since AT+VERB introduced, Fw version is integrated to AT+VER
     * <lrwan_ver>,<fw_ver>
     * ATI is marked as removed but always available.
     */
    if(AT_VERB_cmd) {
      Status = Modem_AT_Cmd(AT_GET, AT_VER, PtrValueFromDevice);
      if (Status == 0) {
        char *ptrChr2;
        /* Skip LoRaWan version and strip model */
        ptrChr = strchr((char *)&PtrValueFromDevice[0], ',');
        ptrChr2 = strchr(ptrChr+1,',');
        if (ptrChr2 == NULL) {
          strcpy((char*)PtrFWVersion, ptrChr+1);
        } else {
          strncpy((char*)PtrFWVersion, ptrChr+1, ptrChr2 - ptrChr - 1);
          PtrFWVersion[ptrChr2 - ptrChr - 1] = '\0';
        }
      }
    }
  }
  return Status;
}

/**************************************************************
 * @brief  Do a request to set the country band code for LoRaWAN
 * @brief  Need to write to DCT and Reset module to enable this setting
 * @param  BAND in value 0(EU-868 Band) / 1(US-Band)
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_SetDeviceBand(uint8_t DeviceBand)
{
  return Modem_AT_Cmd(AT_SET, AT_BAND, &DeviceBand);
}

/**************************************************************
 * @brief  Do a request to get the country band code for LoRaWAN
 * @brief  only used in test mode
 * @param  pointer to BAND out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetDeviceBand(uint8_t *DeviceBand)
{
  return Modem_AT_Cmd(AT_GET, AT_BAND, DeviceBand);
}

/************ Power Control Commands (for USI board) ***************/

/**************************************************************
 * @brief  Do a request to enter the slave in sleep (MCU STOP mode)
 * @param  Void
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_SleepMode(void)
{
  /* Under building*/
  return Modem_AT_Cmd(AT_EXCEPT_1, AT_SLEEP, PtrValueFromDevice);
}

/**************************************************************
 * @brief  Wait for mcu is going to sleep or is waked up
 * @param  void
 * @retval LoRA return code
**************************************************************/
ATEerror_t Lora_SleepStatus(void)
{
  /* Trap the asynchronous accept event coming from USI modem */
  return Modem_AT_Cmd(AT_ASYNC_EVENT, AT, NULL);
}

/**************************************************************
 * @brief  Do a request to set the power control settings of the MCU (slave)
 * @param  Power control IN value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_SetMCUPowerCtrl(sPowerCtrlSet_t *PtrStructData)
{
  return Modem_AT_Cmd(AT_SET, AT_PS, PtrStructData);
}

/**************************************************************
 * @brief  Do a Dumy request to resynchronize the Host and the modem
 * @note   A simple AT cmd where we do not trap the return code
 * @param  void
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_DumyRequest(void)
{
  ATEerror_t Status;
  uint8_t i;

  for (i=0; i<=1; i++)
  {
    /* First iteration to wake-up the modem */
    /* Ssecond iteration to align Host-Modem interface */
    Status = Modem_AT_Cmd(AT_EXCEPT_1, AT, NULL);
    /* Assumption: to be sure that modem is ready */
    delay(1000);
  }
  return(Status);
}

/**************************************************************
 * @brief  Do a request to restore DCT content table with default values
 * @param  void
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_RestoreConfigTable(void)
{
  uint8_t Restore = 0;

  return Modem_AT_Cmd(AT_SET, AT_WDCT, &Restore);
}

/**************************************************************
 * @brief  Do a request to update the DCT content table with new values
 * @param  void
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_UpdateConfigTable(void)
{
  return Modem_AT_Cmd(AT_GET, AT_WDCT, NULL );
}

/**************************************************************
 * @brief  memory copy n bytes from src to dst
 * @param  destination pointer
 * @param  source pointer
 * @param  number of bytes to copy
 * @retval none
**************************************************************/
void memcpy1( uint8_t *dst, const uint8_t *src, uint16_t size )
{
  while( size-- ) {
    *dst++ = *src++;
  }
}

/**************************************************************
 * @brief  Convert keys from char to uint8_t
 * @param  Key to convert.
 * @param  Key converted.
 * @param  length of the integer key.
 **************************************************************/
void keyCharToInt(const char *cKey, uint8_t *iKey, uint8_t length)
{
  if((cKey != NULL) && (iKey != NULL)) {
    char c[3] = {'\0'};
    uint8_t p = 0;
    for(uint8_t i = 0; i < length; i++) {
      c[0] = cKey[p];
      c[1] = cKey[p+1];
      iKey[i] = (uint8_t)strtoul(c, NULL, 16);
      p += 2;
    }
  }
}

/**************************************************************
 * @brief  Convert keys from uint8_t to char
 * @param  Key converted.
 * @param  Key to convert.
 * @param  length of the integer key.
 **************************************************************/
void keyIntToChar(char *cKey, const uint8_t *iKey, uint8_t length)
{
  if((cKey != NULL) && (iKey != NULL)) {
    uint8_t p = 0;
    for(uint8_t i = 0; i < length; i++) {
      itoa(iKey[i], &cKey[p], 16);
      /* Add missing '0' */
      if(cKey[p+1] == '\0') {
        cKey[p+1] = cKey[p];
        cKey[p] = '0';
      }
      p += 2;
    }
  }
}

#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
