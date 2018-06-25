/******************************************************************************
 * @file    lora_driver.h based on V1.1.2
 * @author  MCD Application Team
 * @brief   Header for lora driver module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LORA_DRIVER_H__
#define __LORA_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "i_nucleo_lrwan1_wm_sg_sm_xx.h"
#include "hw.h"

/* Exported types ------------------------------------------------------------*/
typedef enum RetCode
{
  MODULE_READY,
  MODULE_NO_READY,
  MODULE_UART_ERROR,
} RetCode_t;

/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
extern bool AT_VERB_cmd;
/* Exported macros -----------------------------------------------------------*/

/* wait time to request Join status*/
#define DELAY_FOR_JOIN_STATUS_REQ    15000

/* LoRa network join mode */
#define  OTAA_JOIN_MODE                             1
#define  ABP_JOIN_MODE                              0

/* Payload size limitation */
#define MAX_PAYLOAD_LENGTH  64U


/* Exported functions ------------------------------------------------------- */

/************ connection management ************/

/**************************************************************
 * @brief  Check if the LoRa module is working
 * @param  void
 * @retval status of the module (ready or not ready)
**************************************************************/
RetCode_t Lora_Init(void);

/**************************************************************
 * @brief  reset of the LoRa module
 * @param  void
 * @retval void
**************************************************************/
void Lora_Reset(void);

/**************************************************************
 * @brief  Do a request to establish a LoRa Connection with the gateway
 * @param  Mode: by OTAA or by ABP
 * @retval LoRA return code
**************************************************************/
ATEerror_t Lora_Join(uint8_t Mode);

/**************************************************************
 * @brief  Wait for join accept notification either in ABP or OTAA
 * @param  void
 * @retval LoRA return code
 * @Nota this function supports either USI protocol or MDM32L07X01 protocol
**************************************************************/
ATEerror_t Lora_JoinAccept(void);

/**************************************************************
 * @brief  Do a request to set the Network join Mode
 * @param  Mode : OTA, ABP
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_SetJoinMode(uint8_t Mode);

/**************************************************************
 * @brief  Do a request to get the Network join Mode
 * @param  pointer to the Join mode out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetJoinMode(uint8_t *Mode);

/************ MiB management *******************/

/**************************************************************
 * @brief  key configuration
 * @param  KeyType : APPKEY, NWKSKE, APPSKEY
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_SetKey(ATCmd_t KeyType, uint8_t *PtrKey);


/**************************************************************
 * @brief  Request the key type configuration
 * @param  KeyType : APPKEY, NWKSKE, APPSKEY
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_GetKey(ATCmd_t KeyType, uint8_t *PtrKey);


/**************************************************************
 * @brief  Set the Application Identifier
 * @param  pointer to the APPEUI in value
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_SetAppID(uint8_t *PtrAppID);


/**************************************************************
 * @brief  Request the Application Identifier
 * @param  pointer to the APPEUI out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_GetAppID(uint8_t *AppEui);


/**************************************************************
 * @brief  Set the device extended universal indentifier
 * @param  pointer to the DEUI in value
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_SetDeviceID(uint8_t *PtrDeviceID);


/**************************************************************
 * @brief  Request the device extended universal indentifier
 * @param  pointer to the DEUI out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_GetDeviceID(uint8_t *PtrDeviceID);


/**************************************************************
 * @brief  Set the device address
 * @param  pointer to the DADDR in value
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_SetDeviceAddress(uint32_t DeviceAddr);


/**************************************************************
 * @brief  Request the device address
 * @param  pointer to the DADDR out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_GetDeviceAddress(uint32_t *DeviceAddr);


/**************************************************************
 * @brief  Set the NetWork ID
 * @param  NWKID in value
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_SetNetworkID(uint32_t NetworkID);


/**************************************************************
 * @brief  Request the network ID
 * @param  pointer to the NWKID out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_GetNetworkID(uint32_t *NetworkID);

       /************ Network Management ***************/

/**************************************************************
 * @brief  Do a request to set the adaptive data rate
 * @param  ADR in value 0(off) / 1(on)
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_SetAdaptiveDataRate(uint8_t Rate);


/**************************************************************
 * @brief  Do a request to get the adaptive data rate
 * @param  pointer to ADR out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetAdaptiveDataRate(uint8_t *Rate);


/**************************************************************
 * @brief  Do a request to set the LoRa Class
 * @param  CLASS in value [0,1,2]
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_SetClass(uint8_t Class);


/**************************************************************
 * @brief  Do a request to get the LoRa class
 * @param  pointer to CLASS out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetClass(uint8_t *Class);


/**************************************************************
 * @brief  Do a request to set the duty cycle
 * @brief  only used in test mode
 * @param  DCS in value 0(disable) / 1(enable)
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_SetDutyCycle(uint8_t DutyCycle);


/**************************************************************
 * @brief  Do a request to get the duty cycle
 * @param  pointer to DR out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetDutyCycle(uint8_t *DutyCycle);


/**************************************************************
 * @brief  Do a request to set the data Rate
 * @param  DR in value [0,1,2,3,4,5,6,7]
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_SetDataRate(uint8_t DataRate);


/**************************************************************
 * @brief  Do a request to get the data Rate
 * @param  pointer to DR out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetDataRate(uint8_t *DataRate);


/**************************************************************
 * @brief  Do a request to set the frame counter
 * @param  FrameCounterType : FCD, FCU
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_SetFrameCounter(ATCmd_t FrameCounterType,
                                uint32_t FrameCounternumber);


/**************************************************************
 * @brief  Request the frame counter number
 * @param  frameCounterType : FCD, FCU
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_GetFrameCounter(ATCmd_t FrameCounterType,
                                uint32_t *FrameCounternumber);


/**************************************************************
 * @brief  Do a request to set the join accept delay between
 * @brief  the end of the Tx and the join Rx#n window
 * @param  RxWindowType : JN1DL, JN2DL
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_SetJoinDelayRxWind(ATCmd_t RxWindowType,
                                   uint32_t JoinDelayInMs) ;

/**************************************************************
 * @brief  Do a request to get the join accept delay between
 * @brief  the end of the Tx and the join Rx#n window
 * @param  RxWindowType : JN1DL, JN2DL
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_GetJoinDelayRxWind(ATCmd_t FrameCounterType,
                                   uint32_t *JoinDelayInMs);


/**************************************************************
 * @brief  Do a request to set the Public Network mode
 * @param  PNM in value 0(off) / 1(on)
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_SetPublicNetworkMode(uint8_t NetworkMode);


/**************************************************************
 * @brief  Do a request to get the Public Network mode
 * @param  pointer to PNM out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetPublicNetworkMode(uint8_t *NetworkMode);


/**************************************************************
 * @brief  Do a request to set the delay between the end of the Tx
 * @brief  the end of the Tx and the join Rx#n window
 * @param  RxWindowType : RX1DL, RX2DL
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_SetDelayRxWind(ATCmd_t RxWindowType, uint32_t RxDelayInMs);


/**************************************************************
 * @brief  Do a request to get the delay between the end of the Tx
 * @brief  the end of the Tx and the join Rx#n window
 * @param  RxWindowType : RX1DL, RX2DL
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_GetDelayRxWind(ATCmd_t RxWindowType,uint32_t *RxDelayInMs);


/**************************************************************
 * @brief  Set the frequency of the Rx2 window
 * @param  pointer to the RX2FQ in value
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_SetFreqRxWind2(uint32_t Rx2WindFrequency);


/**************************************************************
 * @brief  Request the frequency of the Rx2 window
 * @param  pointer to the RX2FQ out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_GetFreqRxWind2(uint32_t *Rx2WindFrequency);


/**************************************************************
 * @brief  Do a request to set the transmit Tx power
 * @param  TXP in value [0,1,2,3,4,5]
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_SetTxPower(uint8_t TransmitTxPower);


/**************************************************************
 * @brief  Do a request to get the transmit Tx Power
 * @param  pointer to TXP out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetTxPower(uint8_t *TransmitTxPower);



/**************************************************************
 * @brief  Do a request to set the data Rate of Rx2 window
 * @param  RX2DR in value [0,1,2,3,4,5,6,7]
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_SetDataRateRxWind2(uint8_t DataRateRxWind2);


/**************************************************************
 * @brief  Do a request to get the data Rate of Rx2 window
 * @param  pointer to RX2DR out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetDataRateRxWind2(uint8_t *DataRateRxWind2);


      /************ Data Path Management ***************/

/**************************************************************
 * @brief  Send text data to a giving prot number
 * @param  SEND in data struct (ptrString,port)
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_SendData(sSendDataString_t *PtrStructData);


/**************************************************************
 * @brief  Do a request to get the last data (in raw format)
 * @brief  received by the Slave
 * @param  pointer to RECV out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_ReceivedData(sReceivedDataString_t *PtrStructData);


/**************************************************************
 * @brief  Trap an asynchronous event coming from external modem (only USI device)
 * @param  Pointer to RCV out value if any
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_AsyncDownLinkData(sReceivedDataBinary_t *PtrStructData);

/**************************************************************
 * @brief  Send binary data to a giving port number
 * @param  SENDB in value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_SendDataBin(sSendDataBinary_t *PtrStructData);


/**************************************************************
 * @brief  Do a request to get the last data (in binary format)
 * @brief  received by the Slave
 * @param  pointer to RECVB out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_ReceivedDataBin(sReceivedDataBinary_t *PtrStructData);

/**************************************************************
 * @brief  Do a request to set the confirmation mode
 * @param  CFM in value 0(unconfirm) / 1(confirm)
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_SetSendMsgConfirm(uint8_t ConfirmMode);

/**************************************************************
 * @brief  Do a request to get the confirmation mode
 * @param  pointer to CFM out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetSendMsgConfirm(uint8_t *ConfirmMode);


/**************************************************************
 * @brief  Do a request to get the msg status of the last send cmd
 * @param  CFS in value 0(unconfirm) / 1(confirm)
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetSendMsgStatus(uint8_t *MsgStatus);

/**************************************************************
 * @brief  Do a request to get the battery level of the modem (slave)
 * @param  BAT in value
 *              0:    battery connected to external power supply
 *       [1..254]:    1 being at minimum and 254 being at maximum
*             255:    not able to measure the battery level
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetBatLevel(uint32_t *BatLevel);


/**************************************************************
 * @brief  Do a request to get the RSSI of the last received packet
 * @param  RSSI in value [in dbm]
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetRSSI(int32_t *SigStrengthInd);

/**************************************************************
 * @brief  Do a request to get the SNR of the last received packet
 * @param  SNR in value [in db]
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetSNR(uint32_t *SigToNoice);


/**************************************************************
 * @brief  Do a request to set the country band code for LoRaWAN
 * @brief  Need to write to DCT and Reset module to enable this setting
 * @param  BAND in value 0(EU-868 Band) / 1(US-Band)
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_SetDeviceBand(uint8_t DeviceBand);


/**************************************************************
 * @brief  Do a request to get the country band code for LoRaWAN
 * @brief  only used in test mode
 * @param  pointer to BAND out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetDeviceBand(uint8_t *DeviceBand);


/**************************************************************
 * @brief  Do a request to get the firmware version of the modem (slave)
 * @param  pointer to VER out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetVersion(uint8_t *PtrVersion);


/**************************************************************
 * @brief  Do a request to get the firmware version of the modem (slave)
 * @param  pointer to FWVERSION out value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_GetFWVersion(uint8_t *PtrFWVersion);


/**************************************************************
 * @brief  Do a request to enter the slave in sleep (MCU STOP mode)
 * @param  Void
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_SleepMode(void);


/**************************************************************
 * @brief  Wait for mcu is going to sleep or is waked up
 * @param  void
 * @retval LoRA return code
**************************************************************/
ATEerror_t Lora_SleepStatus(void);


/**************************************************************
 * @brief  Do a request to set the power control settings of the MCU (slave)
 * @param  Power control IN value
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_SetMCUPowerCtrl(sPowerCtrlSet_t *PtrStructData);

/**************************************************************
 * @brief  Do a Dumy request to resynchronize the Host and the modem
 * @note   A simple AT cmd where we do not trap the return code
 * @param  void
 * @retval LoRa return code
**************************************************************/
ATEerror_t LoRa_DumyRequest(void);


              /************ DCT commands ***************/

/**************************************************************
 * @brief  Do a request to restore DCT content table with default values
 * @param  void
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_RestoreConfigTable(void);


/**************************************************************
 * @brief  Do a request to update the DCT content table with new values
 * @param  void
 * @retval LoRa return code
**************************************************************/
ATEerror_t Lora_UpdateConfigTable(void);

/**************************************************************
 * @brief  Convert keys from char to uint8_t
 * @param  Key to convert.
 * @param  Key converted.
 * @param  length of the integer key.
 **************************************************************/
void keyCharToInt(const char *cKey, uint8_t *iKey, uint8_t length);

/**************************************************************
 * @brief  Convert keys from uint8_t to char
 * @param  Key converted.
 * @param  Key to convert.
 * @param  length of the integer key.
 **************************************************************/
void keyIntToChar(char *cKey, const uint8_t *iKey, uint8_t length);

#ifdef __cplusplus
}
#endif

#endif /* __LORA_DRIVER_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
