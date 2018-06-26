/******************************************************************************
 * @file    hw_usart.c
 * @author  MCD Application Team
 * @version V1.1.2
 * @date    08-September-2017
 * @brief   This file provides code for the configuration of the USART
 *          instances.
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

/* Includes ------------------------------------------------------------------*/
#include "hw_usart.h"

static HardwareSerial *Serialx = NULL;

#ifdef __cplusplus
 extern "C" {
#endif

/* USART init function */

bool HW_UART_Modem_Init(void *serial, uint32_t BaudRate)
{
  if(serial != NULL) {
    Serialx = (HardwareSerial*)serial;
    Serialx->begin(BaudRate);
    return (true);
  }
  return (false);
}

/* USART deinit function */

void HW_UART_Modem_DeInit(void)
{
  if(Serialx != NULL) {
    Serialx->end();
    Serialx = NULL;
  }
}


/******************************************************************************
  * @brief To check if data has been received
  * @param none
  * @retval boolean: false no data / true data
******************************************************************************/
bool HW_UART_Modem_IsNewCharReceived(void)
{
  bool status;

  if(Serialx != NULL) {
    if(Serialx->available()) {
      status = true;
    } else {
      status = false;
    }
  } else {
    status = false;
  }
  return status;
}

/******************************************************************************
  * @brief Get the received character
  * @param none
  * @retval Return the data received
******************************************************************************/
uint8_t HW_UART_Modem_GetNewChar(void)
{
  if(Serialx != NULL) {
    return Serialx->read();
  } else {
    return 0;
  }
}

/******************************************************************************
  * @brief Send an among of character
  * @param buffer: data to send
  * @param len: number of data to send
  * @retval Return the data number of data sent
******************************************************************************/
uint8_t HW_UART_Modem_Write(uint8_t *buffer, uint16_t len)
{
  if(Serialx != NULL) {
    return Serialx->write(buffer, len);
  } else {
    return 0;
  }
}

/******************************************************************************
  * @brief  Flush serial buffer
  * @param  none
  * @retval none
******************************************************************************/
void HW_UART_Modem_Flush(void)
{
  if(Serialx != NULL) {
    while(Serialx->available()) {
      Serialx->read();
    }
  }
}

#ifdef __cplusplus
 }
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
