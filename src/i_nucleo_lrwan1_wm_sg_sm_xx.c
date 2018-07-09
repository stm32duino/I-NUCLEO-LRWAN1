 /*******************************************************************************
  * @file    i_nucleo_lrwan1_wm_sg_sm_xx.c based on V1.0.1
  * @author  MCD Application Team
  * @brief   driver I_NUCLEO_LRWAN1 for WM_SG_SM_XX modem board
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

/* Includes ------------------------------------------------------------------*/

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "i_nucleo_lrwan1_wm_sg_sm_xx.h"
#include "hw.h"

#include <stdarg.h>

/* External variables --------------------------------------------------------*/
extern ATCmd_t gFlagException;  /* Defined in lora_driver.c */

/* Private variables ---------------------------------------------------------*/

/* Buffer used for AT cmd transmission */
char LoRa_AT_Cmd_Buff[DATA_TX_MAX_BUFF_SIZE];
/* Write position needed for send command */
static uint16_t Offset = 0;
/*
 * Buffer has to be the largest of the response not only for return code
 * but also for return value: exemple KEY
 */
static char response[DATA_RX_MAX_BUFF_SIZE];

/****************************************************************************/
/*here we have to include a list of AT cmd by the way of #include<file>     */
/*this file will be preprocessed for CmdTab and ATE_RetCode definition      */
/****************************************************************************/

/* Avoid recursive include */
#undef  __ATCMD_MODEM_H__
#define AT_CMD_STRING
#define AT_ERROR_STRING
#undef  AT_CMD_INDEX
#undef  AT_ERROR_INDEX
/* Include WM_SG_SM_42 specific string AT cmd definition */
#include "atcmd_modem.h"

/* private functions ------------------------------------------------------- */
static uint8_t at_cmd_format(ATCmd_t Cmd, void *ptr, Marker_t Marker);

static bool at_cmd_send(uint16_t len);

static ATEerror_t at_cmd_receive(void *pdata);

static ATEerror_t at_cmd_responseAnalysing(const char *ReturnResp);

static ATEerror_t at_cmd_receive_async_event(void);

static ATEerror_t at_cmd_AsyncEventAnalysing(const char *ReturnResp,int8_t *Flag);

static ATEerror_t at_cmd_receive_async_event_downlink_data(void *ptr);

/******************************************************************************
 * @brief  Configures modem UART interface.
 * @param  None
 * @retval AT_OK in case of success
 * @retval AT_UART_LINK_ERROR in case of failure
*****************************************************************************/
ATEerror_t Modem_IO_Init( void *serial )
{
  if (HW_UART_Modem_Init(serial, BAUD_RATE)) {
    return AT_OK;
  } else {
    return AT_UART_LINK_ERROR;
  }
}

/******************************************************************************
 * @brief  Deinitialise modem UART interface.
 * @param  None
 * @retval None
*****************************************************************************/
void Modem_IO_DeInit( void )
{
  HW_UART_Modem_DeInit();
}


/******************************************************************************
 * @brief  Handle the AT cmd following their Groupp type
 * @param  at_group AT group [control, set , get)
 *         Cmd AT command
 *         pdata pointer to the IN/OUT buffer
 * @retval module status
 *****************************************************************************/
ATEerror_t  Modem_AT_Cmd(ATGroup_t at_group, ATCmd_t Cmd, void *pdata )
{
  ATEerror_t Status = AT_END_ERROR;
  uint16_t Len;

  /* Reset At_cmd buffer for each transmission */
  memset(LoRa_AT_Cmd_Buff, 0x00, sizeof LoRa_AT_Cmd_Buff);

  switch (at_group) {
    case AT_CTRL:
      Len = at_cmd_format( Cmd, NULL, CTRL_MARKER);
      if(!at_cmd_send(Len)) {
        return (AT_UART_LINK_ERROR);
      }
      if(Cmd != AT_RESET) {
        Status = at_cmd_receive(NULL);
      }
      break;
    case AT_SET:
      Len = at_cmd_format(Cmd, pdata, SET_MARKER);
      if(!at_cmd_send(Len)) {
        return (AT_UART_LINK_ERROR);
      }
      Status = at_cmd_receive(NULL);
      break;
    case AT_GET:
      Len = at_cmd_format(Cmd, pdata, GET_MARKER);
      if(!at_cmd_send(Len)) {
        return (AT_UART_LINK_ERROR);
      }
      Status = at_cmd_receive(pdata);
      break;
    case AT_ASYNC_EVENT:
      if (( Cmd == AT_JOIN) || (Cmd == AT) || (Cmd == AT_TXT)) {
        Status = at_cmd_receive_async_event();
      } else {
        Status = at_cmd_receive_async_event_downlink_data(pdata);
      }
      break;
    case AT_EXCEPT:
      Len = at_cmd_format(Cmd, pdata, SET_MARKER);
       if(!at_cmd_send(Len)) {
        return (AT_UART_LINK_ERROR);
      } else {
        Status = at_cmd_receive(NULL);
      }
      break;
    case AT_EXCEPT_1:
      Len = at_cmd_format(Cmd, NULL, SET_MARKER);
      if(!at_cmd_send(Len)) {
        return (AT_UART_LINK_ERROR);
	  } else {
        Status = at_cmd_receive(NULL);
      }
      break;
    default:
      DBG_PRINTF("unknow group\n\r");
      break;
  } /*end switch(at_group)*/
  return Status;
}

/******************************************************************************
 * @brief  format the cmd in order to be send
 * @param  Cmd AT command
 *         ptr generic pointer to the IN/OUT buffer
 *         Marker to discriminate the Set from the Get
 * @retval length of the formated frame to be send
 *****************************************************************************/
static uint8_t at_cmd_format(ATCmd_t Cmd, void *ptr, Marker_t Marker)
{
  uint16_t len;      /* Length of the formated command */
  uint8_t *PtrValue; /* For IN/OUT buffer */
  uint32_t value;    /* For 32_02X and 32_D */
  uint8_t value_8;   /* For 8_D */

  switch (Cmd){
    case AT:         /* Supported */
    case AT_RESET:   /* Supported */
    case AT_SLEEP:   /* Supported */
    case AT_FWVERSION: /* Supported */
      /* Format = FORMAT_VOID_PARAM; */
      len = AT_VPRINTF("%s%s%s",AT_HEADER,CmdTab[Cmd],AT_TAIL);
      break;
    case  AT_RECV:
    case  AT_VER:
      /* Format = FORMAT_PLAIN_TEXT; */
      if(Marker == SET_MARKER) {
        len = AT_VPRINTF("%s%s%s%d%s%s\r\n", AT_HEADER, CmdTab[Cmd],
                         AT_SET_MARKER,((sSendDataString_t *)ptr)->Port,
                         AT_COLON,((sSendDataString_t *)ptr)->Buffer);
      } else {
        len = AT_VPRINTF("%s%s%s%s", AT_HEADER, CmdTab[Cmd],
                         AT_GET_MARKER,AT_TAIL);
      }
      break;
    case  AT_SEND:   /* Supported - sendB replaced by send - just one send mode on USI */
    case  AT_RECVB:  /* Not supported*/
      /* Format = FORMAT_BINARY_TEXT; */
      if(Marker == SET_MARKER) {
        Offset = AT_VPRINTF("%s%s%s%d%s", AT_HEADER, CmdTab[Cmd],
                            AT_SET_MARKER, ((sSendDataBinary_t *)ptr)->Port,
                            AT_COMMA);
        for (uint32_t i = 0; i < ((sSendDataBinary_t *)ptr)->DataSize; i++) {
          Offset+=AT_VPRINTF("%02x", ((sSendDataBinary_t *)ptr)->Buffer[i]);
        }
        Offset+=AT_VPRINTF("%s%d%s", AT_COMMA, ((sSendDataBinary_t *)ptr)->Ack,
                           AT_TAIL);
        len = Offset;
        Offset = 0;
      } else {
        len = AT_VPRINTF("%s%s%s%s", AT_HEADER, CmdTab[Cmd], AT_GET_MARKER,
                         AT_TAIL);
      }
      break;
    case AT_TXT:
      /* Format = FORMAT_BINARY_TEXT; */
      if(Marker == SET_MARKER) {
      Offset = AT_VPRINTF("%s%s%s%d%s", AT_HEADER, CmdTab[Cmd], AT_SET_MARKER,
                         ((sSendData_t *)ptr)->NbRep, AT_COMMA);
      for (uint32_t i = 0; i < ((sSendData_t *)ptr)->DataSize; i++) {
        Offset+=AT_VPRINTF("%02x", ((sSendData_t *)ptr)->Buffer[i]);
      }
        Offset+=AT_VPRINTF("%s", AT_TAIL);
        len = Offset;
        Offset = 0;
      } else {
        len = 0;
      }
      break;
    case AT_APPKEY:  /* Supported - USI equivalent AK */
    case AT_NWKSKEY: /* Supported - USI equivalent NSK */
    case AT_APPSKEY: /* Supported - USI equivalent ASK */
      if(Marker == SET_MARKER) {
        /* Format = FORMAT_16_02X_PARAM; */
        PtrValue = (uint8_t*) ptr;
        len = AT_VPRINTF(
  "%s%s%s%02x%s%02x%s%02x%s%02x%s%02x%s%02x%s%02x%s%02x%s%02x%s%02x%s%02x%s%02x%s%02x%s%02x%s%02x%s%02x%s",
                      AT_HEADER, CmdTab[Cmd], AT_SET_MARKER,
                      PtrValue[0], AT_SEPARATOR, PtrValue[1], AT_SEPARATOR,
                      PtrValue[2], AT_SEPARATOR, PtrValue[3], AT_SEPARATOR,
                      PtrValue[4], AT_SEPARATOR, PtrValue[5], AT_SEPARATOR,
                      PtrValue[6], AT_SEPARATOR, PtrValue[7], AT_SEPARATOR,
                      PtrValue[8], AT_SEPARATOR, PtrValue[9], AT_SEPARATOR,
                      PtrValue[10], AT_SEPARATOR, PtrValue[11], AT_SEPARATOR,
                      PtrValue[12], AT_SEPARATOR, PtrValue[13], AT_SEPARATOR,
                      PtrValue[14], AT_SEPARATOR, PtrValue[15], AT_TAIL);
      } else {
        len = AT_VPRINTF("%s%s%s%s", AT_HEADER, CmdTab[Cmd], AT_GET_MARKER,
                         AT_TAIL);
      }
      break;
    case AT_DADDR:   /* Supported */
    case AT_NWKID:   /* N/A */
      if(Marker == SET_MARKER) {
        /*Format = FORMAT_32_02X_PARAM;*/
        value =  *(uint32_t*)ptr;
        len = AT_VPRINTF("%s%s%s%02x%s%02x%s%02x%s%02x%s", AT_HEADER,
                         CmdTab[Cmd], AT_SET_MARKER,
                        (unsigned)((unsigned char *)(&value))[3], AT_SEPARATOR,
                        (unsigned)((unsigned char *)(&value))[2], AT_SEPARATOR,
                        (unsigned)((unsigned char *)(&value))[1], AT_SEPARATOR,
                        (unsigned)((unsigned char *)(&value))[0], AT_TAIL);
      } else {
        len = AT_VPRINTF("%s%s%s%s", AT_HEADER, CmdTab[Cmd], AT_GET_MARKER,
                         AT_TAIL);
      }
      break;
    case AT_APPEUI:  /* Supported*/
    case AT_DEUI:    /* USI equivalent EUI - not relevant for SET since burned unique IEEE EUI64 at factory. */
      if(Marker == SET_MARKER) {
        /*  Format = FORMAT_8_02X_PARAM;*/
        PtrValue = (uint8_t*)ptr;
        len = AT_VPRINTF(
			  "%s%s%s%02x%s%02x%s%02x%s%02x%s%02x%s%02x%s%02x%s%02x%s",
              AT_HEADER,CmdTab[Cmd],AT_SET_MARKER,
              PtrValue[0], AT_SEPARATOR, PtrValue[1], AT_SEPARATOR,
              PtrValue[2], AT_SEPARATOR, PtrValue[3], AT_SEPARATOR,
              PtrValue[4], AT_SEPARATOR, PtrValue[5], AT_SEPARATOR,
              PtrValue[6], AT_SEPARATOR, PtrValue[7], AT_TAIL);
      } else {
        len = AT_VPRINTF("%s%s%s%s", AT_HEADER, CmdTab[Cmd], AT_GET_MARKER,
                         AT_TAIL);
      }
      break;
    case  AT_RX2FQ:    /* N/A */
    case  AT_RX1DL:    /* Supported - USI equivalent RX1DT */
    case  AT_RX2DL:    /* Supported - USI equivalent RX2DT */
    case  AT_JN1DL:    /* Supported - USI equivalent JRX1DT */
    case  AT_JN2DL:    /* Supported - USI equivalent JRX2DT */
    case  AT_FCU:      /* Not supported */
    case  AT_FCD:      /* Not supported */
      /* Format = FORMAT_32_D_PARAM; */
      if(Marker == SET_MARKER) {
        value =  *(uint32_t*)ptr;
        len = AT_VPRINTF("%s%s%s%u%s", AT_HEADER, CmdTab[Cmd], AT_SET_MARKER,
                         value, AT_TAIL);
      } else {
        len = AT_VPRINTF("%s%s%s%s", AT_HEADER, CmdTab[Cmd], AT_GET_MARKER,
                         AT_TAIL);
      }
      break;
    case  AT_JOIN:     /* Supported */
    case  AT_ATE:      /* Supported */
    case  AT_DR:       /* Supported */
    case  AT_RX2DR:    /* Supported */
    case  AT_BAND:     /* Supported */
    case  AT_TXP:      /* N/A */
    case  AT_NJM:      /* N/A */
    case  AT_PNM:      /* Supported - USI equivalent NTYP */
    case  AT_DCS:      /* Supported - USI equivalent DC -
                        * Disabling duty cycle for testing only.
                        * It should be enabled for shipping
                        */
    case  AT_ADR:      /* Supported */
    case  AT_CFM:      /* N/A */
    case  AT_CFS:      /* N/A */
    case  AT_BAT:      /* Supported */
    case  AT_RSSI:     /* Not supported by USI FW version */
    case  AT_SNR:      /* Not supported by USI FW version */
    case  AT_NJS:      /* N/A */
    case  AT_CLASS:    /* Not supported on V2.5 USI FW version */
    case  AT_WDCT:     /* Supported */
    case  AT_DEFMODE:
    case  AT_VERB:     /* Supported since V2.8 USI FW version */
      /* Format = FORMAT_8_D_PARAM;*/
      if(Marker == SET_MARKER) {
        value_8 =  *(uint8_t*)ptr;
        len = AT_VPRINTF("%s%s%s%d%s", AT_HEADER, CmdTab[Cmd], AT_SET_MARKER,
                         value_8, AT_TAIL);
      } else {
        len = AT_VPRINTF("%s%s%s%s", AT_HEADER, CmdTab[Cmd], AT_GET_MARKER,
                         AT_TAIL);
      }
      break;
    case AT_PS:
      if(Marker == SET_MARKER) {
        len = AT_VPRINTF("%s%s%s%d,%d%s", AT_HEADER, CmdTab[Cmd],
                         AT_SET_MARKER, ((sPowerCtrlSet_t *)ptr)->SetType,
                         ((sPowerCtrlSet_t *)ptr)->Value, AT_TAIL);
      } else {
        len = AT_VPRINTF("%s%s%s%s", AT_HEADER, CmdTab[Cmd], AT_GET_MARKER, AT_TAIL);
      }
      break;
    case AT_RF:
      if(Marker == SET_MARKER) {
        len = AT_VPRINTF("%s%s%s%d,%d,%d,%d,%d,%d,%d,%d,%s", AT_HEADER,
                         CmdTab[Cmd], AT_SET_MARKER,
                         ((sRadioCtrlSet_t *)ptr)->power,
                         ((sRadioCtrlSet_t *)ptr)->frequency,
                         ((sRadioCtrlSet_t *)ptr)->sf,
                         ((sRadioCtrlSet_t *)ptr)->bw,
                         ((sRadioCtrlSet_t *)ptr)->codingRate,
                         ((sRadioCtrlSet_t *)ptr)->crc,
                         ((sRadioCtrlSet_t *)ptr)->preamble,
                         ((sRadioCtrlSet_t *)ptr)->invIQ, AT_TAIL);
      } else {
        len = 0;
      }
      break;
    default:
      len = AT_VPRINTF("%s%s%s", AT_HEADER, CmdTab[Cmd], AT_TAIL);
      DBG_PRINTF ("format not yet supported \n\r");
      break;
  } /* end switch(cmd)*/
  return len;
}


/******************************************************************************
  * @brief This function sends an AT cmd to the slave device
  * @param len: length of the AT cmd to be sent
  * @retval boolean
******************************************************************************/
static bool at_cmd_send(uint16_t len)
{
  bool RetCode;
  uint16_t size;

  DBG_PRINTF("cmd: %s\r\n", LoRa_AT_Cmd_Buff);

  /* Transmit the command from master to slave */
  size = HW_UART_Modem_Write((uint8_t*)LoRa_AT_Cmd_Buff, len);
  if(size == len ) {
    RetCode = true;
  } else {
    RetCode = false;
  }
  return RetCode;
}

/******************************************************************************
  * @brief This function receives response from the slave device
  * @param pdata: pointeur to the value returned by the slave
  * @retval return code coming from slave
******************************************************************************/
static ATEerror_t at_cmd_receive(void *pdata)
{
  uint8_t  ResponseComplete = 0;
  int8_t i = 0;
  int8_t charnumber = 0;
  char *ptrChr;
  ATEerror_t RetCode;
  /* Discriminate the Get return code from return value */
  uint8_t NoReturnCode = 1;
  uint32_t msStart;

  /* Cleanup the response buffer*/
  memset(response, 0x00, DATA_RX_MAX_BUFF_SIZE);

  while (!ResponseComplete) {
    msStart = millis();
    while (!HW_UART_Modem_IsNewCharReceived()) {
      if((millis() - msStart) > RESPONSE_TIMEOUT) {
        return AT_UART_LINK_ERROR;
      }
    }

    /* Process the response*/
    response[i] = HW_UART_Modem_GetNewChar();

    /*
	 * Delete this first three characters:
	 * "\r# "
	 * to remove response analysing issue
	 */
    if((response[0] == '\r') && (response[1] == '#') &&
       (response[2] == ' ')) {
      i = -1;
    }

    /* Wait up to carriage return OR the line feed marker*/
    if (/*(response[i] =='\r') || */(response[i] == '\n')) {
      DBG_PRINTF("at_cmd_receive: %s\r\n", response);
      if(pdata == NULL) {
        /* Returned code following a SET cmd or simple AT cmd*/
		i= 0;
        ResponseComplete = 1;
        RetCode = at_cmd_responseAnalysing(response);
        break;
      } else {
        /* Returned value following a GET cmd */
        if (i!= 0 && NoReturnCode) {
          /* First statement to get back the return value */
          response[i] = '\0';
          if (gFlagException != AT_FWVERSION) { /* See comment in lora_driver.c */
            ptrChr = strchr(&response[1],'=');  /* Skip the '\0''\r' */
            strcpy(pdata,ptrChr+1);
          } else {
            strcpy(pdata,&response[1]);
            gFlagException = AT_END_AT;
          }
          memset(response, 0x00, strlen(response));
          i= -1; /* Compensate the next index iteration and restart in [0] */
          NoReturnCode = 0; /*return code for the Get cmd*/
        } else {
          if (i>1) {
            /*second statement to get back the return code*/
            i= 0;
            ResponseComplete = 1;   /*when value '=' return code have been trapped*/
            RetCode = at_cmd_responseAnalysing(response);
            memset(response, 0x00, 16);
            break;
          }
        }
      }
    } else {
      if (i ==  (DATA_RX_MAX_BUFF_SIZE-1)) {
        /* Frame overflow */
        i = 0;
        return (AT_TEST_PARAM_OVERFLOW);
      }
    }
    i++;
    charnumber++;
  }
  return RetCode;
}

/******************************************************************************
  * @brief This function receives asynchronus event from the slave device
  * @param none
  * @retval return code coming from slave
******************************************************************************/
static ATEerror_t at_cmd_receive_async_event(void)
{
  uint8_t  ResponseComplete = 0;
  int8_t i = 0;
  int8_t charnumber = 0;
  char *ptrChr;
  ATEerror_t RetCode;
  /* Discriminate the Get return code from return value */
  uint8_t NoReturnCode =1;
  uint32_t msStart;

  /* Cleanup the response buffer */
  memset(response, 0x00, DATA_RX_MAX_BUFF_SIZE);

  while (!ResponseComplete) {
    msStart = millis();
    while (!HW_UART_Modem_IsNewCharReceived()) {
      if((millis() - msStart) > RESPONSE_TIMEOUT) {
        return AT_UART_LINK_ERROR;
      }
    }

    /* Process the response */
    response[i] = HW_UART_Modem_GetNewChar();

    /* Wait up to carriage return OR the line feed marker */
    if (/*(response[i] =='\r') || */(response[i] == '\n')) {
      DBG_PRINTF("at_cmd_receive_async_event: %s\r\n", response);

      if (i!= 0 && NoReturnCode) {  /* Trap the asynchronous event*/
        /* First statement to get back the return value */
        response[i] = '\0';
        ptrChr = strchr(&response[0], '+'); /* Skip the '\0''\r' */
        RetCode = at_cmd_AsyncEventAnalysing(ptrChr, NULL);
        memset(response, 0x00, 16);
        i= -1; /* Compensate the next index iteration and restart in [0] */
        NoReturnCode = 0;  /* Return code for the Get cmd*/
        break;
      } else {
        if (i>1) {
          /* Second statement to get back the return code */
          i= 0;
          ResponseComplete = 1; /* When value + return code have been trapped */
          RetCode = at_cmd_responseAnalysing(response);
          memset(response, 0x00, 16);
          break;
        }
      }
    } else {
      if (i ==  (DATA_RX_MAX_BUFF_SIZE-1)) {
        /* Frame overflow */
        i = 0;
        return (AT_TEST_PARAM_OVERFLOW);
      }
    }
    i++;
    charnumber++;
  }
  return RetCode;
}

/******************************************************************************
  * @brief This function receives asynchronus event from the slave device
  * @param none
  * @retval return code coming from slave
******************************************************************************/
static ATEerror_t at_cmd_receive_async_event_downlink_data(void *pdata)
{
  int8_t i = 0;
  int8_t charnumber = 0;
  char *ptrChr;
  ATEerror_t RetCode;
  /* Discriminate the Get return code from return value */
  uint8_t NoReturnCode =1;
  int8_t DlinkData_Complete = (0x1U);
  uint32_t msStart;

  /* Cleanup the response buffer */
  memset(response, 0x00, DATA_RX_MAX_BUFF_SIZE);

  while (!(DlinkData_Complete & (0x1u <<2))) {  /* Received sequence not complete */
    msStart = millis();
    while (!HW_UART_Modem_IsNewCharReceived()) {
      if((millis() - msStart) > ASYNC_EVENT_TIMEOUT) {
        return AT_UART_LINK_ERROR;
      }
    }

	/* Process the response */
    response[i] = HW_UART_Modem_GetNewChar();

    /* Wait up to carriage return OR the line feed marker */
    if ((response[i] =='\r') || (response[i] == '\n')) {
      DBG_PRINTF("at_cmd_receive_async_event_downlink_data: %s\r\n", response);

      /* Trap the asynchronous events associated to network downlink data */
      if (i!= 0 && NoReturnCode) {
        /* Sequence of events to be trapped: +RXPORT , +PAYLOADSIZE , +RCV */
        response[i] = '\0';
        /*
		 * Here when we go out from low power mode the prefix is '\r' only.
         * We do not skip the '\0''\r' - USI behavior ...
         */
        ptrChr = strchr(&response[0], '+');
        RetCode = at_cmd_AsyncEventAnalysing(ptrChr,&DlinkData_Complete);
        if(RetCode == AT_OK) {
          ptrChr = strchr(&response[1],'='); /* Skip the '\0''\r' */
          strcpy(pdata,ptrChr+1);
          pdata = (char*)pdata + strlen(ptrChr+1);
          if(!(DlinkData_Complete & (0x1u <<2))) {
            /* Introduce separator in order to discriminate port, size and data */
            *((char*)pdata) = ',';
            pdata = (char*)pdata + 1;
          }
        }

        memset(response, 0x00, 16);
        i= -1; /* Compensate the next index iteration and restart in [0 ] */
      }
    } else {
      if (i ==  (DATA_RX_MAX_BUFF_SIZE-1)) {
        /* Frame overflow */
        i = 0;
        return (AT_TEST_PARAM_OVERFLOW);
      }
    }
    i++;
    charnumber++;
  }
  return RetCode;
}

/******************************************************************************
  * @brief This function does analysis of the response received by the device
  * @param response: pointer to the received response
  * @retval ATEerror_t error type
******************************************************************************/
static ATEerror_t at_cmd_responseAnalysing(const char *ReturnResp)
{
  ATEerror_t status;
  int i;
  status = AT_END_ERROR;

  for (i = 0; i < AT_END_ERROR; i++) {
    if (strncmp(ReturnResp,
                ATE_RetCode[i].RetCodeStr,
               (ATE_RetCode[i].SizeRetCodeStr-1)) == 0) {
      /* Command has been found found */
      status = ATE_RetCode[i].RetCode;
      return status;
    }
  }
 return status;
}

/******************************************************************************
  * @brief This function does analysis of the asynchronous event received by the device
  * @param response: pointer to the received response
  * @retval ATEerror_t error type
******************************************************************************/
static ATEerror_t at_cmd_AsyncEventAnalysing(const char *ReturnResp, int8_t *Flag)
{
  ATEerror_t status;
  status = AT_END_ERROR;

  if (strncmp(ReturnResp, "+JoinAccepted\r", sizeof("+JoinAccepted\r")-1) == 0) {
   /* Event has been identified*/
   status = AT_OK;
  } else {
    /* Following statements for network downlink data analysis */
    if (strncmp(ReturnResp, "+RXPORT", sizeof("+RXPORT")-1) == 0) {
      /* Event has been identified */
      *Flag <<= (0x0U);
      status = AT_OK;
    } else {
      /* Following statement for network downlink data */
      if (strncmp(ReturnResp, "+PAYLOADSIZE", sizeof("+PAYLOADSIZE")-1) == 0) {
        /* event has been identified */
        *Flag <<= (0x1U);
        status = AT_OK;
      } else {
        /* Following statement for network downlink data */
        if (strncmp(ReturnResp, "+RCV", sizeof("+RCV")-1) == 0) {
          /* Event has been identified */
          if(*Flag == 0x1U) {
            *Flag = (0x1U << 2);
          } else {
            *Flag <<= (0x1U);
          }
          status = AT_OK;
        } else {
          /* Following statement for network downlink data */
          if (strncmp(ReturnResp, "+PS", sizeof("+PS")-1) == 0) {
           /* Event has been identified */
           status = AT_OK;
          } else {
            /* Following statement for network downlink data */
            if (strncmp(ReturnResp, "+TX: Done", sizeof("+TX: Done")-1) == 0) {
              /* Event has been identified */
              status = AT_OK;
            }
          }
        }
      }
    }
  }
  return status;
}

/******************************************************************************
  * @brief format the AT frame to be sent to the modem (slave)
  * @param pointer to the format string
  * @retval len of the string to be sent
******************************************************************************/
uint16_t at_cmd_vprintf(const char *format, ...)
{
  va_list args;
  uint16_t len;

  va_start(args, format);

  len = tiny_vsnprintf_like(LoRa_AT_Cmd_Buff+Offset,
                            sizeof(LoRa_AT_Cmd_Buff)-Offset, format, args);

  va_end(args);

  return len;
}

#ifdef __cplusplus
 }
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
