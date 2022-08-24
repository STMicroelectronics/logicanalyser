/**
  ******************************************************************************
  * @file    virt_uart.c
  * @author  MCD Application Team
  * @brief   UART HAL module driver.
  *          This file provides firmware functions to manage an rpmsg endpoint
  *          from user application
  *
  *
  @verbatim
 ===============================================================================
                        ##### How to use this driver #####
 ===============================================================================
  [..]
    The VIRTUAL UART driver can be used as follows:
    (#) Initialize the Virtual UART by calling the VIRT_UART_Init() API.
        (++) create an endpoint. listener on the OpenAMP-rpmsg channel is now enabled.
        Receive data  is now possible if user registers a callback to this VIRTUAL UART instance
        by calling in providing a callback function when a message is received from
        remote processor (VIRT_UART_read_cb)
        OpenAMP MW deals with memory allocation/free and signal events
    (#) Transmit data on the created rpmsg channel by calling the VIRT_UART_Transmit()
    (#) Receive data in calling VIRT_UART_RegisterCallback to register user callback


  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
//#include "virt_uart.h"
#include "rpmsg_hdr.h"
#include "metal/utilities.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* this string will be sent to remote processor */
#define RPMSG_SERVICE_NAME              "rpmsg-sdb-channel"
#define MAX_DDR_BUFF 16

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static RPMSG_HDR_DdrBuffTypeDef mArrayDdrBuff[MAX_DDR_BUFF];  // static array to save DdrBuff @ and size
static uint8_t mArrayDdrBuffCount;

static int RPMSG_HDR_read_cb(struct rpmsg_endpoint *ept, void *data,
			    size_t len, uint32_t src, void *priv)
{
  RPMSG_HDR_HandleTypeDef *hdr = metal_container_of(ept, RPMSG_HDR_HandleTypeDef, ept);
  (void)src;

  hdr->pRxBuffPtr = data;
  hdr->RxXferSize = len;
  if (hdr->RxCpltCallback != NULL) {
      hdr->RxCpltCallback(hdr);
  }

  return 0;
}

RPMSG_HDR_StatusTypeDef RPMSG_HDR_Init(RPMSG_HDR_HandleTypeDef *hdr)
{

  int status;

  /* Create a endpoint for rmpsg communication */

  status = OPENAMP_create_endpoint(&hdr->ept, RPMSG_SERVICE_NAME, RPMSG_ADDR_ANY,
          RPMSG_HDR_read_cb, NULL);

  if(status < 0) {
    return RPMSG_HDR_ERROR;
  }
  mArrayDdrBuffCount  = 0;
  memset(mArrayDdrBuff, 0, sizeof(RPMSG_HDR_DdrBuffTypeDef) * MAX_DDR_BUFF);
  return RPMSG_HDR_OK;
}

RPMSG_HDR_StatusTypeDef RPMSG_HDR_DeInit (RPMSG_HDR_HandleTypeDef *hdr)
{
  OPENAMP_destroy_ept(&hdr->ept);

  return RPMSG_HDR_OK;
}

RPMSG_HDR_StatusTypeDef RPMSG_HDR_RegisterCallback(RPMSG_HDR_HandleTypeDef *hdr,
        RPMSG_HDR_CallbackIDTypeDef CallbackID,
        void (* pCallback)(RPMSG_HDR_HandleTypeDef *_hdr))
{

  switch (CallbackID)
  {
  case RPMSG_HDR_RXCPLT_CB_ID :
    hdr->RxCpltCallback = pCallback;
    return RPMSG_HDR_OK;
    break;

  default :
   /* Return error status */
	return  RPMSG_HDR_ERROR;
    break;
  }

  return RPMSG_HDR_OK;
}

RPMSG_HDR_StatusTypeDef RPMSG_HDR_Transmit(RPMSG_HDR_HandleTypeDef *hdr, uint8_t *pData, uint16_t Size)
{
	int res;

	if (Size > (RPMSG_BUFFER_SIZE-16))
	  return RPMSG_HDR_ERROR;

	res = OPENAMP_send(&hdr->ept, pData, Size);
	if (res <0) {
		return RPMSG_HDR_ERROR;
	}

	return RPMSG_HDR_OK;
}
