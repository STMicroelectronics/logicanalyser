/**
  ******************************************************************************
  * @file    virt_uart.h
  * @author  MCD Application Team
  * @brief   Header file of UART VIRT module.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RPMSG_HDR_H
#define __RPMSG_HDR_H


#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "openamp.h"

/* Exported structures --------------------------------------------------------*/
 typedef struct __RPMSG_HDR_HandleTypeDef
 {
   struct rpmsg_endpoint ept;          /*!< rpmsg endpoint                             */
   struct rpmsg_virtio_device *rvdev;  /*< pointer to the rpmsg virtio device          */
   uint8_t              *pRxBuffPtr;   /*!< Pointer to VIRTUAL UART Rx transfer Buffer */
   uint16_t              RxXferSize;   /*!< VIRTUAL UART Rx Transfer size              */
   void    (* RxCpltCallback)( struct __RPMSG_HDR_HandleTypeDef * hppp);    /*!< RX CPLT callback    */
 }RPMSG_HDR_HandleTypeDef;

 typedef struct __RPMSG_HDR_DdrBuffTypeDef
 {
   uint32_t physAddr;
   uint32_t physSize;
 }RPMSG_HDR_DdrBuffTypeDef;

typedef enum
{
    RPMSG_HDR_OK       = 0x00U,
    RPMSG_HDR_ERROR    = 0x01U,
    RPMSG_HDR_BUSY     = 0x02U,
    RPMSG_HDR_TIMEOUT  = 0x03U
} RPMSG_HDR_StatusTypeDef;


typedef enum
{
    RPMSG_HDR_RXCPLT_CB_ID          = 0x00U,    /*!< PPP event 1 callback ID     */
}RPMSG_HDR_CallbackIDTypeDef;


/* Exported functions --------------------------------------------------------*/
/* Initialization and de-initialization functions  ****************************/
RPMSG_HDR_StatusTypeDef RPMSG_HDR_Init(RPMSG_HDR_HandleTypeDef *hdr);
RPMSG_HDR_StatusTypeDef RPMSG_HDR_DeInit (RPMSG_HDR_HandleTypeDef *hdr);
RPMSG_HDR_StatusTypeDef RPMSG_HDR_RegisterCallback(RPMSG_HDR_HandleTypeDef *hdr,
        RPMSG_HDR_CallbackIDTypeDef CallbackID,
        void (* pCallback)(RPMSG_HDR_HandleTypeDef *_hdr));

/* IO operation functions *****************************************************/
RPMSG_HDR_StatusTypeDef RPMSG_HDR_Transmit(RPMSG_HDR_HandleTypeDef *hdr, uint8_t *pData, uint16_t Size);


#ifdef __cplusplus
}
#endif

#endif /* __RPMSG_HDR_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
