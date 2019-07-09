/**
  ******************************************************************************
  * @file    main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H


/* Includes ------------------------------------------------------------------*/
#include "stm32mp1xx_hal.h"
#include <string.h>
#include "openamp.h"
#include "log.h"
#include "rpmsg_hdr.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define Log log_info

#define DMA_SRAM 1

#define TIM2_DMA_ID        TIM_DMA_ID_UPDATE
#define TIM2_TRGO_OCREF    TIM_TRGO_UPDATE      /* OC2REF signal is used as trigger output (TRGO) */

#define LA_0_PIN                     GPIO_PIN_10
#define LA_0_GPIO_PORT               GPIOE
#define LA_1_PIN                     GPIO_PIN_11
#define LA_1_GPIO_PORT               GPIOE
#define LA_2_PIN                     GPIO_PIN_12
#define LA_2_GPIO_PORT               GPIOE
#define LA_3_PIN                     GPIO_PIN_13
#define LA_3_GPIO_PORT               GPIOE
#define LA_4_PIN                     GPIO_PIN_14
#define LA_4_GPIO_PORT               GPIOE
#define LA_5_PIN                     GPIO_PIN_9
#define LA_5_GPIO_PORT               GPIOE
#define LA_6_PIN                     GPIO_PIN_8
#define LA_6_GPIO_PORT               GPIOE
#define LA_MEAS_PIN                  GPIO_PIN_7
#define LA_MEAS_PORT                 GPIOE

typedef enum
{
  LA_STATE_OFF = 0,
  LA_STATE_READY,
  LA_STATE_WAITING_DECL,
  LA_STATE_SAMPLING,
  LA_STATE_SAMPLED,
}LA_State;


/* Exported functions ------------------------------------------------------- */
void Error_Handler(void);



#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
