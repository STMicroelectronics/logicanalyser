/** 
  ******************************************************************************
  * @file    stm32mp15xx_eval_config.h
  * @author  MCD Application Team
  * @brief   configuration file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) $COPYRIGHT_YEAR$ STMicroelectronics</center></h2>
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
#ifndef __STM32MP15XX_EVAL_CONFIG_H
#define __STM32MP15XX_EVAL_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32mp1xx_hal.h"

/* Activation of PMIC */

#if defined (CONFIG_MASTER_MODE)
#define USE_PMU1                            1U
#else
#define USE_PMU1                            0U
#endif

/* Activation of COM log */
#define USE_COM_LOG                         0U

/* IRQ priorities */
#define BSP_PMU1_IT_PRIORITY                   0x03UL
#define BSP_BUTTON_WAKEUP_IT_PRIORITY          0x0FUL
#define BSP_BUTTON_USER_IT_PRIORITY            0x0FUL
#define BSP_BUTTON_USER2_IT_PRIORITY            0x0FUL
   
#endif /* __STM32MP15XX_EVAL_CONFIG_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
