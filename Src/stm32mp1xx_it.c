/** 
  ******************************************************************************
  * @file    QSPI/QSPI_ReadWrite_IT/Src/stm32mp1xx_it.c
  * @author  MCD Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
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
#include "stm32mp1xx_it.h"
#include "stm32mp1xx_hal.h"
#include "log.h"
#include "main.h"

/** @addtogroup STM32MP1xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern IPCC_HandleTypeDef hipcc;
extern TIM_HandleTypeDef htim2;
extern VIRT_UART_HandleTypeDef huart0;
extern char* mUartBuffTx;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
    //log_info("NMI_Handler !!!\n\r");
    sprintf(mUartBuffTx, "NMI_Handler !!!\n\r");
    VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  //log_info("HardFault_Handler !!!\n\r");
  sprintf(mUartBuffTx, "HardFault_Handler !!!\n\r");
  VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  //log_info("MemManage_Handler !!!\n\r");
  sprintf(mUartBuffTx, "MemManage_Handler !!!\n\r");
  VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  //log_info("BusFault_Handler !!!\n\r");
  sprintf(mUartBuffTx, "BusFault_Handler !!!\n\r");
  VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  //log_info("UsageFault_Handler !!!\n\r");
  sprintf(mUartBuffTx, "UsageFault_Handler !!!\n\r");
  VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

void RCC_WAKEUP_IRQHandler(void)
{
  HAL_RCC_WAKEUP_IRQHandler();
}

/******************************************************************************/
/*                 STM32MP1xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (QSPI), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32mp1xx.s).                                               */
/******************************************************************************/

void IPCC_RX0_IRQHandler(void) {

	log_warn("%s: IT spurious\n", __func__);
}

void IPCC_TX0_IRQHandler(void) {

	log_warn("%s: IT spurious\n", __func__);
}

void IPCC_RX1_IRQHandler(void) {
    //log_dbg("%s: IT RX1\r\n", __func__);
    //log_info("%s: IT RX1\r\n", __func__);
	HAL_IPCC_RX_IRQHandler(&hipcc);
}

void IPCC_TX1_IRQHandler(void) {

    //log_dbg("%s: IT_TX1\r\n", __func__);
    //log_info("%s: IT_TX1\r\n", __func__);
	HAL_IPCC_TX_IRQHandler(&hipcc);
}

/**
 * @brief  This function handles External line
 *         interrupt request for BlueNRG.
 * @param  None
 * @retval None
 */
void EXTI10_IRQHandler( void )
{
    log_info("EXTI10_IRQHandler\n");
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_10 );    // PE10 => DIO2
}

void EXTI8_IRQHandler( void )
{
    log_info("EXTI8_IRQHandler\n");
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_8 );    // PE8
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
