/**
  ******************************************************************************
  * File Name          : stm32mp1xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization 
  *                      and de-Initialization codes.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32mp1xx_hal.h"
#include "lock_resource.h"
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

//extern void _Error_Handler(char *, int);
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_3);

  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 4, 0);

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
* @brief IPCC MSP Initialization
* This function configures the hardware resources used in this example
* @param hipcc: IPCC handle pointer
* @retval None
*/
void HAL_IPCC_MspInit(IPCC_HandleTypeDef* hipcc)
{
  if(hipcc->Instance==IPCC)
  {

  /* USER CODE BEGIN IPCC_MspInit 0 */

  /* USER CODE END IPCC_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_IPCC_CLK_ENABLE();

  /* IPCC interrupt Init */
    HAL_NVIC_SetPriority(IPCC_RX1_IRQn, 4, 1);
    //HAL_NVIC_SetPriority(IPCC_RX1_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(IPCC_RX1_IRQn);

  /* USER CODE BEGIN IPCC_MspInit 1 */

  /* USER CODE END IPCC_MspInit 1 */
  }

}

/**
* @brief IPCC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hipcc: IPCC handle pointer
* @retval None
*/
void HAL_IPCC_MspDeInit(IPCC_HandleTypeDef* hipcc)
{

  if(hipcc->Instance==IPCC)
  {
  /* USER CODE BEGIN IPCC_MspDeInit 0 */

  /* USER CODE END IPCC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_IPCC_CLK_DISABLE();

    /* IPCC interrupt DeInit */
    HAL_NVIC_DisableIRQ(IPCC_RX1_IRQn);
  /* USER CODE BEGIN IPCC_MspDeInit 1 */

  /* USER CODE END IPCC_MspDeInit 1 */
  }

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

      if(htim_base->Instance==TIM2)
      {
      /* USER CODE BEGIN TIM2_MspInit 0 */

      /* USER CODE END TIM2_MspInit 0 */
        /* Peripheral clock enable */
          __HAL_RCC_TIM2_CLK_ENABLE();

#ifdef DMA_SRAM
          /* Enable DMA clock */
          __HAL_RCC_DMA2_CLK_ENABLE();
          __HAL_RCC_DMAMUX_CLK_ENABLE();

          /* Configure the DMA handler for Transmission process */
          static DMA_HandleTypeDef hdma_tim;
          hdma_tim.Instance                 = DMA2_Stream0;
          hdma_tim.Init.Request             = DMA_REQUEST_TIM2_UP;
          hdma_tim.Init.Direction           = DMA_PERIPH_TO_MEMORY;
          hdma_tim.Init.PeriphInc           = DMA_PINC_DISABLE;
          hdma_tim.Init.MemInc              = DMA_MINC_ENABLE;
          hdma_tim.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
          hdma_tim.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
          hdma_tim.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
          hdma_tim.Init.Mode                = DMA_CIRCULAR;
          hdma_tim.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;   // using FIFO mode since memory datasize=32bit and GPIO=8bits in our case
          hdma_tim.Init.MemBurst            = DMA_MBURST_SINGLE;
          hdma_tim.Init.PeriphBurst         = DMA_PBURST_SINGLE;

          if (HAL_DMA_Init(&hdma_tim) != HAL_OK)
          {
            Error_Handler();
          }
          /* Associate the initialized DMA handle to the UART handle */
          __HAL_LINKDMA(htim_base, hdma[TIM_DMA_ID_UPDATE], hdma_tim);

          /* NVIC configuration for DMA transfer complete interrupt  */
          HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
          HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
#else
          /* Peripheral interrupt init*/
          HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
          HAL_NVIC_EnableIRQ(TIM2_IRQn);
#endif

      /* USER CODE BEGIN TIM2_MspInit 1 */

      /* USER CODE END TIM2_MspInit 1 */
      }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
