/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @brief   This example describes how to configure and use OpenAMP MW and
  *          the STM32MP1xx VIRTUAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
#include "lock_resource.h"
#include "main.h"

/** @addtogroup STM32MP1xx_HAL_Examples
  * @{
  */

/** @addtogroup OpenAMP_TTY_echo_wakeup
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
typedef enum {
    RISING,                 /*  0 */
    FALLING,                /*  1 */
    BOTH,                /*  1 */
    NONE,                /*  1 */
} sampling_edge_t;

typedef struct __HDR_DdrBuffTypeDef
{
  uint32_t physAddr;
  uint32_t physSize;
}HDR_DdrBuffTypeDef;

#define SAMP_SRAM_PACKET_SIZE 1024
#define SAMP_DDR_BUFFER_SIZE (1024*1024)

#define DMA_MEM_BUFF_SIZE   ((uint32_t) 512)      /* size in byte.. must be aligned with SDRAM word access..so must be a multiple of 4 ... must be less than DMA counter 65535  */

#define MAX_DDR_BUFF 16

//static char machine_state_str[5][13] = {"OFF", "READY", "WAITING_DECL", "SAMPLING", "SAMPLED"};
//static char CMD_FINISHED[3] = "OK";

#define GPIOx_IDR               (GPIOE_BASE + 0x10 + 1)     // portE bit 8..12
#define PRINTF_TIME 5
#define SAMPLE_BUFF_LEN 992


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
IPCC_HandleTypeDef hipcc;
DMA_HandleTypeDef hdma_memtomem_dma2_stream1;

static struct rpmsg_virtio_device rvdev;
RPMSG_HDR_HandleTypeDef hsdb0;
VIRT_UART_HandleTypeDef huart0;

__IO FlagStatus VirtUart0RxMsg = RESET;
__IO FlagStatus SDB0RxMsg = RESET;
__IO FlagStatus VirtUart0OK = RESET;
__IO FlagStatus fDmaBuffAvailable = RESET;
__IO FlagStatus mDMAerror = RESET;
char mUartBuffTx[512];
__IO FlagStatus fDdrDma2Send0 = RESET;
__IO FlagStatus fDdrDma2Send1 = RESET;
__IO FlagStatus fStopRequested = RESET;

uint8_t VirtUart0ChannelBuffRx[100];
uint16_t VirtUart0ChannelRxSize = 0;
uint8_t SDB0ChannelBuffRx[100];
uint16_t SDB0ChannelRxSize = 0;
char mSdbBuffTx[512];

volatile uint32_t mMCUfreq, mTIM2freq;
uint32_t m_samp_freq;

uint8_t mSampBuff[SAMP_SRAM_PACKET_SIZE * 2];   // use a circular buffer in SRAM
uint8_t mSampBuffOut[SAMP_SRAM_PACKET_SIZE * 2];   // use a circular buffer in SRAM
static HDR_DdrBuffTypeDef mArrayDdrBuff[MAX_DDR_BUFF];    // used to store DDR buff allocated by Linux driver
static uint8_t mArrayDdrBuffCount = 0;
static uint8_t mArrayDdrBuffIndex = 0;  // will vary from 0 to mArrayDdrBuffCount-1
uint64_t mTotalCompSampCount=0;
uint32_t mRollingCompSampCount=0;
uint32_t mRawSampCount, mSampCount, mPrevSampCount;    // nb of compressed sample
uint8_t mLastSamp;
int8_t mSampRepet;

char testStr[21] = {"B0Ad4200000L00100000"};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
    /* User may add here some code to deal with this error */
    //log_err("OOOps: file %s, line %d\n", __FILE__, __LINE__);
    sprintf(mUartBuffTx, "OOOps: Error_Handler\n");
    VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));

    while(1);

}

static void MX_IPCC_Init(void)
{

  hipcc.Instance = IPCC;
  if (HAL_IPCC_Init(&hipcc) != HAL_OK)
  {
     Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;
    /*Configure GPIO pins as input */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = 0;
    PERIPH_LOCK(GPIOE);
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    PERIPH_UNLOCK(GPIOE);

    /*Configure GPIO pin as ouput => EXTI timing measurement*/
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = 0;
    PERIPH_LOCK(GPIOE);
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    PERIPH_UNLOCK(GPIOE);
    Log("MX_GPIO_Init GPIOE MODER=%lu\n", GPIOE->MODER);
}

/* TIM6 init function */
static void MX_TIM2_Init(void)
{

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 16;   // 80ns => 12MHz
  HAL_TIM_Base_Init(&htim2);

#ifdef DMA_SRAM
  TIM_MasterConfigTypeDef   sMasterConfig;
  /* Configure TIMx as master & use the UPDATE event as Trigger Output (TRGO) */
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_ENABLE;
  if ( HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
#endif
}

static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMAMUX_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_stream1 on DMA2_Stream0 */
  hdma_memtomem_dma2_stream1.Instance = DMA2_Stream1;
  hdma_memtomem_dma2_stream1.Init.Request = DMA_REQUEST_MEM2MEM;
  hdma_memtomem_dma2_stream1.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream1.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_memtomem_dma2_stream1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_memtomem_dma2_stream1.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream1.Init.Priority = DMA_PRIORITY_LOW;
  hdma_memtomem_dma2_stream1.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream1.Init.MemBurst = DMA_MBURST_INC4;
  hdma_memtomem_dma2_stream1.Init.PeriphBurst = DMA_PBURST_INC4;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream1) != HAL_OK)
  {
    Error_Handler();
  }

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

void VIRT_UART0_RxCpltCallback(VIRT_UART_HandleTypeDef *huart)
{
    // copy received msg in a buffer
    VirtUart0ChannelRxSize = huart->RxXferSize < 100? huart->RxXferSize : 99;
    memcpy(VirtUart0ChannelBuffRx, huart->pRxBuffPtr, VirtUart0ChannelRxSize);
    VirtUart0ChannelBuffRx[VirtUart0ChannelRxSize] = 0;   // insure end of String
    //Log("VIRT_UART0_RxCpltCallback: %s\n", VirtUart0ChannelBuffRx);
    sprintf(mUartBuffTx, "VIRT_UART0_RxCpltCallback: %s\n", VirtUart0ChannelBuffRx);
    VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
    VirtUart0RxMsg = SET;
}

void SDB0_RxCpltCallback(RPMSG_HDR_HandleTypeDef *huart)
{
    // copy received msg in a buffer
    SDB0ChannelRxSize = huart->RxXferSize < 100? huart->RxXferSize : 99;
    memcpy(SDB0ChannelBuffRx, huart->pRxBuffPtr, SDB0ChannelRxSize);
    SDB0ChannelBuffRx[SDB0ChannelRxSize] = 0;   // insure end of String
    sprintf(mUartBuffTx, "SDB0_RxCpltCallback: %s\n", SDB0ChannelBuffRx);
    VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
    //Log("SDB0_RxCpltCallback: %s\n", SDB0ChannelBuffRx);
    SDB0RxMsg = SET;
}

static void TransferCompleteDDR(DMA_HandleTypeDef *DmaHandle)
{
    mTotalCompSampCount += SAMP_SRAM_PACKET_SIZE;
    mRollingCompSampCount += SAMP_SRAM_PACKET_SIZE;
    if ((mTotalCompSampCount % SAMP_DDR_BUFFER_SIZE) == 0) {
        // time to change DDR buffer and send MSG to Linux
        //sprintf(mSdbBuffTx, "Buffer@%dL%08x", mArrayDdrBuffIndex, SAMP_DDR_BUFFER_SIZE);
        //sprintf(mSdbBuffTx, "B%dL%08x", mArrayDdrBuffIndex, SAMP_DDR_BUFFER_SIZE);
        //RPMSG_HDR_Transmit(&hsdb0, (uint8_t*)mSdbBuffTx, strlen(mSdbBuffTx));
        sprintf(mUartBuffTx, "DMA2DDR-B%dL%08x", mArrayDdrBuffIndex, SAMP_DDR_BUFFER_SIZE);
        VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
        mRollingCompSampCount = 0;
        mArrayDdrBuffIndex++;
        if (mArrayDdrBuffIndex == mArrayDdrBuffCount) {
            mArrayDdrBuffIndex = 0;
        }

    //} else {
        //sprintf(mUartBuffTx, "TransferCompleteDDR packet of 1024 bytes sent, mTotalCompSampCount=%d\n", mTotalCompSampCount);
        //VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
    }
}

static void TransferCompleteSRAM(DMA_HandleTypeDef *DmaHandle)
{
    mRawSampCount += SAMP_SRAM_PACKET_SIZE;
    //HAL_DMA_Abort_IT(htim2.hdma[TIM_DMA_ID_UPDATE]);
    //HAL_TIM_Base_Stop(&htim2);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);   // IRQ measurement thanks to GPIO management
    for (int i=0; i<SAMP_SRAM_PACKET_SIZE; i++) {
        if (mLastSamp != 0xFF) {  // needed to start the algo correctly
            if (mLastSamp == mSampBuff[i]) {
                mSampRepet++;
                if (mSampRepet >= 7) {
                    // max number of occurence hit => time to save value
                    mSampBuffOut[mSampCount++] = (mLastSamp | 0xE0);    // save data including repetition
                    mLastSamp = 0xFF;
                    mSampRepet = -1;   // don't know yet, what will be next one
                }
            } else {
                // new value different of previous one => save previous
                mSampBuffOut[mSampCount++] = (mLastSamp | (mSampRepet << 5));    // save data including repetition
                mLastSamp = mSampBuff[i] & 0x1F;
                mSampRepet = 0;   // 1 occurence seen
            }
        } else {
            mLastSamp = mSampBuff[i] & 0x1F;
            mSampRepet = 0;   // 0 means 1 occurence, 7 means 8 occurences
        }
        if (mSampCount == SAMP_SRAM_PACKET_SIZE) {
            // time to transfer compressed buffer0 in DDR
            fDdrDma2Send0 = SET;
        } else if (mSampCount == (SAMP_SRAM_PACKET_SIZE*2)) {
            // time to transfer compressed buffer1 in DDR
            fDdrDma2Send1 = SET;
            mSampCount = 0;
        }
    }
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);

    //sprintf(mUartBuffTx, "DMA TransferCompleteSRAM mRawSampCount=%ld mSampCount=%ld\n", mRawSampCount, mSampCount);
    //VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
}

static void TransferErrorDDR(DMA_HandleTypeDef *DmaHandle)
{
    mDMAerror = SET;
    //Log("DMA DDR TransferError CB !!!\n");
    sprintf(mUartBuffTx, "DMA DDR TransferError CB !!!\n");
    VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
}

static void TransferErrorSRAM(DMA_HandleTypeDef *DmaHandle)
{
    mDMAerror = SET;
    //Log("DMA SRAM TransferError CB !!!\n");
    sprintf(mUartBuffTx, "DMA SRAM TransferError CB !!!\n");
    VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
}

/* @brief  This function handles DMA interrupt request.
* @param  None
* @retval None
*/
void DMA2_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(htim2.hdma[TIM_DMA_ID_UPDATE]);
}

void DMA2_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_memtomem_dma2_stream1);
}

void TIM2_IRQHandler(void)
{
  __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);   // IRQ measurement thanks to GPIO management
  volatile uint32_t portE = (LA_0_GPIO_PORT->IDR >> 8) & 0x0000001F;  // get portE and shift right by 8 bits, then keep last 5 bits

  if (mLastSamp != 0xFF) {  // needed to start the algo correctly
      if (mLastSamp == (uint8_t)portE) {
          mSampRepet++;
          if (mSampRepet >= 7) {
              // max number of occurence hit => time to save value
              mSampBuff[mSampCount++] = (mLastSamp | 0xE0);    // save data including repetition
              mLastSamp = (uint8_t)portE;
              mSampRepet = -1;   // don't know yet, what will be next one
          }
      } else {
          // new value different of previous one => save previous
          mSampBuff[mSampCount++] = (mLastSamp | ((mSampRepet-1) << 5));    // save data including repetition
          mLastSamp = (uint8_t)portE;
          mSampRepet = 0;   // 1 occurence seen
      }
  } else {
      mLastSamp = (uint8_t)portE;
      mSampRepet = 0;   // 0 means 1 occurence, 7 means 8 occurences
  }
  if (mSampCount >= (SAMP_SRAM_PACKET_SIZE*2)) {
      mSampCount = 0;
  }
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
}

uint8_t treatRxCommand2() {
    // example command: S002M => Sample, rate 002MHz
    if (VirtUart0ChannelBuffRx[0] == 'S') {
        for (int i=0; i<3; i++) {
            if (VirtUart0ChannelBuffRx[1+i] < '0' || VirtUart0ChannelBuffRx[1+i] > '9') {
                //Log("treatRxCommand2 ERROR wrong frequency digit:%c at offset:%d\n",
                //        VirtUart0ChannelBuffRx[1+i], 1+i);
                sprintf(mUartBuffTx, "treatRxCommand2 ERROR wrong frequency digit:%c at offset:%d\n",
                        VirtUart0ChannelBuffRx[1+i], 1+i);
                VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));

                return false;
            }
        }
        if (VirtUart0ChannelBuffRx[4] != 'M' && VirtUart0ChannelBuffRx[4] != 'k' && VirtUart0ChannelBuffRx[4] != 'H') {
            //Log("treatRxCommand2 ERROR wrong frequency unit:%c\n", VirtUart0ChannelBuffRx[4]);
            sprintf(mUartBuffTx, "treatRxCommand2 ERROR wrong frequency unit:%c\n", VirtUart0ChannelBuffRx[4]);
            VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
            return false;
        }
        m_samp_freq = (int)(VirtUart0ChannelBuffRx[1] - '0') * 100
                 + (int)(VirtUart0ChannelBuffRx[2] - '0') * 10
                 + (int)(VirtUart0ChannelBuffRx[3] - '0') * 1;
        if (VirtUart0ChannelBuffRx[4] == 'k') {
            m_samp_freq *= 1000;
        } else if (VirtUart0ChannelBuffRx[4] == 'M') {
            m_samp_freq *= 1000000;
        }
        //Log("treatRxCommand2 OK frequency=%ld\n", m_samp_freq);
        sprintf(mUartBuffTx, "treatRxCommand2 OK frequency=%ld\n", m_samp_freq);
        VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
        return true;
    } else if (VirtUart0ChannelBuffRx[0] == 'E') {
        // exit requested
        fStopRequested = SET;
    } else {
        sprintf(mUartBuffTx, "treatRxCommand2 ERROR wrong frequency command:%c instead of: S\n",
                VirtUart0ChannelBuffRx[0]);
        VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
    }
    return false;
}

/*
void treatSDBEvent() {
    uint32_t addr=0, size=0;
    // example command: B0AxxxxxxxxLyyyyyyyy => Buff0 @:xx..x Length:yy..y
    if (SDB0ChannelBuffRx[0] != 'B') {
        Log("treatSDBEvent ERROR wrong buffer command:%c\n", SDB0ChannelBuffRx[0]);
    }
    if (!((SDB0ChannelBuffRx[1] >= '0' && SDB0ChannelBuffRx[1] <= '3'))) {
        Log("treatSDBEvent ERROR wrong buffer index:%c\n", SDB0ChannelBuffRx[1]);
        return;
    }
    if (mArrayDdrBuffCount != (SDB0ChannelBuffRx[1] - '0')) {
        Log("treatSDBEvent ERROR wrong buffer received index:%c awaited index:%d\n",
                SDB0ChannelBuffRx[1], mArrayDdrBuffCount);
        return;
    }
    if (SDB0ChannelBuffRx[2] != 'A') {
        Log("treatSDBEvent ERROR wrong buffer address tag:%c instead of:A\n", SDB0ChannelBuffRx[1]);
        return ;
    }
    if (SDB0ChannelBuffRx[11] != 'L') {
        Log("treatSDBEvent ERROR wrong buffer length tag:%c instead of:L\n", SDB0ChannelBuffRx[11]);
        return ;
    }
    for (int i=0; i<8; i++) {
        if (!((SDB0ChannelBuffRx[3+i] >= '0' && SDB0ChannelBuffRx[3+i] <= '9') || (SDB0ChannelBuffRx[3+i] >= 'a' && SDB0ChannelBuffRx[3+i] <= 'f'))) {
            Log("treatSDBEvent ERROR wrong address digit:%c at offset:%d\n", SDB0ChannelBuffRx[3+i], 3+i);
            return ;
        } else {
            addr <<=4;
            if ((SDB0ChannelBuffRx[3+i] >= '0' && SDB0ChannelBuffRx[3+i] <= '9')) {
                addr |= (SDB0ChannelBuffRx[3+i] - '0');
            } else {
                addr |= (SDB0ChannelBuffRx[3+i] - 'a' + 10);
            }
        }
        if (!((SDB0ChannelBuffRx[12+i] >= '0' && SDB0ChannelBuffRx[12+i] <= '9') || (SDB0ChannelBuffRx[12+i] >= 'a' && SDB0ChannelBuffRx[12+i] <= 'f'))) {
            Log("treatSDBEvent ERROR wrong length digit:%c at offset:%d\n", SDB0ChannelBuffRx[12+i], 12+i);
            return ;
        } else {
            size <<=4;
            if ((SDB0ChannelBuffRx[12+i] >= '0' && SDB0ChannelBuffRx[12+i] <= '9')) {
                size |= (SDB0ChannelBuffRx[12+i] - '0');
            } else {
                size |= (SDB0ChannelBuffRx[12+i] - 'a' + 10);
            }
        }
    }
    // save DDR buff @ and size
    mArrayDdrBuff[mArrayDdrBuffCount].physAddr = addr;
    mArrayDdrBuff[mArrayDdrBuffCount].physSize = size;
    Log("treatSDBEvent OK physAddr=%x physSize=%x mArrayDdrBuffCount=%d\n",
            mArrayDdrBuff[mArrayDdrBuffCount].physAddr,
            mArrayDdrBuff[mArrayDdrBuffCount].physSize,
            mArrayDdrBuffCount);
    mArrayDdrBuffCount++;
}
*/
void treatSDBEvent() {
    // example command: B0AxxxxxxxxLyyyyyyyy => Buff0 @:xx..x Length:yy..y
    if (SDB0ChannelBuffRx[0] != 'B') {
        //Log("treatSDBEvent ERROR wrong buffer command:%c\n", SDB0ChannelBuffRx[0]);
        sprintf(mUartBuffTx, "treatSDBEvent ERROR wrong buffer command:%c\n", SDB0ChannelBuffRx[0]);
        VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
    }
    if (!((SDB0ChannelBuffRx[1] >= '0' && SDB0ChannelBuffRx[1] <= '3'))) {
        //Log("treatSDBEvent ERROR wrong buffer index:%c\n", SDB0ChannelBuffRx[1]);
        sprintf(mUartBuffTx, "treatSDBEvent ERROR wrong buffer index:%c\n", SDB0ChannelBuffRx[1]);
        VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
        return;
    }
    if (mArrayDdrBuffCount != (SDB0ChannelBuffRx[1] - '0')) {
        //Log("treatSDBEvent ERROR wrong buffer received index:%c awaited index:%d\n",
        //        SDB0ChannelBuffRx[1], mArrayDdrBuffCount);
        sprintf(mUartBuffTx, "treatSDBEvent ERROR wrong buffer received index:%c awaited index:%d\n",
                SDB0ChannelBuffRx[1], mArrayDdrBuffCount);
        VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
        return;
    }
    if (SDB0ChannelBuffRx[2] != 'A') {
        //Log("treatSDBEvent ERROR wrong buffer address tag:%c instead of:A\n", SDB0ChannelBuffRx[1]);
        sprintf(mUartBuffTx, "treatSDBEvent ERROR wrong buffer address tag:%c instead of:A\n", SDB0ChannelBuffRx[1]);
        VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
        return ;
    }
    if (SDB0ChannelBuffRx[11] != 'L') {
        //Log("treatSDBEvent ERROR wrong buffer length tag:%c instead of:L\n", SDB0ChannelBuffRx[11]);
        sprintf(mUartBuffTx, "treatSDBEvent ERROR wrong buffer length tag:%c instead of:L\n", SDB0ChannelBuffRx[11]);
        VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
        return ;
    }
    for (int i=0; i<8; i++) {
        if (!((SDB0ChannelBuffRx[3+i] >= '0' && SDB0ChannelBuffRx[3+i] <= '9') || (SDB0ChannelBuffRx[3+i] >= 'a' && SDB0ChannelBuffRx[3+i] <= 'f'))) {
            //Log("treatSDBEvent ERROR wrong address digit:%c at offset:%d\n", SDB0ChannelBuffRx[3+i], 3+i);
            sprintf(mUartBuffTx, "treatSDBEvent ERROR wrong address digit:%c at offset:%d\n", SDB0ChannelBuffRx[3+i], 3+i);
            VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
            return ;
        }
        if (!((SDB0ChannelBuffRx[12+i] >= '0' && SDB0ChannelBuffRx[12+i] <= '9') || (SDB0ChannelBuffRx[12+i] >= 'a' && SDB0ChannelBuffRx[12+i] <= 'f'))) {
            //Log("treatSDBEvent ERROR wrong length digit:%c at offset:%d\n", SDB0ChannelBuffRx[12+i], 12+i);
            sprintf(mUartBuffTx, "treatSDBEvent ERROR wrong length digit:%c at offset:%d\n", SDB0ChannelBuffRx[12+i], 12+i);
            VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
            return ;
        }
    }
    // save DDR buff @ and size
    mArrayDdrBuff[mArrayDdrBuffCount].physAddr = (uint32_t)strtoll((char*)&SDB0ChannelBuffRx[3], NULL, 16);
    mArrayDdrBuff[mArrayDdrBuffCount].physSize = (uint32_t)strtoll((char*)&SDB0ChannelBuffRx[12], NULL, 16);
    /*
    Log("treatSDBEvent OK physAddr=0x%lx physSize=%ld mArrayDdrBuffCount=%d\n",
            mArrayDdrBuff[mArrayDdrBuffCount].physAddr,
            mArrayDdrBuff[mArrayDdrBuffCount].physSize,
            mArrayDdrBuffCount);*/
    sprintf(mUartBuffTx, "treatSDBEvent OK physAddr=0x%lx physSize=%ld mArrayDdrBuffCount=%d\n",
            mArrayDdrBuff[mArrayDdrBuffCount].physAddr,
            mArrayDdrBuff[mArrayDdrBuffCount].physSize,
            mArrayDdrBuffCount);
    VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));

    mArrayDdrBuffCount++;
}


void configureTimer(uint32_t period) {
    htim2.Init.Period = period;    // ex: m_samp_freq=12MHz => period = 16
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    HAL_TIM_Base_Init(&htim2);

}

void LAStateMachine(void) {
    OPENAMP_check_for_message();
    if (VirtUart0RxMsg) {
      VirtUart0RxMsg = RESET;
      if (treatRxCommand2()) {
          if (mArrayDdrBuffCount >= 3) {
              uint32_t period = mTIM2freq / m_samp_freq;
              configureTimer(period);
              mArrayDdrBuffIndex = 0;
              mRollingCompSampCount = 0;
              mRawSampCount = 0;
              mSampCount = 0;
              mLastSamp = 0xFF;   // to be sure that the 1st comparison will fail in TIM IRQ
              mPrevSampCount = SAMP_SRAM_PACKET_SIZE;   // to allow circular buffer management 1st time
              mTotalCompSampCount = 0;
              HAL_StatusTypeDef res = HAL_DMAEx_MultiBufferStart_IT(htim2.hdma[TIM_DMA_ID_UPDATE], GPIOx_IDR,
                      (uint32_t)&mSampBuff[0], (uint32_t)&mSampBuff[SAMP_SRAM_PACKET_SIZE], SAMP_SRAM_PACKET_SIZE);
              if (res == HAL_OK) {
                  HAL_TIM_Base_Start(&htim2);
                  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_UPDATE);
                  sprintf(mUartBuffTx, "LAStateMachine starting...\n");
                  VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
              } else {
                  sprintf(mUartBuffTx, "LAStateMachine starting HAL_DMAEx_MultiBufferStart_IT errorCode=%ld\n", htim2.hdma[TIM_DMA_ID_UPDATE]->ErrorCode);
                  VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
              }
          } else {
              // should return an ERROR !!!
              sprintf(mUartBuffTx, "LAStateMachine cmd ERROR mArrayDdrBuffCount=%d\n", mArrayDdrBuffCount);
              VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
          }

      }
    }
    if (SDB0RxMsg) {
        SDB0RxMsg = RESET;
        treatSDBEvent();
    }
    if (fDdrDma2Send0) {
        fDdrDma2Send0 = RESET;
        if (HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream1, (uint32_t)&mSampBuffOut[0],
                mArrayDdrBuff[mArrayDdrBuffIndex].physAddr+mRollingCompSampCount, SAMP_SRAM_PACKET_SIZE) != HAL_OK)
        {
          /* Transfer Error */
            sprintf(mUartBuffTx, "LAStateMachine, fDdrDma2Send0 SET => HAL_DMA_Start_IT error !!!\n");
            VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
        }
    }
    if (fDdrDma2Send1) {
        fDdrDma2Send1 = RESET;
        if (HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream1, (uint32_t)&mSampBuffOut[SAMP_SRAM_PACKET_SIZE],
                mArrayDdrBuff[mArrayDdrBuffIndex].physAddr+mRollingCompSampCount, SAMP_SRAM_PACKET_SIZE) != HAL_OK)
        {
          /* Transfer Error */
            sprintf(mUartBuffTx, "LAStateMachine, fDdrDma2Send1 SET => HAL_DMA_Start_IT error !!!\n");
            VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
        }

    }
    if (fStopRequested) {
        fStopRequested = RESET;
        HAL_DMA_Abort_IT(&hdma_memtomem_dma2_stream1);
        HAL_DMA_Abort_IT(htim2.hdma[TIM_DMA_ID_UPDATE]);
        HAL_TIM_Base_Stop(&htim2);

    }
}

void testing(void) {
    m_samp_freq = 1000000;     // test @1MHz
    uint32_t period = mTIM2freq / m_samp_freq;
    configureTimer(period);
    mRawSampCount = 0;
    mSampCount = 0;
    mLastSamp = 0xFF;   // to be sure that the 1st comparison will fail in TIM IRQ
    mPrevSampCount = SAMP_SRAM_PACKET_SIZE;   // to allow circular buffer management 1st time
    mTotalCompSampCount = 0;

#ifdef DMA_SRAM
    HAL_StatusTypeDef res = HAL_DMAEx_MultiBufferStart_IT(htim2.hdma[TIM_DMA_ID_UPDATE], GPIOx_IDR,
            (uint32_t)&mSampBuff[0], (uint32_t)&mSampBuff[SAMP_SRAM_PACKET_SIZE], SAMP_SRAM_PACKET_SIZE);
    if (res == HAL_OK) {
        HAL_TIM_Base_Start(&htim2);
        __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_UPDATE);
        Log("testing...\n");
    } else {
        Log("testing, HAL_DMAEx_MultiBufferStart_IT errorCode=%ld\n", htim2.hdma[TIM_DMA_ID_UPDATE]->ErrorCode);
    }
#else
    HAL_TIM_Base_Start_IT(&htim2);
#endif
    //strcpy(SDB0ChannelBuffRx, testStr);
    //treatSDBEvent();

}

/****************************************************************************
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void)
{
  //volatile uint8_t debug_loop = 0;    // needed to let M4 perform a loop until the debugger attaches

  /* Initialize HAL : systick*/
  if (HAL_Init() != HAL_OK)
    Error_Handler();

  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();

#ifdef DMA_SRAM
  htim2.hdma[TIM_DMA_ID_UPDATE]->XferCpltCallback = TransferCompleteSRAM;
  htim2.hdma[TIM_DMA_ID_UPDATE]->XferM1CpltCallback = TransferCompleteSRAM;
  htim2.hdma[TIM_DMA_ID_UPDATE]->XferErrorCallback = TransferErrorSRAM;
#endif


  MX_DMA_Init();
  HAL_DMA_RegisterCallback(&hdma_memtomem_dma2_stream1, HAL_DMA_XFER_CPLT_CB_ID, TransferCompleteDDR);
  HAL_DMA_RegisterCallback(&hdma_memtomem_dma2_stream1, HAL_DMA_XFER_ERROR_CB_ID, TransferErrorDDR);


    /* Initialize OpenAMP framework
   * It will create a remote device handler
   * Then, It will be used to create new rpmsg channels
   * used as virtual device like UART0 and UART1 in this example
   * to communicate with the Master processor CPU1(CA7)
   */

  MX_IPCC_Init();
  MX_OPENAMP_Init(RPMSG_REMOTE, NULL);

  /*
   * Create Virtual UART device
   * defined by a rpmsg channel attached to the remote device
   */
  hsdb0.rvdev =  &rvdev;
  log_info("SDB OpenAMP-rpmsg channel creation\n");
  if (RPMSG_HDR_Init(&hsdb0) != RPMSG_HDR_OK) {
    log_err("RPMSG_HDR_Init HDR failed.\n");
    Error_Handler();
  }

  /*
   * Create Virtual UART device
   * defined by a rpmsg channel attached to the remote device
   */
  huart0.rvdev =  &rvdev;
  log_info("Virtual UART0 OpenAMP-rpmsg channel creation\n");
  if (VIRT_UART_Init(&huart0) != VIRT_UART_OK) {
    log_err("VIRT_UART_Init UART0 failed.\n");
    Error_Handler();
  }

  /*Need to register callback for message reception by channels*/
  if(RPMSG_HDR_RegisterCallback(&hsdb0, RPMSG_HDR_RXCPLT_CB_ID, SDB0_RxCpltCallback) != RPMSG_HDR_OK)
  {
    Error_Handler();
  }

  /*Need to register callback for message reception by channels*/
  if(VIRT_UART_RegisterCallback(&huart0, VIRT_UART_RXCPLT_CB_ID, VIRT_UART0_RxCpltCallback) != VIRT_UART_OK)
  {
    Error_Handler();
  }

  mMCUfreq = HAL_RCC_GetMCUFreq();
  mTIM2freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_TIMG1);

//  Log("mMCUfreq=%u mTIM2freq=%u\n",
//          (unsigned int)mMCUfreq, (unsigned int)mTIM2freq);
  sprintf(mUartBuffTx, "mMCUfreq=%u mTIM2freq=%u\n",
          (unsigned int)mMCUfreq, (unsigned int)mTIM2freq);
  VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));


  //Log("Cortex M4 boot successful with STM32Cube FW version: v%ld.%ld.%ld \n",
  //                                          ((HAL_GetHalVersion() >> 24) & 0x000000FF),
  //                                          ((HAL_GetHalVersion() >> 16) & 0x000000FF),
  //                                          ((HAL_GetHalVersion() >> 8) & 0x000000FF));

  sprintf(mUartBuffTx, "Cortex M4 boot successful with STM32Cube FW version: v%ld.%ld.%ld \n",
                                            ((HAL_GetHalVersion() >> 24) & 0x000000FF),
                                            ((HAL_GetHalVersion() >> 16) & 0x000000FF),
                                           ((HAL_GetHalVersion() >> 8) & 0x000000FF));
  VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));

  //while (debug_loop == 0);

  //testing();

  do {
      LAStateMachine();
    } while(1);


  return 0;
}

/*
 * Android command reception management
 *   UI state
 *   Buffer get
 */
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\n", file, line) */

	log_err("OOOps: file %s, line %d\n", __FILE__, __LINE__);

  /* Infinite loop */
  while (1) {
  }
}
#endif
