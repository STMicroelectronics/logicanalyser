/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "openamp.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "virt_uart.h"
#include "virt_uart.h"
#include "openamp_log.h"
#include "rpmsg_hdr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define COPRO_SYNC_SHUTDOWN_CHANNEL  IPCC_CHANNEL_3

/*
 * USE_COMPRESSION_IN_IRQ flag allows to perform the compression algo under IRQ
 * This does not work when optimization flag -O0 is set, even when compression variables
 * are put in volatile.
 * USE_COMPRESSION_IN_IRQ is working fine when optimization flag -Og is set and compression variables
 * are NOT put in volatile.
 */
//#define USE_COMPRESSION_IN_IRQ


/*
 * USE_COMPRESSION_IN_MAIN flag allows to perform the compression algo in main loop.
 * This configuration works with optimisation flag either set to -O0, either set to -Og,
 * only variables which are modified in IRQ context are set in volatile
 *
 */
#define USE_COMPRESSION_IN_MAIN

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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IPCC_HandleTypeDef hipcc;

TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_up;

DMA_HandleTypeDef hdma_memtomem_dma2_stream1;
/* USER CODE BEGIN PV */
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
volatile uint32_t mDmaSramCount = 0;
volatile uint64_t mTotalCompSampCount=0;
volatile uint32_t mRollingCompSampCount=0;

uint8_t VirtUart0ChannelBuffRx[100];
uint16_t VirtUart0ChannelRxSize = 0;
uint8_t SDB0ChannelBuffRx[100];
uint16_t SDB0ChannelRxSize = 0;
char mSdbBuffTx[512];

volatile uint32_t mMCUfreq, mTIM2freq;
uint32_t m_samp_freq;

volatile uint8_t mSampBuff[SAMP_SRAM_PACKET_SIZE * 2];   // use a circular buffer in SRAM
uint8_t mSampBuffOut[SAMP_SRAM_PACKET_SIZE * 2];   // use a circular buffer in SRAM
volatile static HDR_DdrBuffTypeDef mArrayDdrBuff[MAX_DDR_BUFF];    // used to store DDR buff allocated by Linux driver
volatile uint8_t mArrayDdrBuffCount = 0;
volatile uint8_t mArrayDdrBuffIndex = 0;  // will vary from 0 to mArrayDdrBuffCount-1
uint32_t mSampCount;    // nb of compressed sample
uint8_t mLastSamp;
int8_t mSampRepet;

char testStr[21] = {"B0Ad4200000L00100000"};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_IPCC_Init(void);
static void MX_TIM2_Init(void);
int MX_OPENAMP_Init(int RPMsgRole, rpmsg_ns_bind_cb ns_bind_cb);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  HW reset of used peripheral.
  * @retval none
  */
void resetPeripherals() {
    __HAL_RCC_TIM2_FORCE_RESET();
    //HAL_Delay(2);		// involves an issue in ShutdownCb (timeout)
    __HAL_RCC_TIM2_RELEASE_RESET();
}

/**
  * @brief  CallBack function which will be called when firmware will be stopped by Android application.
  * @retval none
  */
void CoproSync_ShutdownCb(IPCC_HandleTypeDef * hipcc, uint32_t ChannelIndex, IPCC_CHANNELDirTypeDef ChannelDir)
{
  /* Deinit the peripherals */

    HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
    HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);

    resetPeripherals();
    __HAL_RCC_TIM2_CLK_DISABLE();

	  /* When ready, notify the remote processor that we can be shut down */
	  HAL_IPCC_NotifyCPU(hipcc, ChannelIndex, IPCC_CHANNEL_DIR_RX);
}

void VIRT_UART0_RxCpltCallback(VIRT_UART_HandleTypeDef *huart)
{
    // copy received msg in a buffer
    VirtUart0ChannelRxSize = huart->RxXferSize < 100? huart->RxXferSize : 99;
    memcpy(VirtUart0ChannelBuffRx, huart->pRxBuffPtr, VirtUart0ChannelRxSize);
    VirtUart0ChannelBuffRx[VirtUart0ChannelRxSize] = 0;   // insure end of String
    sprintf(mUartBuffTx, "CM4 : VIRT_UART0_RxCpltCallback: %s\n", VirtUart0ChannelBuffRx);
    VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
    VirtUart0RxMsg = SET;
}

void SDB0_RxCpltCallback(RPMSG_HDR_HandleTypeDef *huart)
{
    // copy received msg in a buffer
    SDB0ChannelRxSize = huart->RxXferSize < 100? huart->RxXferSize : 99;
    memcpy(SDB0ChannelBuffRx, huart->pRxBuffPtr, SDB0ChannelRxSize);
    SDB0ChannelBuffRx[SDB0ChannelRxSize] = 0;   // insure end of String
    sprintf(mUartBuffTx, "CM4 : SDB0_RxCpltCallback: %s\n", SDB0ChannelBuffRx);
    VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
    SDB0RxMsg = SET;
}

static void TransferCompleteDDR(DMA_HandleTypeDef *DmaHandle)
{
    mTotalCompSampCount += SAMP_SRAM_PACKET_SIZE;
    mRollingCompSampCount += SAMP_SRAM_PACKET_SIZE;
    if ((mTotalCompSampCount % SAMP_DDR_BUFFER_SIZE) == 0) {
        // time to change DDR buffer and send MSG to Linux
        sprintf(mSdbBuffTx, "B%dL%08x", mArrayDdrBuffIndex, SAMP_DDR_BUFFER_SIZE);
        RPMSG_HDR_Transmit(&hsdb0, (uint8_t*)mSdbBuffTx, strlen(mSdbBuffTx));
        //sprintf(mUartBuffTx, "DMA2DDR-B%dL%08x", mArrayDdrBuffIndex, SAMP_DDR_BUFFER_SIZE);
        //VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
        mRollingCompSampCount = 0;
        mArrayDdrBuffIndex++;
        if (mArrayDdrBuffIndex == mArrayDdrBuffCount) {
            mArrayDdrBuffIndex = 0;
        }
    }
}

static void TransferCompleteSRAM(DMA_HandleTypeDef *DmaHandle)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);   // IRQ measurement thanks to GPIO management
#ifdef USE_COMPRESSION_IN_IRQ
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
#else
    if ((mDmaSramCount & 1) == 0) {
    	// 1st buffer filled
        fDdrDma2Send0 = SET;
    } else {
    	// 2nd buffer filled
        fDdrDma2Send1 = SET;
    }
    mDmaSramCount++;
#endif
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
}

static void TransferErrorDDR(DMA_HandleTypeDef *DmaHandle)
{
    mDMAerror = SET;
    sprintf(mUartBuffTx, "CM4 : DMA DDR TransferError CB !!!\n");
    VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
}

static void TransferErrorSRAM(DMA_HandleTypeDef *DmaHandle)
{
    mDMAerror = SET;
    sprintf(mUartBuffTx, "CM4 : DMA SRAM TransferError CB !!!\n");
    VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
}

uint8_t treatRxCommand2() {
    // example command: S002M => Sample, rate 002MHz
    if (VirtUart0ChannelBuffRx[0] == 'S') {
        for (int i=0; i<3; i++) {
            if (VirtUart0ChannelBuffRx[1+i] < '0' || VirtUart0ChannelBuffRx[1+i] > '9') {
                sprintf(mUartBuffTx, "CM4 : treatRxCommand2 ERROR wrong frequency digit:%c at offset:%d\n",
                        VirtUart0ChannelBuffRx[1+i], 1+i);
                VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));

                return false;
            }
        }
        if (VirtUart0ChannelBuffRx[4] != 'M' && VirtUart0ChannelBuffRx[4] != 'k' && VirtUart0ChannelBuffRx[4] != 'H') {
            sprintf(mUartBuffTx, "CM4 : treatRxCommand2 ERROR wrong frequency unit:%c\n", VirtUart0ChannelBuffRx[4]);
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
        sprintf(mUartBuffTx, "CM4 : treatRxCommand2 OK frequency=%ld\n", m_samp_freq);
        VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
        return true;
    } else if (VirtUart0ChannelBuffRx[0] == 'E') {
        // exit requested
        fStopRequested = SET;
    } else if (VirtUart0ChannelBuffRx[0] == 'r') {
    	  sprintf(mUartBuffTx, "CM4 : mMCUfreq=%u mTIM2freq=%u\n",
    	          (unsigned int)mMCUfreq, (unsigned int)mTIM2freq);
    	  VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));

    	  sprintf(mUartBuffTx, "CM4 : boot successful with STM32Cube FW version: v%ld.%ld.%ld \n",
    	                                            ((HAL_GetHalVersion() >> 24) & 0x000000FF),
    	                                            ((HAL_GetHalVersion() >> 16) & 0x000000FF),
    	                                           ((HAL_GetHalVersion() >> 8) & 0x000000FF));
    	  VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
    } else {
        sprintf(mUartBuffTx, "CM4 : treatRxCommand2 ERROR wrong frequency command:%c instead of: S\n",
                VirtUart0ChannelBuffRx[0]);
        VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
    }
    return false;
}

void treatSDBEvent() {
    // example command: B0AxxxxxxxxLyyyyyyyy => Buff0 @:xx..x Length:yy..y
    if (SDB0ChannelBuffRx[0] != 'B') {
        sprintf(mUartBuffTx, "CM4 : treatSDBEvent ERROR wrong buffer command:%c\n", SDB0ChannelBuffRx[0]);
        VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
    }
    if (!((SDB0ChannelBuffRx[1] >= '0' && SDB0ChannelBuffRx[1] <= '3'))) {
        sprintf(mUartBuffTx, "CM4 : treatSDBEvent ERROR wrong buffer index:%c\n", SDB0ChannelBuffRx[1]);
        VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
        return;
    }
    if (mArrayDdrBuffCount != (SDB0ChannelBuffRx[1] - '0')) {
        sprintf(mUartBuffTx, "CM4 : treatSDBEvent ERROR wrong buffer received index:%c awaited index:%d\n",
                SDB0ChannelBuffRx[1], mArrayDdrBuffCount);
        VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
        return;
    }
    if (SDB0ChannelBuffRx[2] != 'A') {
        sprintf(mUartBuffTx, "CM4 : treatSDBEvent ERROR wrong buffer address tag:%c instead of:A\n", SDB0ChannelBuffRx[1]);
        VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
        return ;
    }
    if (SDB0ChannelBuffRx[11] != 'L') {
        sprintf(mUartBuffTx, "CM4 : treatSDBEvent ERROR wrong buffer length tag:%c instead of:L\n", SDB0ChannelBuffRx[11]);
        VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
        return ;
    }
    for (int i=0; i<8; i++) {
        if (!((SDB0ChannelBuffRx[3+i] >= '0' && SDB0ChannelBuffRx[3+i] <= '9') || (SDB0ChannelBuffRx[3+i] >= 'a' && SDB0ChannelBuffRx[3+i] <= 'f'))) {
            sprintf(mUartBuffTx, "CM4 : treatSDBEvent ERROR wrong address digit:%c at offset:%d\n", SDB0ChannelBuffRx[3+i], 3+i);
            VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
            return ;
        }
        if (!((SDB0ChannelBuffRx[12+i] >= '0' && SDB0ChannelBuffRx[12+i] <= '9') || (SDB0ChannelBuffRx[12+i] >= 'a' && SDB0ChannelBuffRx[12+i] <= 'f'))) {
            sprintf(mUartBuffTx, "CM4 : treatSDBEvent ERROR wrong length digit:%c at offset:%d\n", SDB0ChannelBuffRx[12+i], 12+i);
            VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
            return ;
        }
    }
    // save DDR buff @ and size
    mArrayDdrBuff[mArrayDdrBuffCount].physAddr = (uint32_t)strtoll((char*)&SDB0ChannelBuffRx[3], NULL, 16);
    mArrayDdrBuff[mArrayDdrBuffCount].physSize = (uint32_t)strtoll((char*)&SDB0ChannelBuffRx[12], NULL, 16);
    sprintf(mUartBuffTx, "CM4 : treatSDBEvent OK physAddr=0x%lx physSize=%ld mArrayDdrBuffCount=%d\n",
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
              mDmaSramCount = 0;	// to know which SRAM buffer will be filled
              mSampCount = 0;
              mLastSamp = 0xFF;   // to be sure that the 1st comparison will fail in compr. algo.
              mTotalCompSampCount = 0;
              HAL_StatusTypeDef res = HAL_DMAEx_MultiBufferStart_IT(htim2.hdma[TIM_DMA_ID_UPDATE], GPIOx_IDR,
                      (uint32_t)&mSampBuff[0], (uint32_t)&mSampBuff[SAMP_SRAM_PACKET_SIZE], SAMP_SRAM_PACKET_SIZE);
              if (res == HAL_OK) {
                  HAL_TIM_Base_Start(&htim2);
                  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_UPDATE);
                  sprintf(mUartBuffTx, "CM4 : LAStateMachine starting...\n");
                  VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
              } else {
                  sprintf(mUartBuffTx, "CM4 : LAStateMachine starting HAL_DMAEx_MultiBufferStart_IT errorCode=%ld\n", htim2.hdma[TIM_DMA_ID_UPDATE]->ErrorCode);
                  VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
              }
          } else {
              // should return an ERROR !!!
              sprintf(mUartBuffTx, "CM4 : LAStateMachine cmd ERROR mArrayDdrBuffCount=%d\n", mArrayDdrBuffCount);
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
#ifdef USE_COMPRESSION_IN_IRQ
        if (HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream1, (uint32_t)&mSampBuffOut[0],
                mArrayDdrBuff[mArrayDdrBuffIndex].physAddr+mRollingCompSampCount, SAMP_SRAM_PACKET_SIZE) != HAL_OK)
        {
          /* Transfer Error */
            sprintf(mUartBuffTx, "CM4 : LAStateMachine, fDdrDma2Send0 SET => HAL_DMA_Start_IT error !!!\n");
            VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
        }
#else
#ifdef USE_COMPRESSION_IN_MAIN
		for (int i=0; i<SAMP_SRAM_PACKET_SIZE; i++) {
			/*
			 * mLastSamp saves the last sample value where:
			 * - the 3 MSBs are set to 1 (compression algo has not yet been performed once
			 * - the 3 MSBs are masked to 0 (compression algo has already been performed)
			 * When Linux application request to start sampling, it sets mLastSamp to 0xFF (3 MSBs to 1)
			 */
			if (mLastSamp != 0xFF) {  // needed to start the algo correctly
				if (mLastSamp == (mSampBuff[i] & 0x1F)) {
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
					mLastSamp = (mSampBuff[i] & 0x1F);
					mSampRepet = 0;   // 1 occurence seen
				}
			} else {
				mLastSamp = (mSampBuff[i] & 0x1F);
				mSampRepet = 0;   // 0 means 1 occurence, 7 means 8 occurences
			}
			if (mSampCount == SAMP_SRAM_PACKET_SIZE) {
				// time to transfer compressed buffer0 in DDR
		        HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream1, (uint32_t)&mSampBuffOut[0],
		                mArrayDdrBuff[mArrayDdrBuffIndex].physAddr+mRollingCompSampCount, SAMP_SRAM_PACKET_SIZE);
			} else if (mSampCount == (SAMP_SRAM_PACKET_SIZE*2)) {
				// time to transfer compressed buffer1 in DDR
		        HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream1, (uint32_t)&mSampBuffOut[SAMP_SRAM_PACKET_SIZE],
		                mArrayDdrBuff[mArrayDdrBuffIndex].physAddr+mRollingCompSampCount, SAMP_SRAM_PACKET_SIZE);
				mSampCount = 0;
			}
		}
#else
    	if (HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream1, (uint32_t)&mSampBuff[0],
			mArrayDdrBuff[mArrayDdrBuffIndex].physAddr+mRollingCompSampCount, SAMP_SRAM_PACKET_SIZE) != HAL_OK)
        {
          /* Transfer Error */
            sprintf(mUartBuffTx, "CM4 : LAStateMachine, fDdrDma2Send0 SET => HAL_DMA_Start_IT error !!!\n");
            VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
        }
#endif

#endif
    }
    if (fDdrDma2Send1) {
        fDdrDma2Send1 = RESET;
#ifdef USE_COMPRESSION_IN_IRQ
        if (HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream1, (uint32_t)&mSampBuffOut[SAMP_SRAM_PACKET_SIZE],
                mArrayDdrBuff[mArrayDdrBuffIndex].physAddr+mRollingCompSampCount, SAMP_SRAM_PACKET_SIZE) != HAL_OK)
        {
          /* Transfer Error */
            sprintf(mUartBuffTx, "CM4 : LAStateMachine, fDdrDma2Send0 SET => HAL_DMA_Start_IT error !!!\n");
            VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
        }
#else
#ifdef USE_COMPRESSION_IN_MAIN
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
		        HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream1, (uint32_t)&mSampBuffOut[0],
		                mArrayDdrBuff[mArrayDdrBuffIndex].physAddr+mRollingCompSampCount, SAMP_SRAM_PACKET_SIZE);
			} else if (mSampCount == (SAMP_SRAM_PACKET_SIZE*2)) {
				// time to transfer compressed buffer1 in DDR
		        HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream1, (uint32_t)&mSampBuffOut[SAMP_SRAM_PACKET_SIZE],
		                mArrayDdrBuff[mArrayDdrBuffIndex].physAddr+mRollingCompSampCount, SAMP_SRAM_PACKET_SIZE);
				mSampCount = 0;
			}
		}
#else
		if (HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream1, (uint32_t)&mSampBuff[SAMP_SRAM_PACKET_SIZE],
				mArrayDdrBuff[mArrayDdrBuffIndex].physAddr+mRollingCompSampCount, SAMP_SRAM_PACKET_SIZE) != HAL_OK)
        {
          /* Transfer Error */
            sprintf(mUartBuffTx, "CM4 : LAStateMachine, fDdrDma2Send1 SET => HAL_DMA_Start_IT error !!!\n");
            VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
        }
#endif
#endif

    }
    if (fStopRequested) {
        fStopRequested = RESET;
        HAL_DMA_Abort_IT(&hdma_memtomem_dma2_stream1);
        HAL_DMA_Abort_IT(htim2.hdma[TIM_DMA_ID_UPDATE]);
        HAL_TIM_Base_Stop(&htim2);

    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();

  // Major difference with MCU, state of resources is not known => HW reset before initialization
  resetPeripherals();
  /* USER CODE END Init */

  if(IS_ENGINEERING_BOOT_MODE())
  {
    /* Configure the system clock */
    SystemClock_Config();
  }

  if(IS_ENGINEERING_BOOT_MODE())
  {
    /* Configure the peripherals common clocks */
    PeriphCommonClock_Config();
  }
  else
  {
    /* IPCC initialisation */
     MX_IPCC_Init();
    /* OpenAmp initialisation ---------------------------------*/
    MX_OPENAMP_Init(RPMSG_REMOTE, NULL);
  }

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  htim2.hdma[TIM_DMA_ID_UPDATE]->XferCpltCallback = TransferCompleteSRAM;
  htim2.hdma[TIM_DMA_ID_UPDATE]->XferM1CpltCallback = TransferCompleteSRAM;
  htim2.hdma[TIM_DMA_ID_UPDATE]->XferErrorCallback = TransferErrorSRAM;

  HAL_DMA_RegisterCallback(&hdma_memtomem_dma2_stream1, HAL_DMA_XFER_CPLT_CB_ID, TransferCompleteDDR);
  HAL_DMA_RegisterCallback(&hdma_memtomem_dma2_stream1, HAL_DMA_XFER_ERROR_CB_ID, TransferErrorDDR);

  HAL_IPCC_ActivateNotification(&hipcc, COPRO_SYNC_SHUTDOWN_CHANNEL, IPCC_CHANNEL_DIR_RX,
            CoproSync_ShutdownCb);

  /*
   * Create HDR device
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
  log_info("Virtual UART0 OpenAMP-rpmsg channel creation\r\n");
  if (VIRT_UART_Init(&huart0) != VIRT_UART_OK) {
    log_err("VIRT_UART_Init UART0 failed.\r\n");
    //_Error_Handler(__FILE__, __LINE__);
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

  //mMCUfreq = HAL_RCCEx_GetSystemCoreClockFreq();
  mMCUfreq = HAL_RCC_GetMCUFreq();
  mTIM2freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_TIMG1);

  //while (debug_loop == 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    LAStateMachine();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS_DIG;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSIDivValue = RCC_HSI_DIV1;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL12SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 81;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLFRACV = 0x800;
  RCC_OscInitStruct.PLL.PLLMODE = RCC_PLL_FRACTIONAL;
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL2.PLLSource = RCC_PLL12SOURCE_HSE;
  RCC_OscInitStruct.PLL2.PLLM = 3;
  RCC_OscInitStruct.PLL2.PLLN = 66;
  RCC_OscInitStruct.PLL2.PLLP = 2;
  RCC_OscInitStruct.PLL2.PLLQ = 1;
  RCC_OscInitStruct.PLL2.PLLR = 1;
  RCC_OscInitStruct.PLL2.PLLFRACV = 0x1400;
  RCC_OscInitStruct.PLL2.PLLMODE = RCC_PLL_FRACTIONAL;
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL3.PLLSource = RCC_PLL3SOURCE_HSE;
  RCC_OscInitStruct.PLL3.PLLM = 2;
  RCC_OscInitStruct.PLL3.PLLN = 34;
  RCC_OscInitStruct.PLL3.PLLP = 2;
  RCC_OscInitStruct.PLL3.PLLQ = 17;
  RCC_OscInitStruct.PLL3.PLLR = 37;
  RCC_OscInitStruct.PLL3.PLLRGE = RCC_PLL3IFRANGE_1;
  RCC_OscInitStruct.PLL3.PLLFRACV = 6660;
  RCC_OscInitStruct.PLL3.PLLMODE = RCC_PLL_FRACTIONAL;
  RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL4.PLLSource = RCC_PLL4SOURCE_HSE;
  RCC_OscInitStruct.PLL4.PLLM = 4;
  RCC_OscInitStruct.PLL4.PLLN = 99;
  RCC_OscInitStruct.PLL4.PLLP = 6;
  RCC_OscInitStruct.PLL4.PLLQ = 8;
  RCC_OscInitStruct.PLL4.PLLR = 8;
  RCC_OscInitStruct.PLL4.PLLRGE = RCC_PLL4IFRANGE_0;
  RCC_OscInitStruct.PLL4.PLLFRACV = 0;
  RCC_OscInitStruct.PLL4.PLLMODE = RCC_PLL_INTEGER;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** RCC Clock Config
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_ACLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3|RCC_CLOCKTYPE_PCLK4
                              |RCC_CLOCKTYPE_PCLK5|RCC_CLOCKTYPE_MPU;
  RCC_ClkInitStruct.MPUInit.MPU_Clock = RCC_MPUSOURCE_PLL1;
  RCC_ClkInitStruct.MPUInit.MPU_Div = RCC_MPU_DIV2;
  RCC_ClkInitStruct.AXISSInit.AXI_Clock = RCC_AXISSOURCE_PLL2;
  RCC_ClkInitStruct.AXISSInit.AXI_Div = RCC_AXI_DIV1;
  RCC_ClkInitStruct.MCUInit.MCU_Clock = RCC_MCUSSOURCE_PLL3;
  RCC_ClkInitStruct.MCUInit.MCU_Div = RCC_MCU_DIV1;
  RCC_ClkInitStruct.APB4_Div = RCC_APB4_DIV2;
  RCC_ClkInitStruct.APB5_Div = RCC_APB5_DIV4;
  RCC_ClkInitStruct.APB1_Div = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2_Div = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB3_Div = RCC_APB3_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Set the HSE division factor for RTC clock
  */
  __HAL_RCC_RTC_HSEDIV(24);
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the common periph clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInit.CkperClockSelection = RCC_CKPERCLKSOURCE_HSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IPCC Initialization Function
  * @param None
  * @retval None
  */
static void MX_IPCC_Init(void)
{

  /* USER CODE BEGIN IPCC_Init 0 */

  /* USER CODE END IPCC_Init 0 */

  /* USER CODE BEGIN IPCC_Init 1 */

  /* USER CODE END IPCC_Init 1 */
  hipcc.Instance = IPCC;
  if (HAL_IPCC_Init(&hipcc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IPCC_Init 2 */

  /* USER CODE END IPCC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 16;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream1
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_stream1 on DMA2_Stream1 */
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
    Error_Handler( );
  }

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pins : LA_3_Pin LA_4_Pin LA_2_Pin LA_1_Pin
                           LA_0_Pin */
  GPIO_InitStruct.Pin = LA_3_Pin|LA_4_Pin|LA_2_Pin|LA_1_Pin
                          |LA_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  sprintf(mUartBuffTx, "CM4 : OOOps: Error_Handler\n");
  VIRT_UART_Transmit(&huart0, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
