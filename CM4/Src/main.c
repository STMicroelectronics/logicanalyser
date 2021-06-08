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
/*
 * This demonstration shows code examples of inter-core data exchanges
 * For high data rate (more than 5MHz sampling), it relies on a SDB Linux driver
 *  which provides DDR buffers allocations, and DDR DMA transfers
 *
 * For low data rate (less or equal to 5MHz sampling), it relies on VirtualUART
 *  over RPMSG
 *
 * 2 virtualUARTS are created:
 *  UART0 for A7 to M4 sampling commands (start/stop), and low data rate M4 to A7
 *  UART1 for M4 trace
 *
 * LogicAnalyser sampling is done on PE8..14 available on Arduino connector of MB1372
 *
 * Masking algorithm is done on M4 side, on MSB on data
 *
 * Sampling is done in SARM buffer thanks to TIM2 and DMA2
 *  PE7..17 => DMA2 => SRAM_buff
 *  DMA half and DMA full interrupts are used to avoid data over writing
 *
 * The number of DDR buffers is sent by Linux application at its start
 *
 * Masking of bit7 is then performed by M4
 *
 * Finally masked data buffers are transfered to DDR by DMA or virtualUART
 *
 * Linux application send the "Set DATA" parameter in the Sampling command.
 * If "Set DATA" is set output data will be set by this FW on PE8..12, and these Ports will be initialized as output
 *  output data values are changed in TransferCompleteSRAM every 23 times
 *
 * On UI, refresh is done every new MB of compressed data
 *
 */
#include "virt_uart.h"
#include "virt_uart.h"
#include "openamp_log.h"
#include "rpmsg_hdr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define COPRO_SYNC_SHUTDOWN_CHANNEL  IPCC_CHANNEL_3
#define TRC_LA 1
//#define COMPRESS
#define SET_DATA 1

typedef enum {
    RISING,                 /*  0 */
    FALLING,                /*  1 */
    BOTH,                /*  1 */
    NONE,                /*  1 */
} sampling_edge_t;

typedef enum {
    INITIALIZING,               /*  0 */
    DDR_BUFFERS_OK,             /*  1 */
    SAMPLING_LOW_FREQ_BUFF1,    /*  2 */
    SAMPLING_LOW_FREQ_BUFF2,    /*  3 */
    SAMPLING_HIGH_FREQ_BUFF1,   /*  4 */
    SAMPLING_HIGH_FREQ_BUFF2,   /*  5 */
} Machine_State_t;

typedef struct __HDR_DdrBuffTypeDef
{
  uint32_t physAddr;
  uint32_t physSize;
}HDR_DdrBuffTypeDef;

#define SAMP_SRAM_RPMSG_PACKET_SIZE 	(256*2)
#define SAMP_SRAM_DDR_DMA_PACKET_SIZE 	(1024*2)
#define SAMP_DDR_BUFFER_SIZE 		(1024*1024)
#define SAMP_PERIPH_2_DDR_BUFFER_SIZE (32*1024)

#define DMA_MEM_BUFF_SIZE   ((uint32_t) 512)      /* size in byte.. must be aligned with SDRAM word access..so must be a multiple of 4 ... must be less than DMA counter 65535  */

#define MAX_DDR_BUFF 16

#define GPIOx_IDR               (GPIOE_BASE + 0x10 + 1)     // portE Input bit 8..12
#define GPIOx_ODR               (GPIOE_BASE + 0x14 + 1)     // portE Output bit 8..12
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

/* USER CODE BEGIN PV */
static struct rpmsg_virtio_device rvdev;
RPMSG_HDR_HandleTypeDef hsdb0;
VIRT_UART_HandleTypeDef huart0, huart1;

__IO FlagStatus VirtUart0RxMsg = RESET;
__IO FlagStatus SDB0RxMsg = RESET;
__IO FlagStatus VirtUart0OK = RESET;
__IO FlagStatus fDmaBuffAvailable = RESET;
__IO FlagStatus mDMAerror = RESET;
char mUartBuffTx[512];
char mUartBuffTx1[512];
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

volatile uint8_t mSampBuff[SAMP_SRAM_DDR_DMA_PACKET_SIZE];   	// declare maximum size for DDR DMA
uint8_t mSampBuffOut[SAMP_SRAM_DDR_DMA_PACKET_SIZE];   			// use a circular buffer in SRAM
volatile static HDR_DdrBuffTypeDef mArrayDdrBuff[MAX_DDR_BUFF]; // used to store DDR buff allocated by Linux driver
volatile uint8_t mArrayDdrBuffCount = 0;
volatile uint8_t mArrayDdrBuffIndex = 0;  						// will vary from 0 to mArrayDdrBuffCount-1
uint8_t mDdrBuffCount = 3;		// by default 3 DDR buff, this should be a parameter of sampling command
uint16_t mSramPacketSize;
uint32_t mSampCount;    // nb of compressed sample
uint8_t mLastSamp;
int8_t mSampRepet;

int8_t mSetData = 0;
uint8_t mPatternOut = 0, mPatterPreCounter = 0;

Machine_State_t mMachineState = INITIALIZING;
char testStr[21] = {"B0Ad4200000L00100000"};

volatile uint32_t mDmaSramISR;

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

/**
  * @brief  CallBack function which will be called when UART0 data is received from A7.
  * @retval none
  */
void VIRT_UART0_RxCpltCallback(VIRT_UART_HandleTypeDef *huart)
{
    // copy received msg in a buffer
    VirtUart0ChannelRxSize = huart->RxXferSize < 100? huart->RxXferSize : 99;
    memcpy(VirtUart0ChannelBuffRx, huart->pRxBuffPtr, VirtUart0ChannelRxSize);
    VirtUart0ChannelBuffRx[VirtUart0ChannelRxSize] = 0;   // insure end of String
#if TRC_LA
    sprintf(mUartBuffTx1, "CM4 : VIRT_UART0_RxCpltCallback: %s\n", VirtUart0ChannelBuffRx);
    VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx1, strlen(mUartBuffTx1));
#endif
    VirtUart0RxMsg = SET;
}

/**
  * @brief  CallBack function which will be called when a DDR buffers is allocated by Linux.
  * @retval none
  */
void SDB0_RxCpltCallback(RPMSG_HDR_HandleTypeDef *huart)
{
    // copy received msg in a buffer
    SDB0ChannelRxSize = huart->RxXferSize < 100? huart->RxXferSize : 99;
    memcpy(SDB0ChannelBuffRx, huart->pRxBuffPtr, SDB0ChannelRxSize);
    SDB0ChannelBuffRx[SDB0ChannelRxSize] = 0;   // insure end of String
#if TRC_LA
    sprintf(mUartBuffTx1, "CM4 : SDB0_RxCpltCallback: %s\n", SDB0ChannelBuffRx);
    VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx1, strlen(mUartBuffTx1));
#endif
    SDB0RxMsg = SET;
}

/**
  * @brief  CallBack function which will be called when 1st half SRAM buffer has been DMA filled.
  * @retval none
  */
static void HalfTransferCompleteSRAM(DMA_HandleTypeDef *DmaHandle)
{
	// 1st half buffer filled
	fDdrDma2Send0 = SET;
	HAL_GPIO_WritePin(LA_TRC0_GPIO_Port, LA_TRC0_Pin, GPIO_PIN_SET);
}

/**
  * @brief  CallBack function which will be called when 2nd half SRAM buffer has been DMA filled.
  * @retval none
  */
static void TransferCompleteSRAM(DMA_HandleTypeDef *DmaHandle)
{
    uint8_t v;
    // 2nd half buffer filled
	fDdrDma2Send1 = SET;
	HAL_GPIO_WritePin(LA_TRC0_GPIO_Port, LA_TRC0_Pin, GPIO_PIN_RESET);
	if (mSetData) {
		mPatterPreCounter++;
		if (mPatterPreCounter % 23 == 0) {
			mPatternOut++;
			if (mPatternOut > 7) {
				mPatternOut = 0;
			}
		}
		// pattern put on PE8..14 is composed of : BIT6..3 DDR buff index BIT2..0 counter incremented every 23*2 SRAM buffer
		v = ((mArrayDdrBuffIndex & 15) << 3) | mPatternOut;
		GPIOE->ODR &= 0xFFFF80FF;			// 2nd byte of ODR is PE8..15
		GPIOE->ODR |= (v << 8);
	}
}

/**
  * @brief  CallBack function which will be called in case of SRAM DMA error.
  * @retval none
  */
static void TransferErrorSRAM(DMA_HandleTypeDef *DmaHandle)
{
    mDMAerror = SET;
#if TRC_LA
    sprintf(mUartBuffTx1, "CM4 : DMA TransferError SRAM CB ISR=0x%lx !!!\n",
    		mDmaSramISR);
    VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx1, strlen(mUartBuffTx1));
#endif
}

/**
  * @brief  Function which treats a command coming from A7.
  * @retval boolean command OK or not
  */
uint8_t treatRxCommand2() {
    // example command: S002M => Sample, rate 002MHz
    // example command: S002Msy => Sample, rate 002MHz, SET DATA yes
    if (VirtUart0ChannelBuffRx[0] == 'S') {
        for (int i=0; i<3; i++) {
            if (VirtUart0ChannelBuffRx[1+i] < '0' || VirtUart0ChannelBuffRx[1+i] > '9') {
#if TRC_LA
                sprintf(mUartBuffTx, "CM4 : treatRxCommand2 ERROR wrong frequency digit:%c at offset:%d\n",
                        VirtUart0ChannelBuffRx[1+i], 1+i);
                VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
#endif

                return false;
            }
        }
        if (VirtUart0ChannelBuffRx[4] != 'M' && VirtUart0ChannelBuffRx[4] != 'k' && VirtUart0ChannelBuffRx[4] != 'H') {
#if TRC_LA
            sprintf(mUartBuffTx, "CM4 : treatRxCommand2 ERROR wrong frequency unit:%c\n", VirtUart0ChannelBuffRx[4]);
            VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
#endif
            return false;
        }
        if (VirtUart0ChannelBuffRx[5] != 's') {
#if TRC_LA
            sprintf(mUartBuffTx, "CM4 : treatRxCommand2 ERROR wrong SET DATA tag:%c\n", VirtUart0ChannelBuffRx[5]);
            VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
#endif
            return false;
        }
        if (VirtUart0ChannelBuffRx[6] != 'y' && VirtUart0ChannelBuffRx[6] != 'n') {
#if TRC_LA
            sprintf(mUartBuffTx, "CM4 : treatRxCommand2 ERROR wrong SET DATA value:%c\n", VirtUart0ChannelBuffRx[6]);
            VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
#endif
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
        if (VirtUart0ChannelBuffRx[6] == 'y') {
			mSetData = 1;
        } else {
    		mSetData = 0;
        }
#if TRC_LA
        sprintf(mUartBuffTx1, "CM4 : treatRxCommand2 OK frequency=%ld  setData=%d\n",
        		m_samp_freq, mSetData);
        VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx1, strlen(mUartBuffTx1));
#endif
        return 'S';
    } else if (VirtUart0ChannelBuffRx[0] == 'B') {
        for (int i=0; i<2; i++) {
            if (VirtUart0ChannelBuffRx[1+i] < '0' || VirtUart0ChannelBuffRx[1+i] > '9') {
#if TRC_LA
                sprintf(mUartBuffTx, "CM4 : treatRxCommand2 ERROR wrong DDR Buffer digit:%c at offset:%d\n",
                        VirtUart0ChannelBuffRx[1+i], 1+i);
                VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
#endif

                return false;
            }
        }
        mDdrBuffCount = (int)(VirtUart0ChannelBuffRx[1] - '0') * 10
                + (int)(VirtUart0ChannelBuffRx[2] - '0') * 1;
#if TRC_LA
		sprintf(mUartBuffTx, "CM4 : treatRxCommand2 DDR BUFFER command => mDdrBuffCount=%d\n",
				mDdrBuffCount);
		VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
#endif
        return 'B';
    } else if (VirtUart0ChannelBuffRx[0] == 'E') {
        // exit requested
        fStopRequested = SET;
        return 'E';
    }
    return false;
}

/**
  * @brief  Function which treats a buffer allocation.
  * @retval none
  */
void treatSDBEvent() {
    // example command: B0AxxxxxxxxLyyyyyyyy => Buff0 @:xx..x Length:yy..y
    if (SDB0ChannelBuffRx[0] != 'B') {
#if TRC_LA
        sprintf(mUartBuffTx1, "CM4 : treatSDBEvent ERROR wrong buffer command:%c\n", SDB0ChannelBuffRx[0]);
        VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx1, strlen(mUartBuffTx1));
#endif
    }
    if (!((SDB0ChannelBuffRx[1] >= '0' && SDB0ChannelBuffRx[1] <= '9'))) {
        if (!((SDB0ChannelBuffRx[1] >= 'A' && SDB0ChannelBuffRx[1] <= 'F'))) {
#if TRC_LA
			sprintf(mUartBuffTx1, "CM4 : treatSDBEvent ERROR wrong buffer index:%c\n", SDB0ChannelBuffRx[1]);
			VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx1, strlen(mUartBuffTx1));
#endif
			return;
        } else {
            if (mArrayDdrBuffCount != (SDB0ChannelBuffRx[1] - 'A' + 10)) {
#if TRC_LA
				sprintf(mUartBuffTx1, "CM4 : treatSDBEvent ERROR wrong buffer received index:%c awaited index:%d\n",
						SDB0ChannelBuffRx[1], mArrayDdrBuffCount);
				VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx1, strlen(mUartBuffTx1));
#endif

            }
    	}
    }
    if (mArrayDdrBuffCount != (SDB0ChannelBuffRx[1] - '0')) {
#if TRC_LA
        sprintf(mUartBuffTx1, "CM4 : treatSDBEvent ERROR wrong buffer received index:%c awaited index:%d\n",
                SDB0ChannelBuffRx[1], mArrayDdrBuffCount);
        VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx1, strlen(mUartBuffTx1));
#endif
        return;
    }
    if (SDB0ChannelBuffRx[2] != 'A') {
#if TRC_LA
        sprintf(mUartBuffTx1, "CM4 : treatSDBEvent ERROR wrong buffer address tag:%c instead of:A\n", SDB0ChannelBuffRx[1]);
        VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx1, strlen(mUartBuffTx1));
#endif
        return ;
    }
    if (SDB0ChannelBuffRx[11] != 'L') {
#if TRC_LA
        sprintf(mUartBuffTx1, "CM4 : treatSDBEvent ERROR wrong buffer length tag:%c instead of:L\n", SDB0ChannelBuffRx[11]);
        VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx1, strlen(mUartBuffTx1));
#endif
        return ;
    }
    for (int i=0; i<8; i++) {
        if (!((SDB0ChannelBuffRx[3+i] >= '0' && SDB0ChannelBuffRx[3+i] <= '9') || (SDB0ChannelBuffRx[3+i] >= 'a' && SDB0ChannelBuffRx[3+i] <= 'f'))) {
#if TRC_LA
            sprintf(mUartBuffTx1, "CM4 : treatSDBEvent ERROR wrong address digit:%c at offset:%d\n", SDB0ChannelBuffRx[3+i], 3+i);
            VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx1, strlen(mUartBuffTx1));
#endif
            return ;
        }
        if (!((SDB0ChannelBuffRx[12+i] >= '0' && SDB0ChannelBuffRx[12+i] <= '9') || (SDB0ChannelBuffRx[12+i] >= 'a' && SDB0ChannelBuffRx[12+i] <= 'f'))) {
#if TRC_LA
            sprintf(mUartBuffTx1, "CM4 : treatSDBEvent ERROR wrong length digit:%c at offset:%d\n", SDB0ChannelBuffRx[12+i], 12+i);
            VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx1, strlen(mUartBuffTx1));
#endif
            return ;
        }
    }
    // save DDR buff @ and size
    mArrayDdrBuff[mArrayDdrBuffCount].physAddr = (uint32_t)strtoll((char*)&SDB0ChannelBuffRx[3], NULL, 16);
    mArrayDdrBuff[mArrayDdrBuffCount].physSize = (uint32_t)strtoll((char*)&SDB0ChannelBuffRx[12], NULL, 16);
#if TRC_LA
    sprintf(mUartBuffTx1, "CM4 : treatSDBEvent OK physAddr=0x%lx physSize=%ld mArrayDdrBuffCount=%d\n",
            mArrayDdrBuff[mArrayDdrBuffCount].physAddr,
            mArrayDdrBuff[mArrayDdrBuffCount].physSize,
            mArrayDdrBuffCount);
    VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx1, strlen(mUartBuffTx1));
#endif

    mArrayDdrBuffCount++;
}


/**
  * @brief  Function which configure the TIM period according to Sampling frequency.
  * @retval none
  */
void configureTimer(uint32_t period) {
    htim2.Init.Period = period;    // ex: m_samp_freq=12MHz => period = 16
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    HAL_TIM_Base_Init(&htim2);

}

void config_GPIOs(uint8_t setData) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

if (setData) {
  HAL_GPIO_WritePin(GPIOE, LA_6_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, LA_5_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, LA_4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, LA_3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, LA_2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, LA_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, LA_0_Pin, GPIO_PIN_RESET);
}
  GPIO_InitStruct.Pin = LA_6_Pin|LA_5_Pin|LA_4_Pin|LA_3_Pin
		  |LA_2_Pin|LA_1_Pin|LA_0_Pin;
if (setData) {
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
} else {
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
}
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/**
  * @brief  Function which implements tha LogicAnalyser machine state.
  * @retval none
  */
void LAStateMachine(void) {
    OPENAMP_check_for_message();
    switch (mMachineState) {
    case INITIALIZING:
    	// waiting for SDB driver events, with at least 3 DDR buffers
        if (SDB0RxMsg) {
            SDB0RxMsg = RESET;
            treatSDBEvent();
        	if (mArrayDdrBuffCount >= mDdrBuffCount) {	// is number of buffer in line with received command ?
        		mMachineState = DDR_BUFFERS_OK;
        		  sprintf(mUartBuffTx1, "CM4 : LA ready MCU freq=%lu TIM2 freq=%lu mDdrBuffCount=%d\n",
        				  mMCUfreq, mTIM2freq, mDdrBuffCount);
        		  VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx1, strlen(mUartBuffTx1));
        	}
        }
        if (VirtUart0RxMsg) {
          VirtUart0RxMsg = RESET;
          treatRxCommand2();	// to receive DDR BUFFER command
        }
    	break;
    case DDR_BUFFERS_OK:
    	// ready to accept a sampling command
        if (VirtUart0RxMsg) {
          VirtUart0RxMsg = RESET;
          if (treatRxCommand2() == 'S') {
    		  // check SET DATA and set GPIOs accordingly
    		  config_GPIOs(mSetData);
        	  if (m_samp_freq > 5000000) {
        		  //   if > 4MHz => no compression & Data over DMA
				  uint32_t period = mTIM2freq / m_samp_freq;
				  configureTimer(period);
				  mArrayDdrBuffIndex = 0;
				  mRollingCompSampCount = 0;
				  mSampCount = 0;
				  mLastSamp = 0xFF;   // to be sure that the 1st comparison will fail in compr. algo.
				  mTotalCompSampCount = 0;
				  mSramPacketSize = SAMP_SRAM_DDR_DMA_PACKET_SIZE;
				  // in DDR DMA transfer we will transfer by packet of 1024 bytes (4 times the size of RPMSG)
				  HAL_StatusTypeDef res = HAL_DMA_Start_IT(htim2.hdma[TIM_DMA_ID_UPDATE], GPIOx_IDR, (uint32_t)&mSampBuff[0], mSramPacketSize);
				  if (res == HAL_OK) {
					  HAL_TIM_Base_Start(&htim2);
					  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_UPDATE);
#if TRC_LA
					  sprintf(mUartBuffTx1, "CM4 : LAStateMachine start sampling high frequency...\n");
					  VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx1, strlen(mUartBuffTx1));
#endif
			    	  mMachineState = SAMPLING_HIGH_FREQ_BUFF1;
				  } else {
#if TRC_LA
					  sprintf(mUartBuffTx1, "CM4 : LAStateMachine starting HAL_DMA_Start_IT errorCode=%ld\n", htim2.hdma[TIM_DMA_ID_UPDATE]->ErrorCode);
					  VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx1, strlen(mUartBuffTx1));
#endif
					  // no machine state change
				  }
        	  } else {
            	  // samp. freq.  < 4MHz => compression & Data over TTY
        		  uint32_t period = mTIM2freq / m_samp_freq;
        		  configureTimer(period);
        		  mArrayDdrBuffIndex = 0;
        		  mRollingCompSampCount = 0;
        		  mSampCount = 0;
        		  mTotalCompSampCount = 0;
        		  mLastSamp = 0xFF;
				  mSramPacketSize = SAMP_SRAM_RPMSG_PACKET_SIZE;
        		  // RPMSG transfer size is limited to 496 due t0 buffer header, so we will perform transfer 256bytes at half and full DMA interrupt
				  HAL_StatusTypeDef res = HAL_DMA_Start_IT(htim2.hdma[TIM_DMA_ID_UPDATE], GPIOx_IDR, (uint32_t)&mSampBuff[0], mSramPacketSize);
        		  if (res == HAL_OK) {
        			  HAL_TIM_Base_Start(&htim2);
        			  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_UPDATE);
#if TRC_LA
        			  sprintf(mUartBuffTx1, "CM4 : LAStateMachine start sampling low frequency...\n");
					  VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx1, strlen(mUartBuffTx1));
#endif
        	    	  mMachineState = SAMPLING_LOW_FREQ_BUFF1;
        		  } else {
#if TRC_LA
        			  sprintf(mUartBuffTx1, "CM4 : LAStateMachine starting HAL_DMA_Start_IT errorCode=%ld\n", htim2.hdma[TIM_DMA_ID_UPDATE]->ErrorCode);
					  VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx1, strlen(mUartBuffTx1));
#endif
					  // no machine state change
        		  }
        	  }
          }
        }
    	break;
    case SAMPLING_LOW_FREQ_BUFF1:
        if (fDdrDma2Send0) {
            fDdrDma2Send0 = RESET;
            HAL_GPIO_WritePin(GPIOE, LA_0_Pin, GPIO_PIN_SET);
			uint32_t * pSource = (uint32_t *)mSampBuff;
			uint32_t * pDest = (uint32_t *)mSampBuffOut;
			for (int i=0; i<mSramPacketSize/8; i++) {
				*(pDest+i) = *(pSource+i) & 0x7F7F7F7F;
			}
			VIRT_UART_Transmit(&huart0, (uint8_t*)&mSampBuff[0], mSramPacketSize/2);
            HAL_GPIO_WritePin(GPIOE, LA_0_Pin, GPIO_PIN_RESET);
	    	mMachineState = SAMPLING_LOW_FREQ_BUFF2;
        }
        if (mDMAerror) {
        	mDMAerror = RESET;
			HAL_DMA_Abort_IT(htim2.hdma[TIM_DMA_ID_UPDATE]);
			HAL_TIM_Base_Stop(&htim2);
			mMachineState = DDR_BUFFERS_OK;
			sprintf(mUartBuffTx1, "CM4 : LAStateMachine SAMPLING_LOW_FREQ_BUFF1 mDMAerror !!!\n");
			VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx1, strlen(mUartBuffTx1));
        }
        if (VirtUart0RxMsg) {
          VirtUart0RxMsg = RESET;
          if (treatRxCommand2() == 'E') {
			if (fStopRequested) {
				fStopRequested = RESET;
				HAL_DMA_Abort_IT(htim2.hdma[TIM_DMA_ID_UPDATE]);
				HAL_TIM_Base_Stop(&htim2);
				mMachineState = DDR_BUFFERS_OK;
				sprintf(mUartBuffTx1, "CM4 : LAStateMachine SAMPLING_LOW_FREQ_BUFF1 fStopRequested\n");
				VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx1, strlen(mUartBuffTx1));
			}
          }
        }
    	break;
    case SAMPLING_LOW_FREQ_BUFF2:
        if (fDdrDma2Send1) {
            fDdrDma2Send1 = RESET;
            HAL_GPIO_WritePin(GPIOE, LA_0_Pin, GPIO_PIN_SET);
			uint32_t * pSource = (uint32_t *)&mSampBuff[mSramPacketSize/2];
			uint32_t * pDest = (uint32_t *)&mSampBuffOut[mSramPacketSize/2];
			for (int i=0; i<mSramPacketSize/2; i++) {
				*(pDest+i) = *(pSource+i) & 0x7F7F7F7F;
			}
			VIRT_UART_Transmit(&huart0, (uint8_t*)&mSampBuff[mSramPacketSize/2], mSramPacketSize/2);
            HAL_GPIO_WritePin(GPIOE, LA_0_Pin, GPIO_PIN_RESET);
	    	mMachineState = SAMPLING_LOW_FREQ_BUFF1;
        }
        if (mDMAerror) {
        	mDMAerror = RESET;
			HAL_DMA_Abort_IT(htim2.hdma[TIM_DMA_ID_UPDATE]);
			HAL_TIM_Base_Stop(&htim2);
			mMachineState = DDR_BUFFERS_OK;
			sprintf(mUartBuffTx1, "CM4 : LAStateMachine SAMPLING_LOW_FREQ_BUFF2 mDMAerror !!!\n");
			VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx1, strlen(mUartBuffTx1));
        }
        if (VirtUart0RxMsg) {
          VirtUart0RxMsg = RESET;
          if (treatRxCommand2() == 'E') {
			if (fStopRequested) {
				fStopRequested = RESET;
				HAL_DMA_Abort_IT(htim2.hdma[TIM_DMA_ID_UPDATE]);
				HAL_TIM_Base_Stop(&htim2);
				mMachineState = DDR_BUFFERS_OK;
				sprintf(mUartBuffTx1, "CM4 : LAStateMachine SAMPLING_LOW_FREQ_BUFF2 fStopRequested\n");
				VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx1, strlen(mUartBuffTx1));
			}
          }
        }
    	break;
    case SAMPLING_HIGH_FREQ_BUFF1:
    	// waiting end of DMA transfer
        if (fDdrDma2Send0) {
            fDdrDma2Send0 = RESET;
        	HAL_GPIO_WritePin(LA_TRC1_GPIO_Port, LA_TRC0_Pin, GPIO_PIN_SET);
			// mask and transfer first SRAM buff

        	uint32_t * pSource = (uint32_t *)&mSampBuff[0];
        	uint32_t * pDest = (uint32_t *)(mArrayDdrBuff[mArrayDdrBuffIndex].physAddr+mRollingCompSampCount);
        	// we have a buffer of mSramPacketSize/2 bytes, to send by packet of 4 => loop on mSramPacketSize/8
        	for (int i=0; i<mSramPacketSize/8; i++) {
        		*(pDest+i) = *(pSource+i) & 0x7F7F7F7F;		// 76µs to perform this operation, a memcpy takes 104µs
        	}
			mTotalCompSampCount += SAMP_SRAM_DDR_DMA_PACKET_SIZE/2;
			mRollingCompSampCount += SAMP_SRAM_DDR_DMA_PACKET_SIZE/2;
			if ((mTotalCompSampCount % SAMP_DDR_BUFFER_SIZE) == 0) {
				// time to change DDR buffer and send MSG to Linux
				sprintf(mSdbBuffTx, "B%dL%08x", mArrayDdrBuffIndex, SAMP_DDR_BUFFER_SIZE);
				RPMSG_HDR_Transmit(&hsdb0, (uint8_t*)mSdbBuffTx, strlen(mSdbBuffTx));
				mRollingCompSampCount = 0;
				mArrayDdrBuffIndex++;
				if (mArrayDdrBuffIndex == mArrayDdrBuffCount) {
					mArrayDdrBuffIndex = 0;
				}
			}
			HAL_GPIO_WritePin(LA_TRC1_GPIO_Port, LA_TRC0_Pin, GPIO_PIN_RESET);
	    	mMachineState = SAMPLING_HIGH_FREQ_BUFF2;
	    	mMachineState = SAMPLING_HIGH_FREQ_BUFF2;
        }
        if (mDMAerror) {
        	mDMAerror = RESET;
			HAL_DMA_Abort_IT(htim2.hdma[TIM_DMA_ID_UPDATE]);
			HAL_TIM_Base_Stop(&htim2);
			mMachineState = DDR_BUFFERS_OK;
			sprintf(mUartBuffTx1, "CM4 : LAStateMachine SAMPLING_HIGH_FREQ_BUFF1 mDMAerror !!!\n");
			VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx1, strlen(mUartBuffTx1));
        }
        if (VirtUart0RxMsg) {
          VirtUart0RxMsg = RESET;
          if (treatRxCommand2() == 'E') {
			if (fStopRequested) {
				fStopRequested = RESET;
				HAL_DMA_Abort_IT(htim2.hdma[TIM_DMA_ID_UPDATE]);
				HAL_TIM_Base_Stop(&htim2);
				mMachineState = DDR_BUFFERS_OK;
				sprintf(mUartBuffTx1, "CM4 : LAStateMachine SAMPLING_HIGH_FREQ_BUFF1 fStopRequested\n");
				VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx1, strlen(mUartBuffTx1));
			}
          }
        }
    	break;
    case SAMPLING_HIGH_FREQ_BUFF2:
        if (fDdrDma2Send1) {
            fDdrDma2Send1 = RESET;
        	HAL_GPIO_WritePin(LA_TRC1_GPIO_Port, LA_TRC0_Pin, GPIO_PIN_SET);
			// mask and transfer second SRAM buff
        	uint32_t * pSource = (uint32_t *)&mSampBuff[mSramPacketSize/2];
        	uint32_t * pDest = (uint32_t *)(mArrayDdrBuff[mArrayDdrBuffIndex].physAddr+mRollingCompSampCount);
        	// we have a buffer of mSramPacketSize/2 bytes, to send by packet of 4 => loop on mSramPacketSize/8
        	for (int i=0; i<mSramPacketSize/8; i++) {
        		*(pDest+i) = *(pSource+i) & 0x7F7F7F7F;		// 76µs to perform this operation, a memcpy takes 104µs
        	}
			mTotalCompSampCount += SAMP_SRAM_DDR_DMA_PACKET_SIZE/2;
			mRollingCompSampCount += SAMP_SRAM_DDR_DMA_PACKET_SIZE/2;
			if ((mTotalCompSampCount % SAMP_DDR_BUFFER_SIZE) == 0) {
				// time to change DDR buffer and send MSG to Linux
				sprintf(mSdbBuffTx, "B%dL%08x", mArrayDdrBuffIndex, SAMP_DDR_BUFFER_SIZE);
				RPMSG_HDR_Transmit(&hsdb0, (uint8_t*)mSdbBuffTx, strlen(mSdbBuffTx));
				mRollingCompSampCount = 0;
				mArrayDdrBuffIndex++;
				if (mArrayDdrBuffIndex == mArrayDdrBuffCount) {
					mArrayDdrBuffIndex = 0;
				}
			}
        	HAL_GPIO_WritePin(LA_TRC1_GPIO_Port, LA_TRC0_Pin, GPIO_PIN_RESET);
	    	mMachineState = SAMPLING_HIGH_FREQ_BUFF1;
        }
        if (mDMAerror) {
        	mDMAerror = RESET;
			HAL_DMA_Abort_IT(htim2.hdma[TIM_DMA_ID_UPDATE]);
			HAL_TIM_Base_Stop(&htim2);
			mMachineState = DDR_BUFFERS_OK;
			sprintf(mUartBuffTx1, "CM4 : LAStateMachine SAMPLING_HIGH_FREQ_BUFF2 mDMAerror !!!\n");
			VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx1, strlen(mUartBuffTx1));
        }
        if (VirtUart0RxMsg) {
          VirtUart0RxMsg = RESET;
          if (treatRxCommand2() == 'E') {
			if (fStopRequested) {
				fStopRequested = RESET;
				HAL_DMA_Abort_IT(htim2.hdma[TIM_DMA_ID_UPDATE]);
				HAL_TIM_Base_Stop(&htim2);
				mMachineState = DDR_BUFFERS_OK;
				sprintf(mUartBuffTx1, "CM4 : LAStateMachine SAMPLING_HIGH_FREQ_BUFF2 fStopRequested\n");
				VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx1, strlen(mUartBuffTx1));
			}
          }
        }
    	break;
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
  htim2.hdma[TIM_DMA_ID_UPDATE]->XferHalfCpltCallback = HalfTransferCompleteSRAM;
  htim2.hdma[TIM_DMA_ID_UPDATE]->XferCpltCallback = TransferCompleteSRAM;
  htim2.hdma[TIM_DMA_ID_UPDATE]->XferErrorCallback = TransferErrorSRAM;

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
   * Create Virtual UART devices
   */
  log_info("Virtual UART0 OpenAMP-rpmsg channel creation\r\n");
  if (VIRT_UART_Init(&huart0) != VIRT_UART_OK) {
    log_err("VIRT_UART_Init UART0 failed.\r\n");
    Error_Handler();
  }

  log_info("Virtual UART1 OpenAMP-rpmsg channel creation\r\n");
  if (VIRT_UART_Init(&huart1) != VIRT_UART_OK) {
    log_err("VIRT_UART_Init UART1 failed.\r\n");
    Error_Handler();
  }

  /*Need to register callback for message reception by channels*/
  if(RPMSG_HDR_RegisterCallback(&hsdb0, RPMSG_HDR_RXCPLT_CB_ID, SDB0_RxCpltCallback) != RPMSG_HDR_OK)
  {
    Error_Handler();
  }

  /*Need to register callback for message reception by channels (not for UART1 as used only for trace outpout) */
  if(VIRT_UART_RegisterCallback(&huart0, VIRT_UART_RXCPLT_CB_ID, VIRT_UART0_RxCpltCallback) != VIRT_UART_OK)
  {
    Error_Handler();
  }

  mMCUfreq = HAL_RCC_GetMCUFreq();
  mTIM2freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_TIMG1);

  // init buffers
  memset(&mSampBuffOut[0], 0x55, SAMP_SRAM_DDR_DMA_PACKET_SIZE/2);
  memset(&mSampBuffOut[SAMP_SRAM_DDR_DMA_PACKET_SIZE/2], 0x33, SAMP_SRAM_DDR_DMA_PACKET_SIZE/2);

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
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pins : LA_3_Pin LA_4_Pin LA_2_Pin LA_1_Pin
                           LA_0_Pin */
  GPIO_InitStruct.Pin = LA_3_Pin|LA_4_Pin|LA_2_Pin|LA_1_Pin
                          |LA_0_Pin|LA_TRC1_Pin|LA_TRC0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LA_TRC1_Pin|LA_TRC0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LA_TRC2_Pin */
  GPIO_InitStruct.Pin = LA_TRC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
#if 1
  sprintf(mUartBuffTx, "CM4 : OOOps: Error_Handler\n");
  VIRT_UART_Transmit(&huart1, (uint8_t*)mUartBuffTx, strlen(mUartBuffTx));
#endif
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
