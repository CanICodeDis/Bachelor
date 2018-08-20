
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include"ZeroCrossTimings.h"
#include"RPMList.h"
#include"PWMFrequency.h"
#include"Commutation.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint32_t HighsideFrequency;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void chargeBootsraps(uint16_t);
static void setHighsidePWMFreq(enum PWMFreq aFreqency);
static void InitializeLists();
static void DefineComm();
static void enableDMA2(uint32_t aStream);
static void changeTIM1DMADestination(uint32_t stream);
static void PWMStart();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
struct ZeroCrossing ZeroC[NB_ZCROSS];
struct ZeroCrossing *pZeroCrossCurrent;
struct RPMList RPM[NB_RPM];
struct RPMList *pRPMCurrent;
struct commData Kommutierung[6];
struct commData *pKomm;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
 InitializeLists();
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  chargeBootsraps(50);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_3);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_3)
  {
  Error_Handler();  
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
    
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_12, 192, LL_RCC_PLLP_DIV_4);

  LL_RCC_PLL_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_12, 192, LL_RCC_PLLQ_DIV_8);

  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);

  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_Init1msTick(96000000);

  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);

  LL_SetSystemCoreClock(96000000);

  LL_RCC_ConfigMCO(LL_RCC_MCO2SOURCE_SYSCLK, LL_RCC_MCO2_DIV_1);

  LL_RCC_SetUSBClockSource(LL_RCC_USB_CLKSOURCE_PLL);

  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  LL_ADC_InitTypeDef ADC_InitStruct;
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct;
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct;

  LL_GPIO_InitTypeDef GPIO_InitStruct;

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
  
  /**ADC1 GPIO Configuration  
  PC0   ------> ADC1_IN10
  PC1   ------> ADC1_IN11
  PC2   ------> ADC1_IN12
  PC3   ------> ADC1_IN13
  PC4   ------> ADC1_IN14 
  */
  GPIO_InitStruct.Pin = Me_A_Pin|Me_VS_Pin|Me_B_Pin|Me_C_Pin 
                          |CurrDirectA_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* ADC1 DMA Init */
  
  /* ADC1 Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_0, LL_DMA_CHANNEL_0);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_0, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_0, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MEMORY_NOINCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_0);

    /**Common config 
    */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);

  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);

  LL_ADC_REG_SetFlagEndOfConversion(ADC1, LL_ADC_REG_FLAG_EOC_UNITARY_CONV);

  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);

    /**Configure Regular Channel 
    */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_10);

  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_10, LL_ADC_SAMPLINGTIME_3CYCLES);

}

/* DAC init function */
static void MX_DAC_Init(void)
{

  LL_DAC_InitTypeDef DAC_InitStruct;

  LL_GPIO_InitTypeDef GPIO_InitStruct;

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DAC1);
  
  /**DAC GPIO Configuration  
  PA4   ------> DAC_OUT1
  PA5   ------> DAC_OUT2 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4|LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* DAC DMA Init */
  
  /* DAC1 Init */
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_5, LL_DMA_CHANNEL_7);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_5, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_5, LL_DMA_PRIORITY_MEDIUM);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MEMORY_NOINCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_5, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_5);

  /* DAC2 Init */
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_6, LL_DMA_CHANNEL_7);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_6, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_6, LL_DMA_PRIORITY_MEDIUM);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_6, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MEMORY_NOINCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_6, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_6, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_6);

    /**DAC channel OUT1 config 
    */
  DAC_InitStruct.TriggerSource = LL_DAC_TRIG_SOFTWARE;
  DAC_InitStruct.WaveAutoGeneration = LL_DAC_WAVE_AUTO_GENERATION_NONE;
  DAC_InitStruct.OutputBuffer = LL_DAC_OUTPUT_BUFFER_ENABLE;
  LL_DAC_Init(DAC, LL_DAC_CHANNEL_1, &DAC_InitStruct);

    /**DAC channel OUT2 config 
    */
  LL_DAC_Init(DAC, LL_DAC_CHANNEL_2, &DAC_InitStruct);

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  LL_TIM_InitTypeDef TIM_InitStruct;

  LL_GPIO_InitTypeDef GPIO_InitStruct;

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
  
  /**TIM1 GPIO Configuration  
  PA8   ------> TIM1_CH1
  PA9   ------> TIM1_CH2
  PA10   ------> TIM1_CH3 
  */
  GPIO_InitStruct.Pin = BEMF_C_Pin|BEMF_B_Pin|BEMF_A_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* TIM1 DMA Init */
  
  /* TIM1_CH1 Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_1, LL_DMA_CHANNEL_6);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_1, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_1, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_1, LL_DMA_MEMORY_NOINCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_1, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_1, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_1);

  /* TIM1_CH2 Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_2, LL_DMA_CHANNEL_6);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_2, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MEMORY_NOINCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_2, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_2, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_2);

  /* TIM1_CH3 Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_6, LL_DMA_CHANNEL_6);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_6, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_6, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_6, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_6, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_6, LL_DMA_MEMORY_NOINCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_6, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_6, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_6);

  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 0;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);

  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);

  LL_TIM_DisableMasterSlaveMode(TIM1);

  LL_TIM_IC_SetActiveInput(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);

  LL_TIM_IC_SetPrescaler(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);

  LL_TIM_IC_SetFilter(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV32_N8);

  LL_TIM_IC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);

  LL_TIM_IC_SetActiveInput(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);

  LL_TIM_IC_SetPrescaler(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);

  LL_TIM_IC_SetFilter(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV32_N8);

  LL_TIM_IC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);

  LL_TIM_IC_SetActiveInput(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_ACTIVEINPUT_DIRECTTI);

  LL_TIM_IC_SetPrescaler(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_ICPSC_DIV1);

  LL_TIM_IC_SetFilter(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_IC_FILTER_FDIV32_N8);

  LL_TIM_IC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_IC_POLARITY_RISING);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  LL_TIM_InitTypeDef TIM_InitStruct;
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct;

  LL_GPIO_InitTypeDef GPIO_InitStruct;

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 0;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);

  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH1);

  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);

  LL_TIM_OC_EnableFast(TIM3, LL_TIM_CHANNEL_CH1);

  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH2);

  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);

  LL_TIM_OC_EnableFast(TIM3, LL_TIM_CHANNEL_CH2);

  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH3);

  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);

  LL_TIM_OC_EnableFast(TIM3, LL_TIM_CHANNEL_CH3);

  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);

  LL_TIM_DisableMasterSlaveMode(TIM3);

  /**TIM3 GPIO Configuration  
  PA6   ------> TIM3_CH1
  PA7   ------> TIM3_CH2
  PB0   ------> TIM3_CH3 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* TIM8 init function */
static void MX_TIM8_Init(void)
{

  LL_TIM_InitTypeDef TIM_InitStruct;
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct;
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct;

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);

  /* TIM8 DMA Init */
  
  /* TIM8_CH2 Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_3, LL_DMA_CHANNEL_7);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_3, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_3, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_3, LL_DMA_MEMORY_NOINCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_3, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_3, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_3);

  /* TIM8_CH3 Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_4, LL_DMA_CHANNEL_7);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_4, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_4, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_4, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_4, LL_DMA_MEMORY_NOINCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_4, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_4, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_4);

  /* TIM8_CH4_TRIG_COM Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_7, LL_DMA_CHANNEL_7);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_7, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_7, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_7, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_7, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_7, LL_DMA_MEMORY_NOINCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_7, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_7, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_7);

  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 0;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM8, &TIM_InitStruct);

  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_FROZEN;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM8, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);

  LL_TIM_OC_DisableFast(TIM8, LL_TIM_CHANNEL_CH2);

  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  LL_TIM_OC_Init(TIM8, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);

  LL_TIM_OC_DisableFast(TIM8, LL_TIM_CHANNEL_CH3);

  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  LL_TIM_OC_Init(TIM8, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);

  LL_TIM_OC_DisableFast(TIM8, LL_TIM_CHANNEL_CH4);

  LL_TIM_SetTriggerOutput(TIM8, LL_TIM_TRGO_RESET);

  LL_TIM_DisableMasterSlaveMode(TIM8);

  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 0;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
  LL_TIM_BDTR_Init(TIM8, &TIM_BDTRInitStruct);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream4_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
     PC9   ------> RCC_MCO_2
*/
static void MX_GPIO_Init(void)
{

  LL_GPIO_InitTypeDef GPIO_InitStruct;
  LL_EXTI_InitTypeDef EXTI_InitStruct;

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LI_A_Pin|LI_B_Pin|LI_C_Pin|BW_SEL_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, FreigabeMCU_Pin|StopMCU_Pin|elekKomm_Pin|Hochlauf_Pin);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13|LL_GPIO_PIN_14|LL_GPIO_PIN_15|LL_GPIO_PIN_5 
                          |LL_GPIO_PIN_6|LL_GPIO_PIN_7|LL_GPIO_PIN_8|LL_GPIO_PIN_10 
                          |LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LI_A_Pin|LI_B_Pin|LI_C_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BW_SEL_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BW_SEL_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_10|LL_GPIO_PIN_12 
                          |LL_GPIO_PIN_13|LL_GPIO_PIN_14|LL_GPIO_PIN_15|LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = ClockOut_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(ClockOut_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = FreigabeMCU_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(FreigabeMCU_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = StopMCU_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(StopMCU_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = elekKomm_Pin|Hochlauf_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE11);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE4);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE7);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_11;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_4;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_7;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinPull(StopButton_GPIO_Port, StopButton_Pin, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinPull(freigegeben_GPIO_Port, freigegeben_Pin, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinPull(OverCurr_GPIO_Port, OverCurr_Pin, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinMode(StopButton_GPIO_Port, StopButton_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(freigegeben_GPIO_Port, freigegeben_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(OverCurr_GPIO_Port, OverCurr_Pin, LL_GPIO_MODE_INPUT);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(EXTI4_IRQn);
  NVIC_SetPriority(EXTI9_5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(EXTI9_5_IRQn);
  NVIC_SetPriority(EXTI15_10_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

static void chargeBootsraps(uint16_t aMicro){
/*
 * This function loads the gate-drivers' bootstrap-capacitors for
 * the time specified as argument. It is achieved by switching on
 * the low-side MOSFETS for the specified time via TIMER 4 in
 * One-Pulse Mode. After the specified time, Lowside-Mosfets are switched off.
 *
 * @ param aMicro: time (in µs) to switch on low-side-Mosfets
 * */
	//Enable Timer4 Clock
	SET_BIT(RCC->APB1ENR,RCC_APB1ENR_TIM14EN);
	// Configure Timer 4
	// Set ONe-Pulse Mode
	TIM4->CR1 |= (H_BIT<<3);
	// Prescaler auf 96 , deshalb Zählerfrequenz 1MHz
	TIM4->PSC = 96;

	//Zählt bis zu gewünschter Zahl
	TIM4->ARR = aMicro;

	TIM4->CR1 |= H_BIT; //startet Timer
	HAL_GPIO_WritePin(LI_A_GPIO_Port,LI_A_Pin | LI_B_Pin |LI_C_Pin,GPIO_PIN_SET);

	while (!(TIM4->SR & H_BIT)); //Zeit abwarten

	HAL_GPIO_WritePin(LI_A_GPIO_Port,LI_A_Pin | LI_B_Pin |LI_C_Pin,GPIO_PIN_RESET);
	__TIM4_CLK_DISABLE();
}

static void setHighsidePWMFreq(enum PWMFreq aFreqency){
	float currDutycycle;
	uint16_t ARR_Content,CCR_Content,newCCR;

	ARR_Content = TIM3->ARR;
	CCR_Content = TIM3->CCR1;
	currDutycycle = (float) CCR_Content/ARR_Content;
	newCCR= ARRValue[aFreqency] * currDutycycle;

	//disable Udpate-event
	TIM3->CR1 |= (H_BIT<<1);

	//write new duty-cycle into CCR
	TIM3->CCR1 = (uint32_t) newCCR;
	TIM3->CCR2 = (uint32_t) newCCR;
	TIM3->CCR3 = (uint32_t) newCCR;
	//write new Frequency into ARR
	TIM3->ARR = ARRValue[aFreqency];
	//enable Update-event
	TIM3->CR1 &= ~(H_BIT<<1);
	}

static void PWMStart(){
	TIM3->CR1 |= (H_BIT<<7);
	TIM3->CR1 |= H_BIT;
}


static void setTIM1DMASource(){
	//Diese Funktion legt fest, dass der DMA-Controller
	//aus den Capture-Registern von Timer 1 liest
	LL_DMA_SetPeriphAddress(DMA2,LL_DMA_STREAM_1,TIM1->CCR1);
	LL_DMA_SetPeriphAddress(DMA2,LL_DMA_STREAM_2,TIM1->CCR2);
	LL_DMA_SetPeriphAddress(DMA2,LL_DMA_STREAM_6,TIM1->CCR3);
}

static void enableDMA2(uint32_t aStream){
	//Diese Funktion gibt den DMA auf einem Stream frei
	LL_DMA_EnableStream(DMA2,aStream);
}
static void changeTIM1DMADestination(uint32_t stream){
//DMA2 deaktivieren
	LL_DMA_DisableStream(DMA2,stream);
//DMA sollen immer auf das aktuelle Element des Zero-Cross
//übertragen
LL_DMA_SetMemoryAddress(DMA2,stream,&(pZeroCrossCurrent->count));
// DMA2 aktivieren
LL_DMA_EnableStream(DMA2,stream);
}

static void DefineComm(){
uint8_t i;
for (i=0;i<6;i++)
	{
	if ( i == 0 )
		Kommutierung[i].prev = &Kommutierung[5];
	else
		Kommutierung[i].prev = &Kommutierung[i-1];
	if ( i == 5 )
		Kommutierung[i].next = &Kommutierung[0];
	else
		Kommutierung[i].next = &Kommutierung[i+1];
	}

//Elektrischer Abschnitt1
Kommutierung[0].LIA=0;
Kommutierung[0].LIB=0;
Kommutierung[0].LIC=1;
Kommutierung[0].OCRA=1;
Kommutierung[0].OCRB=0;
Kommutierung[0].OCRC=0;
Kommutierung[0].stream = LL_DMA_STREAM_2;
Kommutierung[0].capturePolarity=1;
Kommutierung[0].CapA=0;
Kommutierung[0].CapB=1;
Kommutierung[0].CapC=0;

//Elektrischer Abschnitt2
Kommutierung[1].LIA=0;
Kommutierung[1].LIB=0;
Kommutierung[1].LIC=1;
Kommutierung[1].OCRA=0;
Kommutierung[1].OCRB=1;
Kommutierung[1].OCRC=0;
Kommutierung[1].stream= LL_DMA_STREAM_1;
Kommutierung[1].capturePolarity=0;
Kommutierung[1].CapA=1;
Kommutierung[1].CapB=0;
Kommutierung[1].CapC=0;

//Elektrischer Abschnitt3
Kommutierung[2].LIA=1;
Kommutierung[2].LIB=0;
Kommutierung[2].LIC=0;
Kommutierung[2].OCRA=0;
Kommutierung[2].OCRB=1;
Kommutierung[2].OCRC=0;
Kommutierung[2].stream= LL_DMA_STREAM_6;
Kommutierung[2].capturePolarity=1;
Kommutierung[2].CapA=0;
Kommutierung[2].CapB=0;
Kommutierung[2].CapC=1;

//Elektrischer Abschnitt4
Kommutierung[3].LIA=1;
Kommutierung[3].LIB=0;
Kommutierung[3].LIC=0;
Kommutierung[3].OCRA=0;
Kommutierung[3].OCRB=0;
Kommutierung[3].OCRC=1;
Kommutierung[3].stream = LL_DMA_STREAM_2;
Kommutierung[3].capturePolarity=0;
Kommutierung[3].CapA=0;
Kommutierung[3].CapB=1;
Kommutierung[3].CapC=0;

//Elektrischer Abschnitt5
Kommutierung[4].LIA=0;
Kommutierung[4].LIB=1;
Kommutierung[4].LIC=0;
Kommutierung[4].OCRA=0;
Kommutierung[4].OCRB=0;
Kommutierung[4].OCRC=1;
Kommutierung[4].stream = LL_DMA_STREAM_2;
Kommutierung[4].capturePolarity=1;
Kommutierung[4].CapA=1;
Kommutierung[4].CapB=0;
Kommutierung[4].CapC=0;

//Elektrischer Abschnitt6
Kommutierung[5].LIA=0;
Kommutierung[5].LIB=1;
Kommutierung[5].LIC=0;
Kommutierung[5].OCRA=1;
Kommutierung[5].OCRB=0;
Kommutierung[5].OCRC=0;
Kommutierung[5].stream= LL_DMA_STREAM_6;
Kommutierung[5].capturePolarity=0;
Kommutierung[5].CapA=0;
Kommutierung[5].CapB=0;
Kommutierung[5].CapC=1;

pKomm = &Kommutierung[0];
}

static void InitializeLists()
{
uint8_t i;
ZeroC[0].prev = &ZeroC[NB_ZCROSS];
for (i=1; i<NB_ZCROSS;i++)
	{
	ZeroC[i].prev = &ZeroC[i-1];
	if (i == (NB_ZCROSS-1))
		ZeroC[i].next = &ZeroC[0];
	else
		ZeroC[i].next = &ZeroC[i+1];
	}
pZeroCrossCurrent = &ZeroC[0];

RPM[0].prev = &RPM[NB_RPM];
for(i=1;i<NB_RPM;i++)
	{
	RPM[i].prev = &RPM[i-1];
		if (i == (NB_RPM-1))
			RPM[i].next = &RPM[0];
		else
			RPM[i].next = &RPM[i+1];
	}
pRPMCurrent = &RPM[0];
DefineComm();
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
