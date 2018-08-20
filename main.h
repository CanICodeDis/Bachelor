/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_dac.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define Me_A_Pin LL_GPIO_PIN_0
#define Me_A_GPIO_Port GPIOC
#define Me_VS_Pin LL_GPIO_PIN_1
#define Me_VS_GPIO_Port GPIOC
#define Me_B_Pin LL_GPIO_PIN_2
#define Me_B_GPIO_Port GPIOC
#define Me_C_Pin LL_GPIO_PIN_3
#define Me_C_GPIO_Port GPIOC
#define LI_A_Pin LL_GPIO_PIN_0
#define LI_A_GPIO_Port GPIOA
#define LI_B_Pin LL_GPIO_PIN_1
#define LI_B_GPIO_Port GPIOA
#define LI_C_Pin LL_GPIO_PIN_2
#define LI_C_GPIO_Port GPIOA
#define BW_SEL_Pin LL_GPIO_PIN_3
#define BW_SEL_GPIO_Port GPIOA
#define CurrDirectA_Pin LL_GPIO_PIN_4
#define CurrDirectA_GPIO_Port GPIOC
#define ClockOut_Pin LL_GPIO_PIN_9
#define ClockOut_GPIO_Port GPIOC
#define BEMF_C_Pin LL_GPIO_PIN_8
#define BEMF_C_GPIO_Port GPIOA
#define BEMF_B_Pin LL_GPIO_PIN_9
#define BEMF_B_GPIO_Port GPIOA
#define BEMF_A_Pin LL_GPIO_PIN_10
#define BEMF_A_GPIO_Port GPIOA
#define StopButton_Pin LL_GPIO_PIN_11
#define StopButton_GPIO_Port GPIOC
#define StopButton_EXTI_IRQn EXTI15_10_IRQn
#define freigegeben_Pin LL_GPIO_PIN_4
#define freigegeben_GPIO_Port GPIOB
#define freigegeben_EXTI_IRQn EXTI4_IRQn
#define FreigabeMCU_Pin LL_GPIO_PIN_5
#define FreigabeMCU_GPIO_Port GPIOB
#define StopMCU_Pin LL_GPIO_PIN_6
#define StopMCU_GPIO_Port GPIOB
#define OverCurr_Pin LL_GPIO_PIN_7
#define OverCurr_GPIO_Port GPIOB
#define OverCurr_EXTI_IRQn EXTI9_5_IRQn
#define elekKomm_Pin LL_GPIO_PIN_8
#define elekKomm_GPIO_Port GPIOB
#define Hochlauf_Pin LL_GPIO_PIN_9
#define Hochlauf_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define SYSCLK (96e6)
#define H_BIT 0x0001
#define W_BIT 0x00000001

#define NB_ZCROSS 10
#define NB_RPM 10
#define NB_CAPVALUES 2
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
