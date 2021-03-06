/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define Seg7_Pin GPIO_PIN_13
#define Seg7_GPIO_Port GPIOC
#define SegB_Pin GPIO_PIN_0
#define SegB_GPIO_Port GPIOA
#define SegF_Pin GPIO_PIN_1
#define SegF_GPIO_Port GPIOA
#define SegA_Pin GPIO_PIN_2
#define SegA_GPIO_Port GPIOA
#define Seg2_Pin GPIO_PIN_3
#define Seg2_GPIO_Port GPIOA
#define Seg3_Pin GPIO_PIN_4
#define Seg3_GPIO_Port GPIOA
#define Seg4_Pin GPIO_PIN_5
#define Seg4_GPIO_Port GPIOA
#define Seg6_Pin GPIO_PIN_6
#define Seg6_GPIO_Port GPIOA
#define SegH_Pin GPIO_PIN_0
#define SegH_GPIO_Port GPIOB
#define SegD_Pin GPIO_PIN_1
#define SegD_GPIO_Port GPIOB
#define SegC_Pin GPIO_PIN_10
#define SegC_GPIO_Port GPIOB
#define SegE_Pin GPIO_PIN_11
#define SegE_GPIO_Port GPIOB
#define buzzer_Pin GPIO_PIN_12
#define buzzer_GPIO_Port GPIOB
#define Geiger_CNT_Pin GPIO_PIN_15
#define Geiger_CNT_GPIO_Port GPIOB
#define PWM_GEIGER_Pin GPIO_PIN_8
#define PWM_GEIGER_GPIO_Port GPIOA
#define Seg8_Pin GPIO_PIN_11
#define Seg8_GPIO_Port GPIOA
#define ESP_RST_Pin GPIO_PIN_12
#define ESP_RST_GPIO_Port GPIOA
#define CH_PD_Pin GPIO_PIN_15
#define CH_PD_GPIO_Port GPIOA
#define SegG_Pin GPIO_PIN_3
#define SegG_GPIO_Port GPIOB
#define Seg5_Pin GPIO_PIN_4
#define Seg5_GPIO_Port GPIOB
#define PWM_CLOCK_Pin GPIO_PIN_5
#define PWM_CLOCK_GPIO_Port GPIOB
#define Seg1_Pin GPIO_PIN_8
#define Seg1_GPIO_Port GPIOB
#define Seg9_Pin GPIO_PIN_9
#define Seg9_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

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
