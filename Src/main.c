/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h> /* memset */
#include <unistd.h> /* close */
#include "stm32f1xx_hal_uart.h"
#include <stdlib.h>     /* atoi */
#include "Buffers.h"
#include "UartCommand.h"
#include "CommandsManager.h"
#include "Ntp.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static char buf[10] = {0};
static IW18_PinState dots[9] = {OFF};
IW18_PinState uart_led = OFF;
static uint8_t index = 0;
static UartCommand* pCommands2 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void gpio_clr(GPIO_PinState state);
void printCharOnSegment(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, char num, IW18_PinState dot);
void setDateTime(char* pBuf);
void user_pwm_setvalue(uint16_t value);
extern void TIM2_callback();

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
extern void initialise_monitor_handles(void);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  //user_pwm_setvalue(100);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  //user_pwm_setvalue(300);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //initialise_monitor_handles();
  CM_init();
  Buffers_init();
  NTP_init(&hrtc);
  CM_SetUartHandle(&huart1);
  memset(dots,OFF,9);
  UartCommand test[5];
  pCommands2 = &test[0];
  //test[0] = UC_CreateUartCommand(GetCmdID(), NTP_ReserveBufferTxEnter, ReadEnter);
//  test[0] = UC_CreateUartCommand(GetCmdID(), NTP_ReserveBufferTx4Sync, ReadSync);
//  test[1] = UC_CreateUartCommand(GetCmdID(), NTP_ReserveBufferTx4Time, ReadTime);
//  test[2] = UC_CreateUartCommand(GetCmdID(), NTP_ReserveBufferTx4Hum, ReadHumi);

  UartCommand* cmd = &test[index];
  //HAL_StatusTypeDef def = HAL_UART_Transmit_IT(&huart1, (uint8_t*)(cmd->m_Tx.buf_cmd), cmd->m_Tx.buf_size);
  cmd = &test[1];
  RTC_TimeTypeDef sTime;

  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0x0;

  RTC_DateTypeDef DateToUpdate;
  HAL_RTC_GetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
  uint16_t pwm=0;




  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	 // HAL_Delay(500);
	  //HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	  //user_pwm_setvalue(pwm);
	  pwm=pwm+10;
	  if(pwm>1000) pwm=0;
	  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	  //HAL_Delay(1000);
	  CM_StateManager();
	  CM_ExecuteCommand();
	  GPIO_PinState state = HAL_GPIO_ReadPin(GPIOB,Geiger_CNT_Pin);
	  if(state == GPIO_PIN_SET)
	  {
		  HAL_Delay(100);
		  state = GPIO_PIN_RESET;
	  }

//  cmd = &test[1];
//  if(cmd->m_IsRecivedData)
//  {
//	  setDateTime(cmd->m_Rx.buf_cmd);
//	  cmd->m_IsRecivedData = 0;
//  }
//
//  cmd = &test[2];
//  if(cmd->m_IsRecivedData)
//  {
//	  //setHumidity(cmd->m_Rx.buf_cmd);
//	  cmd->m_IsRecivedData = 0;
//  }
//
//  if( HAL_OK== HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) )
//  {
//	  sprintf(&buf[0],"%02d-%02d-%02d", sTime.Hours, sTime.Minutes, sTime.Seconds );
//  }

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Enables the Clock Security System
    */
  HAL_RCC_EnableCSS();

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef DateToUpdate;

    /**Initialize RTC Only
    */
  hrtc.Instance = RTC;
  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F2){
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initialize RTC and set the Time and Date
    */
  sTime.Hours = 9;
  sTime.Minutes = 48;
  sTime.Seconds = 0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  DateToUpdate.WeekDay = RTC_WEEKDAY_SUNDAY;
  DateToUpdate.Month = RTC_MONTH_NOVEMBER;
  DateToUpdate.Date = 5;
  DateToUpdate.Year = 17;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR1,0x32F2);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 200;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 28500;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim2, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Seg7_GPIO_Port, Seg7_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SegB_Pin|SegF_Pin|SegA_Pin|Seg2_Pin
                          |Seg3_Pin|Seg4_Pin|Seg6_Pin|Seg8_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SegH_Pin|SegD_Pin|SegC_Pin|SegE_Pin
                          |buzzer_Pin|SegG_Pin|Seg5_Pin|Seg1_Pin
                          |Seg9_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ESP_RST_Pin|CH_PD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Seg7_Pin */
  GPIO_InitStruct.Pin = Seg7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Seg7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SegB_Pin SegF_Pin SegA_Pin Seg2_Pin
                           Seg3_Pin Seg4_Pin Seg6_Pin Seg8_Pin */
  GPIO_InitStruct.Pin = SegB_Pin|SegF_Pin|SegA_Pin|Seg2_Pin
                          |Seg3_Pin|Seg4_Pin|Seg6_Pin|Seg8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SegH_Pin SegD_Pin SegC_Pin SegE_Pin
                           buzzer_Pin SegG_Pin Seg5_Pin Seg1_Pin
                           Seg9_Pin */
  GPIO_InitStruct.Pin = SegH_Pin|SegD_Pin|SegC_Pin|SegE_Pin
                          |buzzer_Pin|SegG_Pin|Seg5_Pin|Seg1_Pin
                          |Seg9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Geiger_CNT_Pin */
  GPIO_InitStruct.Pin = Geiger_CNT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Geiger_CNT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ESP_RST_Pin CH_PD_Pin */
  GPIO_InitStruct.Pin = ESP_RST_Pin|CH_PD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void gpio_clr(GPIO_PinState state)
{
	  HAL_GPIO_WritePin(GPIOB, Seg1_Pin|Seg5_Pin|Seg9_Pin|SegC_Pin|SegD_Pin|SegE_Pin|SegG_Pin|SegH_Pin, state);
	  HAL_GPIO_WritePin(GPIOC, Seg7_Pin, state);
	  HAL_GPIO_WritePin(GPIOA, Seg2_Pin|Seg3_Pin|Seg4_Pin|Seg6_Pin|Seg8_Pin|SegA_Pin|SegB_Pin|SegF_Pin, state);
}
void user_pwm_setvalue(uint16_t value)
{
    TIM_OC_InitTypeDef sConfigOC;

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

//    while((TIM1->EGR & TIM_EGR_UG) == SET){}
//
     TIM1->EGR |= TIM_EGR_UG;
//     TIM1->BDTR |= TIM_BDTR_MOE;
//     TIM1->CCER |= TIM_CCER_CC1E;
//     TIM1->CR1 |= TIM_CR1_CEN;
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}
void printCharOnSegment(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, char num, IW18_PinState dot)
{

	gpio_clr(OFF);

	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, ON);
	// '-' = 45, //'0' = 48 ....'9' = 59
	if(num == '0'){

		  HAL_GPIO_WritePin(GPIOA,SegA_Pin,ON);
		  HAL_GPIO_WritePin(GPIOA,SegB_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegC_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegD_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegE_Pin,ON);
		  HAL_GPIO_WritePin(GPIOA,SegF_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegG_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOB,SegH_Pin,dot);
	}
	else if (num == '1') {
		  HAL_GPIO_WritePin(GPIOA,SegA_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOA,SegB_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegC_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegD_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOB,SegE_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOA,SegF_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOB,SegG_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOB,SegH_Pin,dot);
	}
	else if (num == '2') {
		  HAL_GPIO_WritePin(GPIOA,SegA_Pin,ON);
		  HAL_GPIO_WritePin(GPIOA,SegB_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegC_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOB,SegD_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegE_Pin,ON);
		  HAL_GPIO_WritePin(GPIOA,SegF_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOB,SegG_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegH_Pin,dot);
	}
	else if (num == '3') {
		  HAL_GPIO_WritePin(GPIOA,SegA_Pin,ON);
		  HAL_GPIO_WritePin(GPIOA,SegB_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegC_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegD_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegE_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOA,SegF_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOB,SegG_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegH_Pin,dot);
	}
	else if (num == '4') {
		  HAL_GPIO_WritePin(GPIOA,SegA_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOA,SegB_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegC_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegD_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOB,SegE_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOA,SegF_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegG_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegH_Pin,dot);
	}
	else if (num == '5') {
		  HAL_GPIO_WritePin(GPIOA,SegA_Pin,ON);
		  HAL_GPIO_WritePin(GPIOA,SegB_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOB,SegC_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegD_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegE_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOA,SegF_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegG_Pin,ON);
		  HAL_GPIO_WritePin(GPIOE,SegH_Pin,dot);
	}
	else if (num == '6') {
		  HAL_GPIO_WritePin(GPIOA,SegA_Pin,ON);
		  HAL_GPIO_WritePin(GPIOA,SegB_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOB,SegC_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegD_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegE_Pin,ON);
		  HAL_GPIO_WritePin(GPIOA,SegF_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegG_Pin,ON);
		  HAL_GPIO_WritePin(GPIOE,SegH_Pin,dot);
	}
	else if (num == '7') {
		  HAL_GPIO_WritePin(GPIOA,SegA_Pin,ON);
		  HAL_GPIO_WritePin(GPIOA,SegB_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegC_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegD_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOB,SegE_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOA,SegF_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegG_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOE,SegH_Pin,dot);
	}
	else if (num == '8') {
		  HAL_GPIO_WritePin(GPIOA,SegA_Pin,ON);
		  HAL_GPIO_WritePin(GPIOA,SegB_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegC_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegD_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegE_Pin,ON);
		  HAL_GPIO_WritePin(GPIOA,SegF_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegG_Pin,ON);
		  HAL_GPIO_WritePin(GPIOE,SegH_Pin,dot);
	}
	else if (num == '9') {
		  HAL_GPIO_WritePin(GPIOA,SegA_Pin,ON);
		  HAL_GPIO_WritePin(GPIOA,SegB_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegC_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegD_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegE_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOA,SegF_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegG_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegH_Pin,dot);
	}
	else if (num == '-') {
		  HAL_GPIO_WritePin(GPIOA,SegA_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOA,SegB_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOB,SegC_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOB,SegD_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOB,SegE_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOA,SegF_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOB,SegG_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegH_Pin,dot);
	}
	else if (num == '.') {
		  HAL_GPIO_WritePin(GPIOA,SegA_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOA,SegB_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOB,SegC_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOB,SegD_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOB,SegE_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOA,SegF_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOB,SegG_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOB,SegH_Pin,ON);
	}
	else if(num == 'h')
	{
		  HAL_GPIO_WritePin(GPIOA,SegA_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOA,SegB_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOB,SegC_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegD_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOB,SegE_Pin,ON);
		  HAL_GPIO_WritePin(GPIOA,SegF_Pin,ON);
		  HAL_GPIO_WritePin(GPIOB,SegG_Pin,ON);
	}
	else{
		  HAL_GPIO_WritePin(GPIOA,SegA_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOA,SegB_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOB,SegC_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOB,SegD_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOB,SegE_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOA,SegF_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOB,SegG_Pin,OFF);
		  HAL_GPIO_WritePin(GPIOB,SegH_Pin,dot);
	}
	//HAL_Delay(1);

}

void setDateTime(char* pBuf)
{
	RTC_DateTypeDef DateToUpdate;
	RTC_TimeTypeDef TimeToUpdate;

	char tempBuf[4];
	memset(&tempBuf, 0 , 4);
	memcpy(&tempBuf, &pBuf[5], 2);
	DateToUpdate.Year = atoi(tempBuf);

	memset(&tempBuf, 0 , 4);
	memcpy(&tempBuf, &pBuf[8], 2);
	DateToUpdate.Month = atoi(tempBuf);

	memset(&tempBuf, 0 , 4);
	memcpy(&tempBuf, &pBuf[11], 2);
	DateToUpdate.Date = atoi(tempBuf);

	memset(&tempBuf, 0 , 4);
	memcpy(&tempBuf, &pBuf[14], 2);
	TimeToUpdate.Hours = atoi(tempBuf);

	memset(&tempBuf, 0 , 4);
	memcpy(&tempBuf, &pBuf[17], 2);
	TimeToUpdate.Minutes = atoi(tempBuf);

	memset(&tempBuf, 0 , 4);
	memcpy(&tempBuf, &pBuf[20], 2);
	TimeToUpdate.Seconds = atoi(tempBuf);

	if (HAL_RTC_SetTime(&hrtc, &TimeToUpdate, RTC_FORMAT_BIN) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR1,0x32F2);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	  UartCommand* pCmd =  CM_FindUartCommand(ReadCmdID() - 1);//UC_FindUartCommand(CM_GetCommands(), CM_GetCommandsCount(), ReadCmdID() - 1);
	  pCmd->m_IsTransmitedData = 1;
	  if(pCmd==0) return;

	  HAL_UART_Receive_IT(huart, pCmd->m_Rx.buf_cmd, pCmd->m_Rx.buf_size);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	  uint8_t *pRxBuffPtrTest = huart->pRxBuffPtr - huart->RxXferSize;
	  char buf[4] = {0, 0, 0, 0};
	  memcpy(&buf[0],pRxBuffPtrTest, 3);
	  uint8_t cmd_id = atoi(&buf[0]);
	  if(buf[0] == '>')
	  {
		  memcpy(&buf[0],pRxBuffPtrTest+2, 3);
		  cmd_id = atoi(&buf[0]);
	  }
	  UartCommand* pCmd = CM_FindUartCommand(cmd_id);
	  pCmd->m_IsRecivedData = 1;
}

void TIM2_callback()
{
	char* disp = IW18_BUF();
	static uint8_t segment = 0;

		if(segment == 1){
			printCharOnSegment(GPIOB,Seg1_Pin,0, dots[0]);//0 = 48
		}
		else if(segment == 2){
			printCharOnSegment(GPIOA,Seg2_Pin, disp[0], dots[1]);
		}
		else if(segment == 3){
			printCharOnSegment(GPIOA,Seg3_Pin, disp[1], dots[2]); // '-' = 45, '9' = 59
		}
		else if(segment == 4){
			printCharOnSegment(GPIOA,Seg4_Pin, disp[2], dots[3]);
		}
		else if(segment == 5){
			printCharOnSegment(GPIOB,Seg5_Pin, disp[3], dots[4]);
		}
		else if(segment == 6){
			printCharOnSegment(GPIOA,Seg6_Pin, disp[4], dots[5]);
		}
		else if(segment == 7){
			printCharOnSegment(GPIOC,Seg7_Pin, disp[5], dots[6]);
		}
		else if(segment == 8){
			printCharOnSegment(GPIOA,Seg8_Pin, disp[6], dots[7]);
		}
		else if(segment == 9){
			printCharOnSegment(GPIOB,Seg9_Pin, disp[7], dots[8]);
		}
		if(segment++>8) segment=0;

	  //AS_RDA5807M_ReadRegisters();
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
