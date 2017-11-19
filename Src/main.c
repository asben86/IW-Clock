/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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

#include "UartCommand.h"
#include "Ntp.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static char buf[10] = {0};
static IW18_PinState dots[9] = {OFF};
uint8_t Received[100];
uint8_t recvBytes = 0;
static uint8_t RxBuf[100];
uint16_t RxBufIndex = 0;
static uint8_t aTxEndMessage[20] = {0};
IW18_PinState uart_led = OFF;
char ntp_key[] = "NTP CLIENT OK! ";
static uint8_t index = 0;
static UartCommand* pCommands = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void gpio_clr(GPIO_PinState state);
void printCharOnSegment(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, char num, IW18_PinState dot);
void setDateTime(char* pBuf);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
extern void initialise_monitor_handles(void);
/* USER CODE END 0 */

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

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //HAL_Delay(5000);
  //initialise_monitor_handles();
  memset(dots,OFF,9);
  memset(buf,0,20);
  memset(RxBuf,0,100);
  memset(aTxEndMessage,0,20);
  UartCommand test[5];
  pCommands = &test[0];
  test[0] = UC_CreateUartCommand(GetCmdID(), GetSync, ReadSync);
  test[1] = UC_CreateUartCommand(GetCmdID(), GetTime, ReadTime);

  UartCommand* cmd = &test[index];
  HAL_StatusTypeDef def = HAL_UART_Transmit_IT(&huart1, (uint8_t*)(cmd->m_Tx.buf_cmd), cmd->m_Tx.buf_size);
  cmd = &test[1];
  RTC_TimeTypeDef sTime;

  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0x0;

  RTC_DateTypeDef DateToUpdate;
  HAL_RTC_GetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);

  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  if(cmd->m_IsRecivedData)
  {
	  setDateTime(cmd->m_Rx.buf_cmd);
	  cmd->m_IsRecivedData = 0;
  }

  memset(&buf[0], 0, 10);
  if( HAL_OK== HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) )
  {

	  sprintf(&buf[0],"%02d-%02d-%02d", sTime.Hours, sTime.Minutes, sTime.Seconds );
  }
  uint8_t ms = 1;
  dots[4] = dots[6] = OFF;
  //HAL_Delay(1);
  HAL_Delay(ms);
  printCharOnSegment(GPIOB,Seg1_Pin,'.', uart_led);//0 = 48

  HAL_Delay(ms);
  printCharOnSegment(GPIOB,Seg1_Pin,0, dots[0]);//0 = 48
  HAL_Delay(ms);
  printCharOnSegment(GPIOA,Seg2_Pin,buf[0], dots[1]);
  HAL_Delay(ms);
  printCharOnSegment(GPIOA,Seg3_Pin,buf[1], dots[2]); // '-' = 45, '9' = 59
  HAL_Delay(ms);
  printCharOnSegment(GPIOA,Seg4_Pin,buf[2], dots[3]);
  HAL_Delay(ms);
  printCharOnSegment(GPIOB,Seg5_Pin,buf[3], dots[4]);
  HAL_Delay(ms);
  printCharOnSegment(GPIOA,Seg6_Pin,buf[4], dots[5]);
  HAL_Delay(ms);
  printCharOnSegment(GPIOC,Seg7_Pin,buf[5], dots[6]);
  HAL_Delay(ms);
  printCharOnSegment(GPIOA,Seg8_Pin,buf[6], dots[7]);
  HAL_Delay(ms);
  printCharOnSegment(GPIOB,Seg9_Pin,buf[7], dots[8]);

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
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
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initialize RTC and set the Time and Date
    */
  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F2){
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
                          |SegG_Pin|Seg5_Pin|Seg1_Pin|Seg9_Pin, GPIO_PIN_SET);

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
                           SegG_Pin Seg5_Pin Seg1_Pin Seg9_Pin */
  GPIO_InitStruct.Pin = SegH_Pin|SegD_Pin|SegC_Pin|SegE_Pin
                          |SegG_Pin|Seg5_Pin|Seg1_Pin|Seg9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void gpio_clr(GPIO_PinState state)
{
	  HAL_GPIO_WritePin(GPIOB, Seg1_Pin|Seg5_Pin|Seg9_Pin|SegC_Pin|SegD_Pin|SegE_Pin|SegG_Pin|SegH_Pin, state);
	  HAL_GPIO_WritePin(GPIOC, Seg7_Pin, state);
	  HAL_GPIO_WritePin(GPIOA, Seg2_Pin|Seg3_Pin|Seg4_Pin|Seg6_Pin|Seg8_Pin|SegA_Pin|SegB_Pin|SegF_Pin, state);
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
	  UartCommand* pCmd = UC_FindUartCommand(pCommands, 4, index);
	  if(pCmd==0) return;
	  HAL_UART_Receive_IT(huart, pCmd->m_Rx.buf_cmd,  pCmd->m_Rx.buf_size);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	  uint8_t                       *pRxBuffPtrTest = huart->pRxBuffPtr - huart->RxXferSize;
	  char buf[4];
	  memcpy(&buf[0],pRxBuffPtrTest, 3);
	  buf[4] = 0;
	  uint8_t cmd_id = atoi(&buf[0]);
	  UartCommand* pCmd = UC_FindUartCommand(pCommands, 4, cmd_id);
	  pCmd->m_IsRecivedData = 1;
	  if(pCmd==0) return;

	  //NTP_Execute(pCmd->m_pFuncRecive, &pCmd->m_Rx);
	  pCmd = UC_FindUartCommand(pCommands, 4, ++index);
	  if(pCmd==0) return;

	  HAL_StatusTypeDef def = HAL_UART_Transmit_IT(huart, (uint8_t*)(pCmd->m_Tx.buf_cmd), pCmd->m_Tx.buf_size);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
