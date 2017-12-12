/*
 * Ntp.c
 *
 *  Created on: 14.11.2017
 *      Author: ben
 */

#include "Ntp.h"
#include <string.h> 	/* memset */
#include <stdlib.h>     /* atoi */
#include "Buffers.h"
#include <stdlib.h>     /* malloc, free, rand */
#include "CommandsManager.h"
static const uint8_t CONST_PREFIX = 2 + 3 + 2; // "> 000....\r\n"

void  NTP_init(RTC_HandleTypeDef* pHrtc)
{
	pRtc = pHrtc;
	pTx = &Tx[0];
	pRx = &Rx[0];
	memset(pTx, 0, BUF_SIZE);
	memset(pRx, 0, BUF_SIZE);

	//static char* pEnter[2] = {0x0d, 0x0A}; //\r\n
	//static char* pEnterOdp[2] = {0x00, 0x00}; //\r\n

}
Buffer ReadTime()
{
	Buffer x;
	x.buf_cmd = UART_RX_BUF();
	x.buf_size = 19 + CONST_PREFIX;
	pRx+=x.buf_size;
	return x;
}


Buffer ReadSync()
{
	Buffer x;
	x.buf_size = 5 + CONST_PREFIX;
	x.buf_cmd = (char*) malloc (x.buf_size);
	return x;
}
Buffer ReadEnter()
{
	Buffer x;
	x.buf_cmd = UART_RX_BUF();
	x.buf_size = 2;
	return x;
}

Buffer ReadHumi()
{
	Buffer x;
	x.buf_size = 30 + CONST_PREFIX;
	x.buf_cmd = (char*) malloc (x.buf_size);
	return x;
	//003DHT Temperature:24;Humidity:95
}


Buffer NTP_ReserveBufferTx4Time(uint8_t cmd_id)
{
	Buffer x;
	x.buf_cmd = (char*) malloc (20);
	x.buf_size = sprintf(x.buf_cmd,"ntp.GetTime(%03d)\r\n", cmd_id);
	return x;
}

Buffer NTP_ReserveBufferTx4Sync(uint8_t cmd_id)
{
	Buffer x;
	x.buf_cmd = (char*) malloc (20);
	x.buf_size = sprintf(x.buf_cmd,"ntp.Sync(%03d)\r\n", cmd_id);
	return x;
}
Buffer NTP_ReserveBufferTx4Hum(uint8_t cmd_id)
{
	Buffer x;
	x.buf_cmd = (char*) malloc (20);
	x.buf_size = sprintf(x.buf_cmd,"ReadMyDht(%03d)\r\n", cmd_id);
	return x;
}
Buffer NTP_ReserveBufferTxEnter(uint8_t cmd_id)
{
	Buffer x;
	x.buf_cmd = UART_TX_BUF();
	x.buf_size = 2;
	return x; //ret 2znaki = 0x3E, 0x20
}

void NTP_ExecuteSync(char* buf)
{
	UartCommand* cmd = 0;
	CM_NewCommand(&cmd);
	UC_CreateUartCommand(cmd, GetCmdID(), NTP_ReserveBufferTx4Time, ReadTime, NTP_ExecuteGetTime);
	CM_SendCommand(cmd);
}

void NTP_ExecuteGetTime(char* pBuf)
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

	if (HAL_RTC_SetTime(pRtc, &TimeToUpdate, RTC_FORMAT_BIN) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_RTC_SetDate(pRtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_RTCEx_BKUPWrite(pRtc, RTC_BKP_DR1, 0x32F2);
}

void NTP_ExecuteGetHumidity(char* pBuf)
{
	m_humidity = 0;

	char tempBuf[4];
	memset(&tempBuf, 0 , 4);
	memcpy(&tempBuf, &pBuf[19], 2);
	m_temperature = atoi(tempBuf);


	memset(&tempBuf, 0 , 4);
	memcpy(&tempBuf, &pBuf[31], 2);
	m_humidity = atoi(tempBuf);
}



uint8_t GetCmdID()
{
	return CmdID++;
}
uint8_t ReadCmdID()
{
	return CmdID;
}

void NTP_Execute(FuncUartRx pFuncUartRx, Buffer* buf)
{
	if(pFuncUartRx == ReadSync)
	{
		char* tt = buf->buf_cmd;
		uint8_t size = buf->buf_size;
	}
	if(pFuncUartRx == ReadTime)
	{
		char* tt = buf->buf_cmd;
		uint8_t size = buf->buf_size;
	}
}

void NTP_GetTimestamp(RTC_DateTypeDef* pDate, RTC_TimeTypeDef* pTime)
{
	HAL_RTC_GetDate(pRtc, pDate, RTC_FORMAT_BIN);
	HAL_RTC_GetTime(pRtc, pTime, RTC_FORMAT_BIN);
}
uint8_t NTP_GetHumidity()
{
	return m_humidity;
}
uint8_t NTP_GetTemperature()
{
	return m_temperature;
}
