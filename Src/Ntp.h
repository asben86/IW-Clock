/*
 * Ntp.h
 *
 *  Created on: 14.11.2017
 *      Author: ben
 */

#ifndef NTP_H_
#define NTP_H_
#include "classdefs.h"
#include "stm32f1xx_hal.h"

#define BUF_SIZE  200
static char Tx[BUF_SIZE];
static char Rx[BUF_SIZE];

static char* pTx = NULL;
static char* pRx = NULL;
static RTC_HandleTypeDef* pRtc = NULL;

static uint8_t CmdID = 250;
static char pEnter[2] = {0x0d, 0x0A}; //\r\n
static char pEnterOdp[2] = {0x00, 0x00}; //\r\n
static uint8_t m_humidity = 0;
static uint8_t m_temperature = 0;
	enum NTP_CMD
	{
		GET_SYNC 	= 0,
		GET_TIME 	= 1,
		GET_YEAR 	= 2,
		GET_MONTH 	= 3,
		GET_DAY		= 4,
		GET_HOUR 	= 5,
		GET_MIN 	= 6,
		GET_SEC 	= 7,
		LAST_NTP	= GET_SEC + 1,
	};
void  NTP_init(RTC_HandleTypeDef* pHrtc);
Buffer ReadTime();
Buffer ReadSync();
Buffer ReadEnter();
Buffer ReadHumi();

Buffer NTP_ReserveBufferTx4Time(uint8_t cmd_id);
Buffer NTP_ReserveBufferTx4Sync(uint8_t cmd_id);
Buffer NTP_ReserveBufferTx4Hum(uint8_t cmd_id);
Buffer NTP_ReserveBufferTxEnter(uint8_t cmd_id);

void NTP_ExecuteSync(char* pBuf);
void NTP_ExecuteGetTime(char* pBuf);
void NTP_ExecuteGetHumidity(char* pBuf);

uint8_t GetCmdID();
uint8_t ReadCmdID();

void NTP_Execute(FuncUartRx pFuncUartRx, Buffer* buf);
void NTP_GetTimestamp(RTC_DateTypeDef* pDate, RTC_TimeTypeDef* pTime);
uint8_t NTP_GetHumidity();
uint8_t NTP_GetTemperature();


#endif /* NTP_H_ */
