/*
 * Ntp.h
 *
 *  Created on: 14.11.2017
 *      Author: ben
 */

#ifndef NTP_H_
#define NTP_H_
#include "classdefs.h"

static char Tx[100] ={0};
static char* pTx = &Tx[0];

static char Rx[100] ={0};
static char* pRx = &Rx[0];
static uint8_t CmdID = 0;


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

Buffer ReadTime();
Buffer ReadYear();
Buffer ReadSync();

Buffer GetYear(uint8_t cmd_id);
Buffer GetTime(uint8_t cmd_id);
Buffer GetSync(uint8_t cmd_id);
uint8_t GetCmdID();
uint8_t ReadCmdID();

void NTP_Execute(FuncUartRx pFuncUartRx, Buffer* buf);

#endif /* NTP_H_ */
