/*
 * Ntp.c
 *
 *  Created on: 14.11.2017
 *      Author: ben
 */

#include "Ntp.h"
#include <string.h> 	/* memset */
#include <stdlib.h>     /* atoi */

static const CONST_PREFIX = 2 + 3 + 2; // "> 000....\r\n"


Buffer ReadTime()
{
	Buffer x;
	x.buf_cmd = pRx;
	x.buf_size = 19 + CONST_PREFIX;
	pRx+=x.buf_size;
	return x;
}

Buffer ReadYear()
{
	Buffer x;
	x.buf_cmd = pRx;
	x.buf_size = 3;
	pRx+=x.buf_size;
	return x;
}

Buffer ReadSync()
{
	Buffer x;
	x.buf_cmd = pRx;
	x.buf_size = 5 + CONST_PREFIX;
	pRx+=x.buf_size;

	return x;
}

Buffer GetYear(uint8_t cmd_id)
{
	if((&Tx[100] - pTx) <30)
	{
		pTx=&Tx[0];
		memset(pTx, 0, 100);
	}
	Buffer x;
	x.buf_cmd = pTx;
	int size = sizeof("ntp.GetYear(%03d)\r\n");
	x.buf_size = sprintf(x.buf_cmd,"ntp.GetYear(%03d)\r\n", cmd_id);
	pTx+=x.buf_size + 1; // behind 0 which end string
	return x;
}

Buffer GetTime(uint8_t cmd_id)
{
	Buffer x;
	x.buf_cmd = pTx;
	x.buf_size = sprintf(x.buf_cmd,"ntp.GetTime(%03d)\r\n", cmd_id);
	pTx+=x.buf_size + 1; // behind 0 which end string
	return x;
}

Buffer GetSync(uint8_t cmd_id)
{
	Buffer x;
	x.buf_cmd = pTx;
	x.buf_size = sprintf(x.buf_cmd,"ntp.Sync(%03d)\r\n", cmd_id);
	pTx+=x.buf_size + 1; // behind 0 which end string
	return x;
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
