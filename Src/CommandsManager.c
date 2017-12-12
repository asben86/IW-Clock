/*
 * CommandsManager.c
 *
 *  Created on: 23.11.2017
 *      Author: ben
 */

#include "CommandsManager.h"
#include "UartCommand.h"
#include "Ntp.h"
#include "Buffers.h"
#include <string.h> /* memset */

void CM_SetUartHandle(UART_HandleTypeDef *huart)
{
	pUart = huart;
}

void CM_init()
{
	pCommands = (UartCommand*) malloc (sizeof(UartCommand) * commands_count);
	memset(pCommands, 0, sizeof(UartCommand) * commands_count);
}
void CM_SendCommand(UartCommand* pCmd)
{
	HAL_StatusTypeDef def = HAL_UART_Transmit_IT(pUart, (uint8_t*)(pCmd->m_Tx.buf_cmd), pCmd->m_Tx.buf_size);
	pCmd->m_IsTransmitedData = 1;
}
void CM_ExecuteCommand()
{
	RTC_DateTypeDef date;
	RTC_TimeTypeDef time;
	NTP_GetTimestamp(&date, &time);
	if(DeviceState == Boot)
	{
		UartCommand* cmd = 0;
		CM_NewCommand(&cmd);
		UC_CreateUartCommand(cmd, GetCmdID(), NTP_ReserveBufferTx4Sync, ReadSync, NTP_ExecuteSync);
		CM_SendCommand(cmd);


		sprintf(IW18_BUF(), "........");
		DeviceState = Clock_set;
		//HAL_Delay(1000);
	}
	if(DeviceState == Clock_set)
	{
		sprintf(IW18_BUF(), "--------");
		HAL_Delay(1000);
	}
	if(DeviceState == Clock_display)
	{
		  sprintf(IW18_BUF(), "%02d-%02d-%02d", time.Hours, time.Minutes, time.Seconds );

		//Send humidity cmd
	}
	if(DeviceState == Humidity_display)
	{
		  sprintf(IW18_BUF(), "h--%02d---", NTP_GetHumidity());
		  if(humidity_send==0)
		  {
				UartCommand* cmd = 0;
				CM_NewCommand(&cmd);
				UC_CreateUartCommand(cmd, GetCmdID(), NTP_ReserveBufferTx4Hum, ReadHumi, NTP_ExecuteGetHumidity);
				CM_SendCommand(cmd);
				humidity_send =1 ;
		  }
	}

	if(DeviceState == Temperature_display)
	{
		humidity_send = 0;
		  sprintf(IW18_BUF(), "---%02d---", NTP_GetTemperature());
	}
	if(DeviceState == Cmd_display)
	{
		humidity_send = 0;
		  sprintf(IW18_BUF(), "5--%03d--", ReadCmdID());
	}
	ExecuteReceivedCommands();

}
void CM_StateManager()
{
	if(DeviceState == Boot)
	{

	}

	else
	{
		RTC_DateTypeDef date;
		RTC_TimeTypeDef time;
		NTP_GetTimestamp(&date, &time);

		if(time.Seconds >20 && time.Seconds <25)
		{
			DeviceState = Humidity_display;
		}
		else if(time.Seconds >=25 && time.Seconds <30)
		{
			DeviceState = Temperature_display;
		}

		else if(time.Seconds == 50 && time.Minutes == 16)
		{
			DeviceState = Clock_set;
		}
		else if(time.Seconds >=5 && time.Seconds <7)
		{
			DeviceState = Cmd_display;
		}

		else
		{
			DeviceState = Clock_display;
		}
	}
}

UartCommand* CM_GetCommands()
{
	return pCommands;
}

uint8_t CM_GetCommandsCount()
{
	return indexCmd;
}

void CM_NewCommand(UartCommand** pCmd)
{
	if(indexCmd >= commands_count)
	{
		memset(pCommands, 0, sizeof(UartCommand) * commands_count);
		*pCmd = pCommands;
		indexCmd=0;
	}
	else
	{
		*pCmd = pCommands + indexCmd;
		indexCmd++;
	}
}

void ExecuteReceivedCommands()
{
	for(uint8_t i = 0; i<commands_count; ++i)
	{
		UartCommand* ptr = pCommands+i;
		if(ptr->m_cmd_id>0 && ptr->m_IsExecuted == 0 && ptr->m_IsRecivedData > 0)
		{
			ptr->m_pFuncExecute(ptr->m_Rx.buf_cmd);
			ptr->m_IsExecuted = 1;
			free(ptr->m_Rx.buf_cmd);
			free(ptr->m_Tx.buf_cmd);
			memset(ptr, 0, sizeof(UartCommand));
		}
	}
}


UartCommand* CM_FindUartCommand(uint8_t cmdId)
{
	for(uint8_t i = 0; i<commands_count; ++i)
	{
		UartCommand* ptr = pCommands+i;
		if(ptr->m_cmd_id == cmdId)
		{
			return ptr;
		}
	}
	return 0;
}
