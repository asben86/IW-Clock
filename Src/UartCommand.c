/*
 * UartCommand.c
 *
 *  Created on: 15.11.2017
 *      Author: ben
 */

#include "UartCommand.h"

void UC_CreateUartCommand(UartCommand* ptr, uint8_t cmdId, FuncUartTx pFuncUartTx, FuncUartRx pFuncUartRx, FuncUartExe pExe)
{
	ptr->m_cmd_id = cmdId;
	ptr->m_Tx = pFuncUartTx(cmdId);
	ptr->m_Rx = pFuncUartRx(cmdId);
	ptr->m_pFuncSend = pFuncUartTx;
	ptr->m_pFuncRecive = pFuncUartRx;
	ptr->m_pFuncExecute = pExe;
	ptr->m_IsTransmitedData = 0;
	ptr->m_IsRecivedData = 0;
	ptr->m_IsExecuted = 0;
}

uint8_t UC_GetCmdId(UartCommand* pUCmd)
{
	return pUCmd->m_cmd_id;
}

Buffer UC_GetTxBuffer(UartCommand* pUCmd)
{
	return pUCmd->m_pFuncSend(pUCmd->m_cmd_id);
}

char* UC_GetTxBufPtr(UartCommand* pUCmd)
{
	return pUCmd->m_pFuncSend(pUCmd->m_cmd_id).buf_cmd;
}

uint16_t UC_GetTxBufSize(UartCommand* pUCmd)
{
	return pUCmd->m_pFuncSend(pUCmd->m_cmd_id).buf_size;
}

Buffer UC_GetRxBuffer(UartCommand* pUCmd)
{
	return pUCmd->m_pFuncRecive(pUCmd->m_cmd_id);
}

char* UC_GetRxBufPtr(UartCommand* pUCmd)
{
	return pUCmd->m_pFuncRecive(pUCmd->m_cmd_id).buf_cmd;
}

uint16_t UC_GetRxBufSize(UartCommand* pUCmd)
{
	return pUCmd->m_pFuncRecive(pUCmd->m_cmd_id).buf_size;
}

UartCommand* UC_FindUartCommand(UartCommand* pCommands, uint8_t size, uint8_t cmdId)
{
	for(uint8_t i = 0; i<size; ++i)
	{
		UartCommand* ptr = pCommands+i;
		if(ptr->m_cmd_id == cmdId)
		{
			return ptr;
		}
	}
	return 0;
}
