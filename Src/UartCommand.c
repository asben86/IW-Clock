/*
 * UartCommand.c
 *
 *  Created on: 15.11.2017
 *      Author: ben
 */

#include "UartCommand.h"

UartCommand UC_CreateUartCommand(uint8_t cmdId, FuncUartTx pFuncUartTx, FuncUartRx pFuncUartRx)
{
	UartCommand cmd;
	cmd.m_cmd_id = cmdId;
	cmd.m_Tx = pFuncUartTx(cmdId);
	cmd.m_Rx = pFuncUartRx(cmdId);
	cmd.m_pFuncSend = pFuncUartTx;
	cmd.m_pFuncRecive = pFuncUartRx;
	cmd.m_IsRecivedData = 0;
	return cmd;
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
