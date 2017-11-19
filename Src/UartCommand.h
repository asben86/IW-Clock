/*
 * UartCommand.h
 *
 *  Created on: 15.11.2017
 *      Author: ben
 */

#ifndef UARTCOMMAND_H_
#define UARTCOMMAND_H_
#include "classdefs.h"

typedef struct UartCommand
{
	uint8_t m_cmd_id;
	FuncUartTx m_pFuncSend;
	FuncUartRx m_pFuncRecive;
	Buffer m_Tx;
	Buffer m_Rx;
	uint8_t m_IsRecivedData;

}UartCommand;

UartCommand UC_CreateUartCommand(uint8_t cmdId, FuncUartTx pFuncUartTx, FuncUartRx pFuncUartRx);

uint8_t UC_GetCmdId(UartCommand* pUCmd);

Buffer UC_GetTxBuffer(UartCommand* pUCmd);
char* UC_GetTxBufPtr(UartCommand* pUCmd);
uint16_t UC_GetTxBufSize(UartCommand* pUCmd);

Buffer UC_GetRxBuffer(UartCommand* pUCmd);
char* UC_GetRxBufPtr(UartCommand* pUCmd);
uint16_t UC_GetRxBufSize(UartCommand* pUCmd);

UartCommand* UC_FindUartCommand(UartCommand* pCommands, uint8_t size, uint8_t cmdId);

#endif /* UARTCOMMAND_H_ */
