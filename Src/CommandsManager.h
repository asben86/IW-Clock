/*
 * CommandsManager.h
 *
 *  Created on: 23.11.2017
 *      Author: ben
 */

#ifndef COMMANDSMANAGER_H_
#define COMMANDSMANAGER_H_
#include "stm32f1xx_hal.h"
#include "UartCommand.h"
#include <stdlib.h>     /* malloc, free, rand */

enum DevState
{
	Boot 				= 0,
	Clock_set			= 1,
	Clock_display 		= 2,
	Humidity_display 	= 3,
	Temperature_display = 4,
	Cmd_display 		= 5,
};


static uint8_t DeviceState = Boot;
static UART_HandleTypeDef* pUart = 0;
static UartCommand* pCommands = 0;
static uint8_t indexCmd = 0;
static uint8_t commands_count = 5;
static uint8_t humidity_send = 0;
void CM_SetUartHandle(UART_HandleTypeDef *huart);
void CM_init();

void CM_ExecuteCommand();
void CM_SendCommand(UartCommand* pCmd);
void CM_StateManager();
UartCommand* CM_GetCommands();
uint8_t CM_GetCommandsCount();

void CM_NewCommand(UartCommand** pCmd);
void ExecuteReceivedCommands();
UartCommand* CM_FindUartCommand(uint8_t cmdId);

#endif /* COMMANDSMANAGER_H_ */
