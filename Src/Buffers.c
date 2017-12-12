/*
 * Buffers.c
 *
 *  Created on: 23.11.2017
 *      Author: ben
 */

#include "Buffers.h"
#include <string.h> /* memset */

void Buffers_init()
{
	memset(UART_RX_BUF(), 0, UART_BUF_SIZE);
	memset(UART_TX_BUF(), 0, UART_BUF_SIZE);
	memset(IW18_BUF(), 0, IW_18_BUF_SIZE);
}

char* UART_RX_BUF()
{
	return &Rx1[0];
}

char* UART_TX_BUF()
{
	return &Tx1[0];
}

char* IW18_BUF()
{
	return &IW18[0];
}
