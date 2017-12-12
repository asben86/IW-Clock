/*
 * Buffers.h
 *
 *  Created on: 23.11.2017
 *      Author: ben
 */

#ifndef BUFFERS_H_
#define BUFFERS_H_

#define UART_BUF_SIZE 	100
#define IW_18_BUF_SIZE 	10

static char Tx1[UART_BUF_SIZE] = {0};
static char Rx1[UART_BUF_SIZE] = {0};
static char IW18[IW_18_BUF_SIZE] = {0};

void Buffers_init();
char* UART_RX_BUF();
char* UART_TX_BUF();
char* IW18_BUF();

#endif /* BUFFERS_H_ */
