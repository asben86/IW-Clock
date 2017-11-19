/*
 * classdefs.hpp
 *
 *  Created on: 31.10.2017
 *      Author: Andrzej Spintzyk
 */

#ifndef CLASSDEFS_H_
#define CLASSDEFS_H_
#include "stdint.h"
typedef enum
{
  ON = 0,
  OFF =1,
}IW18_PinState;


typedef struct Buffer{
	char* buf_cmd;
	uint16_t buf_size;
}Buffer;

typedef Buffer (*FuncUartTx)(uint8_t);
typedef Buffer (*FuncUartRx)();
#endif /* CLASSDEFS_H_ */
