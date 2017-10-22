/*
 * DbgUart.h
 *
 *  Created on: Aug 14, 2016
 *      Author: loywong
 */

#ifndef TSKTOP_DBGUART_H_
#define TSKTOP_DBGUART_H_

#include "FreeRTOS.h"
#include "queue.h"

namespace TskPrint
{
	extern QueueHandle_t MbCmd;
	void Init();
	int UartWrite(char *str);
	void UartGetLine(char *line);
}

#endif /* TSKTOP_DBGUART_H_ */
