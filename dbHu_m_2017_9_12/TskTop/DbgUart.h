/*
 * DbgUart.h
 *
 *  Created on: Aug 14, 2016
 *      Author: loywong
 */

#ifndef TSKTOP_DBGUART_H_
#define TSKTOP_DBGUART_H_

void InitDbgUart();

bool DbgUartPutLine(char *str, bool wait = false);

int DbgUartGetLine(char *str);

#endif /* TSKTOP_DBGUART_H_ */
