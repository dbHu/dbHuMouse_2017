/*
 * DbgUart.h
 *
 *  Created on: Aug 14, 2016
 *      Author: loywong
 */

#ifndef TSKTOP_DBGUART_H_
#define TSKTOP_DBGUART_H_

#include <ti/sysbios/knl/Mailbox.h>

namespace TskPrint
{
	extern Mailbox_Handle MbCmd;
	void Init();
	bool DbgUartPutLine(char* str, bool wait);
}

#endif /* TSKTOP_DBGUART_H_ */
