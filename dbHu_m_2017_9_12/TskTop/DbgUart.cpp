/*
 * DbgUart.cc
 *
 *  Created on: Aug 14, 2016
 *      Author: loywong
 */

#include <xdc/runtime/System.h>
#include "../dbmouse_chassis.h"
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/UART.h>
#include <ti/sysbios/BIOS.h>
#include <string.h>
#include "DbgUart.h"

UART_Handle dbgUart;
Semaphore_Handle semDbgUart;

void dbgUartWriteCb(UART_Handle handle, void *buf, size_t count)
{
    Semaphore_post(semDbgUart);
}

// last char of str will be modified to '\n' if it's not '\n'
bool DbgUartPutLine(char *str, bool wait)
{
    if(!Semaphore_pend(semDbgUart, BIOS_NO_WAIT))
        return false;

    int len = strlen(str);
    if(str[len - 1] != '\n') str[len - 1] = '\n';

    UART_write(dbgUart, str, strlen(str));

    if(wait)
    {
        Semaphore_pend(semDbgUart, BIOS_WAIT_FOREVER);
        Semaphore_post(semDbgUart);
    }
    return true;
}

int DbgUartGetLine(char *str)
{
    int len;
    len = UART_read(dbgUart, str, UART_BUF_LEN);
    Task_sleep(10);
    UART_readCancel(dbgUart);
    if(len > 0)
        str[len-1] = '\0';
    return len;
}

void InitDbgUart()
{
    Semaphore_Params semParams;
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    semDbgUart = Semaphore_create(1, &semParams, NULL);
    if(semDbgUart == NULL)
        System_abort("create semDbgUart failed.\n");

    UART_Params uartParams;
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readReturnMode = UART_RETURN_NEWLINE;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.writeMode = UART_MODE_CALLBACK;
    uartParams.readMode = UART_MODE_BLOCKING;
    uartParams.baudRate = 115200;
    uartParams.dataLength = UART_LEN_8;
    uartParams.parityType = UART_PAR_NONE;
    uartParams.stopBits = UART_STOP_ONE;
    uartParams.writeCallback = dbgUartWriteCb;
    dbgUart = UART_open(DBMOUSE_UART_DBG, &uartParams);
    if(dbgUart == NULL)
        System_abort("open UART failed!\n");
}


