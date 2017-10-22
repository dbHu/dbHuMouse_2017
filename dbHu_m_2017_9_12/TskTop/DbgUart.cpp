/*
 * DbgUart.cc
 *
 *  Created on: Aug 14, 2016
 *      Author: loywong
 */
#include <file.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Assert.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/utils/Load.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Mailbox.h>

/* TI-RTOS Header files */
#include <ti/drivers/UART.h>

#include "../dbmouse_chassis.h"
#include "UARTUtils.h"
#include "DbgUart.h"
#include "TskTop.h"
#include <ti/drivers/GPIO.h>

UART_Handle dbgUart;

namespace TskPrint{

const int tskPrio = 2;
const int tskStkSize = 1024;
Task_Params tskParams;
Task_Handle tsk;

Mailbox_Handle MbCmd;
char msg[128];

void task(UArg arg0, UArg arg1)
{
    bool rtn;
    /*
     *  Add the UART device to the system.
     *  All UART peripherals must be setup and the module must be initialized
     *  before opening.  This is done by Board_initUART().  The functions used
     *  are implemented in UARTUtils.c.
     */
    add_device("UART", _MSA, UARTUtils_deviceopen,
               UARTUtils_deviceclose, UARTUtils_deviceread,
               UARTUtils_devicewrite, UARTUtils_devicelseek,
               UARTUtils_deviceunlink, UARTUtils_devicerename);

    /* Open UART0 for writing to stdout and set buffer */
    freopen("UART:0", "w", stdout);
    setvbuf(stdout, NULL, _IOLBF, 128);

    /* Open UART0 for reading from stdin and set buffer */
    freopen("UART:0", "r", stdin);
    setvbuf(stdin, NULL, _IOLBF, 128);

    /* Open UART0 for reading from stdin and set buffer */
    freopen("UART:0", "w", stderr);
    setvbuf(stderr, NULL, _IOLBF, 128);
    /*
     *  Initialize UART port 0 used by SysCallback.  This and other SysCallback
     *  UART functions are implemented in UARTUtils.c. Calls to System_printf
     *  will go to UART0, the same as printf.
     */
    UARTUtils_systemInit(0);
    while(true){
        rtn=Mailbox_pend(MbCmd, msg, BIOS_WAIT_FOREVER);
        Assert_isTrue(rtn,NULL);
        puts(msg);
        Task_sleep(5);
    }
}

void Init()
{
    Task_Params_init(&tskParams);
    tskParams.priority = tskPrio;
    tskParams.stackSize = tskStkSize;

    MbCmd = Mailbox_create(128, 10, NULL, NULL);
    if(MbCmd == NULL)
        System_abort("create TskPrint::MbCmd failed.\n");

    tsk = Task_create((Task_FuncPtr)task, &tskParams, NULL);
    if(tsk == NULL)
    {
        System_abort("Task Print failed");
    }

}

}

