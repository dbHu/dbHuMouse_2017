/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== empty.c ========
 */
/* XDCtools Header files */
#include <dbmouse_chassis.h>
//#include <xdc/std.h>
//#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
//#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
//#include <driverlib/fpu.h>
//#include <driverlib/ROM.h>
#include <TskTop/TskTop.h>
#include "Board.h"
#include "includes.h"
//
//#include <stdbool.h>
//#include <inc/hw_ints.h>
//#include <inc/hw_memmap.h>
//#include <inc/hw_types.h>
//#include <inc/hw_gpio.h>
//#include <driverlib/sysctl.h>
//#include <driverlib/gpio.h>

#define testflash 0
/*
 *  ======== main ========
 */
int main(void)
{
//    Task_Params taskParams;
//    FPULazyStackingEnable();
//    FPUEnable();


    Board_initGeneral();
    Board_initGPIO();
    Board_initI2C();
//    Board_initPWM();
    // Board_initSDSPI();
    // Board_initSPI();
    Board_initUART();
    // Board_initUSB(Board_USBDEVICE);
    // Board_initWatchdog();
    // Board_initWiFi();

//    /* Turn on user LED */
//    GPIO_write(DBMOUSE_LED_0, DBMOUSE_LED_ON);
//    GPIO_write(DBMOUSE_LED_1, DBMOUSE_LED_ON);

//    System_printf("Starting the example\nSystem provider is set to SysMin. "
//                  "Halt the target to view any SysMin contents in ROV.\n");
//    /* SysMin will only print to the console when you call flush or exit */
//    System_flush();

#if testflash
	unsigned char testStr[10] = {0x01,0x02,0x03,0x04,0x05,0x06,0x00,0x00,0x00,0x00};
	int a = sizeof(testStr);
    TskIr::eraseFlashBlock(254);
    TskIr::programFlash(254 * 1024,(unsigned int*)&testStr[0],sizeof(testStr) / 2);
    TskIr::ReadFlash(0x3F800,(unsigned char*)&testStr[0],sizeof(testStr));
#endif

    TskTop::Init();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
