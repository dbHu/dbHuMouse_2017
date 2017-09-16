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

#ifndef __BOARD_H
#define __BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "dbmouse_chassis.h"

#define Board_initDMA               DBMOUSE_initDMA
#define Board_initGeneral           DBMOUSE_initGeneral
#define Board_initGPIO              DBMOUSE_initGPIO
#define Board_initI2C               DBMOUSE_initI2C
#define Board_initPWM               DBMOUSE_initPWM
#define Board_initSDSPI             DBMOUSE_initSDSPI
#define Board_initSPI               DBMOUSE_initSPI
#define Board_initUART              DBMOUSE_initUART
#define Board_initUSB               DBMOUSE_initUSB
#define Board_initWatchdog          DBMOUSE_initWatchdog
#define Board_initWiFi              DBMOUSE_initWiFi

#define Board_LED_0                 DBMOUSE_LED_0
#define Board_LED_1                 DBMOUSE_LED_1
#define Board_LED_2                 DBMOUSE_LED_2
#define Board_LED_3                 DBMOUSE_LED_3

#define Board_I2C_IMU               DBMOUSE_I2C_IMU
//#define Board_I2C1                  DBMOUSE_I2C3
//#define Board_I2C_TMP               DBMOUSE_I2C3
//#define Board_I2C_NFC               DBMOUSE_I2C3
//#define Board_I2C_TPL0401           DBMOUSE_I2C3

#define Board_PWM0                  DBMOUSE_PWM6
#define Board_PWM1                  DBMOUSE_PWM7

#define Board_SDSPI0                DBMOUSE_SDSPI0

#define Board_SPI0                  DBMOUSE_SPI0
#define Board_SPI1                  DBMOUSE_SPI3

#define Board_USBDEVICE             DBMOUSE_USBDEVICE

#define Board_UART0                 DBMOUSE_UART_DBG

#define Board_WATCHDOG0             DBMOUSE_WATCHDOG0

#define Board_WIFI                  DBMOUSE_WIFI
#define Board_WIFI_SPI              DBMOUSE_SPI2

/* Board specific I2C addresses */
#define Board_TMP006_ADDR           (0x40)
#define Board_RF430CL330_ADDR       (0x28)
#define Board_TPL0401_ADDR          (0x40)

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
