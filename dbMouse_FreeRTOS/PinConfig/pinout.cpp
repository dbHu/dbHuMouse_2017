//*****************************************************************************
//
// pinout.c - Function to configure the device pins on the EK-TM4C1294XL.
//
// Copyright (c) 2013-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 2.1.0.12573 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_gpio.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_sysctl.h"

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"


#include "pinout.h"

#define GPIOFUNC    1

//*****************************************************************************
//
//! \addtogroup pinout_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! Configures the device pins for the standard usages on the EK-TM4C1294XL.
//!
//! This function enables the GPIO modules and configures the device pins for
//! the default, standard usages on the EK-TM4C1294XL.  Applications that
//! require alternate configurations of the device pins can either not call
//! this function and take full responsibility for configuring all the device
//! pins, or can reconfigure the required device pins after calling this
//! function.
//!
//! \return None.
//
//*****************************************************************************
void Board_General_Init(void)
{
    //
    // Enable all the GPIO peripherals.
    //
    HWREG(SYSCTL_RCGCGPIO) = 0x00007FFF;

    // reset gpiod
    HWREG(SYSCTL_SRGPIO) |= 0x00007FFF;
    HWREG(SYSCTL_SRGPIO) &= ~0x00007FFF;
    while((HWREG(SYSCTL_PRGPIO) & 0x00007FFF) != 0x00007FFF)
    {
    }

    //UNLOCK PF0 PD7

    /* DBMOUSE_ENC_SR - PD7 requires unlocking before configuration */
    HWREG(GPIO_PORTD_AHB_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_AHB_BASE + GPIO_O_CR)  |= GPIO_PIN_7;
    HWREG(GPIO_PORTD_AHB_BASE | GPIO_O_DEN) |= 0x00000080;     //PD7 GPIO FUNCTION
    HWREG(GPIO_PORTD_AHB_BASE | GPIO_O_DIR) |= 0x00000010;     //PD7 DIR OUTPUT

#if GPIOFUNC
    //
    // PE1 IR_FL
    // PD5 IR_FR
    // PE3 IR_SL
    // PD7 IR_SR
    //
    HWREG(GPIO_PORTD_AHB_BASE | GPIO_O_DIR) |= 0x000000A0;     //PD5 PD7 DIR OUTPUT
    HWREG(GPIO_PORTD_AHB_BASE | GPIO_O_DEN) |= 0x000000A0;     //PD5 PD7 GPIO FUNCTION
    HWREG(GPIO_PORTD_AHB_BASE | GPIO_O_DATA)&= ~0x000000A0;    //OUTPUT 0

    HWREG(GPIO_PORTE_AHB_BASE | GPIO_O_DIR) |= 0x0000000A;     //PE1 PE3 DIR OUTPUT
    HWREG(GPIO_PORTE_AHB_BASE | GPIO_O_DEN) |= 0x0000000A;     //PE1 PE3 GPIO FUNCTION
    HWREG(GPIO_PORTE_AHB_BASE | GPIO_O_DATA)&= ~0x0000000A;    //OUTPUT 0

    //
    // PD2 -> PA5 and PD3 -> PM1 are used for USER LEDs.
    //
    HWREG(GPIO_PORTA_AHB_BASE | GPIO_O_DIR) |= 0x00000020;     //PA5 DIR OUTPUT
    HWREG(GPIO_PORTA_AHB_BASE | GPIO_O_DEN) |= 0x00000020;     //PA5 PA5 GPIO FUNCTION
    HWREG(GPIO_PORTA_AHB_BASE | GPIO_O_DATA)|= 0x00000020;     //OUTPUT 1

    HWREG(GPIO_PORTM_BASE | GPIO_O_DIR) |= 0x00000002;     //PM1 DIR OUTPUT
    HWREG(GPIO_PORTM_BASE | GPIO_O_DEN) |= 0x00000002;     //PM1 GPIO FUNCTION
    HWREG(GPIO_PORTM_BASE | GPIO_O_DATA)|= 0x00000002;     //OUTPUT 1
#endif

#if I2CFUNC
    //
    // PD0-1 are used for I2C7.
    //
    HWREG(SYSCTL_RCGCI2C) |= 0x000000080;
    HWREG(GPIO_PORTD_AHB_BASE | GPIO_O_AFSEL) |= 0x00000003;    // port D[1..0] as alternate func
    HWREG(GPIO_PORTD_AHB_BASE | GPIO_O_DEN)   |= 0x00000003;    // port D[1..0] digital enable
    // port D[1..0] as AF2(uart)
    HWREG(GPIO_PORTA_BASE | GPIO_O_PCTL) = HWREG(GPIO_PORTA_BASE | GPIO_O_PCTL) & 0xFFFFFF00 | 0x00000022;

    HWREG(I2C7_BASE | I2C_O_MCR) = 0x00000010;              // initialize I2C master 
    /*
     * SCL clock speed of 400 Kbps 
     * TPR = (System clock / 2 * (SCL_LP + SCL_HP) * SCL_CLK)) - 1
     * TPR = (120M / 2 * (6 + 4) * 400000) - 1 = 14
     */
    HWREG(I2C7_BASE | I2C_O_MTPR) = 0x0000000E;             
#endif
}

//*****************************************************************************
//
//! This function writes a state to the LED bank.
//!
//! \param ui32LEDMask is a bit mask for which GPIO should be changed by this
//! call.
//! \param ui32LEDValue is the new value to be applied to the LEDs after the
//! ui32LEDMask is applied.
//!
//! The first parameter acts as a mask.  Only bits in the mask that are set
//! will correspond to LEDs that may change.  LEDs with a mask that is not set
//! will not change. This works the same as GPIOPinWrite. After applying the
//! mask the setting for each unmasked LED is written to the corresponding
//! LED port pin via GPIOPinWrite.
//!
//! \return None.
//
//*****************************************************************************
void LED_write(unsigned int index, unsigned int value)
{
    //
    // Check the mask and set or clear the LED as directed.
    //
    if(index & 0x1)        //DBMOUSE_LED_0 PA5
    {
        if (value)    //DBMOUSE_LED_OFF
        {
            HWREG(GPIO_PORTA_AHB_BASE | GPIO_O_DATA)|= 0x00000020;     //OUTPUT 1
        }
        else                //DBMOUSE_LED_ON
        {
            HWREG(GPIO_PORTA_AHB_BASE | GPIO_O_DATA)&= ~0x00000020;     //OUTPUT 0
        }
    }

    if (index & 0x2)        //DBMOUSE_LED_1 PM1
    {
        if (value)    //DBMOUSE_LED_OFF
        {
            HWREG(GPIO_PORTM_BASE | GPIO_O_DATA)|= 0x00000002;     //OUTPUT 1
        }
        else                //DBMOUSE_LED_ON
        {
            HWREG(GPIO_PORTM_BASE | GPIO_O_DATA)&= ~0x00000002;     //OUTPUT 0
        }
    }

}

void IR_write(unsigned int index, unsigned int value)
{
    //
    // PE1 IR_FL
    // PD5 IR_FR
    // PE3 IR_SL
    // PD7 IR_SR
    //
    if(index == DBMOUSE_IR_FL)        //DBMOUSE_IR_FL
    {
        if (value)
        {
            HWREG(GPIO_PORTE_AHB_BASE | GPIO_O_DATA)|= 0x00000002;     //OUTPUT 1
        }
        else
        {
            HWREG(GPIO_PORTE_AHB_BASE | GPIO_O_DATA)&= ~0x00000002;     //OUTPUT 0
        }
    }

    if (index == DBMOUSE_IR_FR)       //DBMOUSE_IR_FR
    {
        if (value)
        {
            HWREG(GPIO_PORTD_AHB_BASE | GPIO_O_DATA)|= 0x00000020;     //OUTPUT 1
        }
        else
        {
            HWREG(GPIO_PORTD_AHB_BASE | GPIO_O_DATA)&= ~0x00000020;     //OUTPUT 0
        }
    }

    if (index == DBMOUSE_IR_SL)        //DBMOUSE_IR_SL
    {
        if (value)
        {
            HWREG(GPIO_PORTE_BASE | GPIO_O_DATA)|= 0x00000008;     //OUTPUT 1
        }
        else
        {
            HWREG(GPIO_PORTE_BASE | GPIO_O_DATA)&= ~0x00000008;     //OUTPUT 0
        }
    }

    if (index == DBMOUSE_IR_SR)       //DBMOUSE_IR_SR
    {
        if (value)
        {
            HWREG(GPIO_PORTD_AHB_BASE | GPIO_O_DATA)|= 0x00000080;     //OUTPUT 1
        }
        else
        {
            HWREG(GPIO_PORTD_AHB_BASE | GPIO_O_DATA)&= ~0x00000080;     //OUTPUT 0
        }
    }
}
//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
