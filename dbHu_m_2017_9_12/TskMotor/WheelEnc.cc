/*
 * MotorEnc.cc
 *
 *  Created on: Aug 3, 2016
 *      Author: loywong
 */

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/hal/Hwi.h>

#include <xdc/runtime/System.h>

#include <inc/hw_qei.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_ints.h>
#include <inc/hw_sysctl.h>
#include <inc/hw_gpio.h>

#include "../dbmouse_chassis.h"
#include "WheelEnc.h"
#include "../physparams.h"

#include <ti/drivers/GPIO.h>

Semaphore_Handle SemMotTick;
Semaphore_Handle SemIrTick;
Semaphore_Handle SemActTick;

Hwi_Handle hwiQeiHandle;
Hwi_Params hwiQeiParams;

void qeiISR(UArg arg)
{
//TODO
//    Hwi_clearInterrupt(INT_QEI1_TM4C123);
//    HWREG(QEI1_BASE | QEI_O_ISC) = 0x2; // clear
    Semaphore_post(SemMotTick);
    Semaphore_post(SemIrTick);
    Semaphore_post(SemActTick);
}

void WheelEncInit()
{
    Semaphore_Params semParams;
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    SemMotTick = Semaphore_create(0, &semParams, NULL);
    if(SemMotTick == NULL)
        System_abort("create SemMotTick failed.\n");

    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    SemIrTick = Semaphore_create(0, &semParams, NULL);
    if(SemIrTick == NULL)
        System_abort("create SemIrTick failed.\n");

    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    SemActTick = Semaphore_create(0, &semParams, NULL);
    if(SemActTick == NULL)
        System_abort("SemActTick creation failed.");

//TODO
//    HWREG(SYSCTL_RCGCQEI) = 0x03;    // enable QEI0 & 1
//
//    HWREG(GPIO_PORTD_BASE | GPIO_O_AFSEL) |= 0x000000C0;    // port d[7..6] as alternate unc
//    HWREG(GPIO_PORTD_BASE | GPIO_O_DEN) |= 0x000000C0;    // port d[7..6] digital enable
//    HWREG(GPIO_PORTD_BASE | GPIO_O_PCTL) = HWREG(GPIO_PORTD_BASE | GPIO_O_PCTL) & 0x00FFFFFF | 0x66000000;  // port d[7..6] as af6(qei)
//    HWREG(GPIO_PORTC_BASE | GPIO_O_AFSEL) |= 0x00000060;    // port c[6..5] as alternate func
//    HWREG(GPIO_PORTC_BASE | GPIO_O_DEN) |= 0x00000060;    // port c[6..5] digital enable
//    HWREG(GPIO_PORTC_BASE | GPIO_O_PCTL) = HWREG(GPIO_PORTC_BASE | GPIO_O_PCTL) & 0xF00FFFFF | 0x06600000;  // port c[6..5] as af6(qei)
//
//    HWREG(QEI0_BASE | QEI_O_CTL) = 0x000F2028;   // filter-15, no stall, no inv, no div, vel en
//    HWREG(QEI0_BASE | QEI_O_MAXPOS) = 12;
//    HWREG(QEI0_BASE | QEI_O_LOAD) = 79999;//1ms. //999; // 1s test
//
//    HWREG(QEI1_BASE | QEI_O_CTL) = 0x000F2028;   // filter-15, no stall, no inv, no div, vel en
//    HWREG(QEI1_BASE | QEI_O_MAXPOS) = 12;
//    HWREG(QEI1_BASE | QEI_O_LOAD) = 79999;//1ms. //999; // 1s test
//
//    HWREG(QEI0_BASE | QEI_O_CTL) |= 0x00000001;   // start qei
//    HWREG(QEI1_BASE | QEI_O_CTL) |= 0x00000001;   // start qei

//    Hwi_Params_init(&hwiQeiParams);
//    hwiQeiParams.priority = 240;
//    hwiQeiParams.maskSetting = Hwi_MaskingOption_SELF;
//    hwiQeiHandle = Hwi_create(INT_QEI1_TM4C123, qeiISR, &hwiQeiParams, NULL);
//    Hwi_clearInterrupt(INT_QEI1_TM4C123);
//    HWREG(QEI1_BASE | QEI_O_ISC) = 0x2; // clear
//    HWREG(QEI1_BASE | QEI_O_INTEN) = 0x2; // enable qei_timer int
}

float WheelEncGetVel(volatile float &r, volatile float &l)
{
    int rp, lp;
    rp = (HWREG(QEI0_BASE | QEI_O_STAT) & 0x00000002)?
            -HWREG(QEI0_BASE | QEI_O_SPEED) :
            HWREG(QEI0_BASE | QEI_O_SPEED);
    lp = (HWREG(QEI1_BASE | QEI_O_STAT) & 0x00000002)?
            HWREG(QEI1_BASE | QEI_O_SPEED) :
            -HWREG(QEI1_BASE | QEI_O_SPEED);
    r = (float)rp * EncUnit * CP::EncoderUnitCompensation;
    l = (float)lp * EncUnit * CP::EncoderUnitCompensation;
    return 0.5f * (r + l);
}
//---------------------------------------------------

