/*
 * MotorEnc.cc
 *
 *  Created on: Aug 3, 2016
 *      Author: loywong
 */

#include <xdc/runtime/System.h>

#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/hal/Timer.h>
#include <ti/sysbios/hal/Hwi.h>

#include "Board.h"
#include "TskMotor/WheelEnc.h"
#include <ti/drivers/GPIO.h>

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_ints.h>
#include <inc/hw_gpio.h>
#include <inc/hw_timer.h>
#include <inc/hw_sysctl.h>

#include "driverlib/timer.h"

#define ENCRA  HWREG(GPIO_PORTE_AHB_BASE + GPIO_O_DATA) & 0x10
#define ENCRB  HWREG(GPIO_PORTB_AHB_BASE + GPIO_O_DATA) & 0x20
#define ENCLA  HWREG(GPIO_PORTK_BASE + GPIO_O_DATA) & 0x01
#define ENCLB  HWREG(GPIO_PORTK_BASE + GPIO_O_DATA) & 0x02

volatile int rotateR = 0,rotateL = 0;
int rp,lp;

Semaphore_Handle SemMotTick;
Semaphore_Handle SemIrTick;
Semaphore_Handle SemActTick;

int val = 0;

void BaseTimerISR(unsigned int index)
{
    HWREG(TIMER3_BASE | TIMER_O_ICR) |= 0x00000001;      // clear TimerA timeout int
    Hwi_clearInterrupt(INT_TIMER3A_TM4C129);
    Semaphore_post(SemMotTick);
    Semaphore_post(SemIrTick);
    Semaphore_post(SemActTick);
    lp = rotateL;
    rp = rotateR;
    rotateL = 0;
    rotateR = 0;
}

Hwi_Handle hwiTimerHandle;
Hwi_Params hwiTimerParams;

//void WheelEncInit()
void BaseTimeInit()
{
    GPIO_write(DBMOUSE_LED_1, DBMOUSE_LED_ON);
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

    HWREG(SYSCTL_RCGCTIMER) |= 0x000000008;            // enable Timer3
    GPIO_write(DBMOUSE_LED_2, DBMOUSE_LED_ON);
    HWREG(TIMER3_BASE | TIMER_O_CTL) &= ~0x00000001; // GPTM Timer A clear
    HWREG(TIMER3_BASE | TIMER_O_CFG) = 0x00000000;   // 32-bit timer configuration
    HWREG(TIMER3_BASE | TIMER_O_TAMR) = 0x00000002;  // Timer A Periodic mode.
    HWREG(TIMER3_BASE | TIMER_O_TAILR) = 0x0001D4BF; // Timer A count down 120000
    HWREG(TIMER3_BASE | TIMER_O_IMR) = 0x00000001;  // Timer A timeout interrupt
    Hwi_Params_init(&hwiTimerParams);
    hwiTimerParams.priority = 240;
    hwiTimerParams.maskSetting = Hwi_MaskingOption_ALL;
    hwiTimerHandle = Hwi_create(INT_TIMER3A_TM4C129, BaseTimerISR, &hwiTimerParams, NULL);
    Hwi_clearInterrupt(INT_TIMER3A_TM4C129);
    HWREG(TIMER3_BASE | TIMER_O_CTL) = 0x00000001;  // GPTM Timer A enable
    HWREG(TIMER3_BASE | TIMER_O_ICR) = 0x00000001;  // clear TimerA timeout int
}

/*
 *  ======== portEISR ========
 *  Callback function for the GPIO interrupt on Board_ENCRA.
 *  
 */

void portEISR(unsigned int index)
{
    Hwi_disable();

    HWREG(GPIO_PORTE_AHB_BASE | GPIO_O_ICR) |= 0x00000010;      // clear PE4 int
    Hwi_clearInterrupt(INT_GPIOE_TM4C129);
    /*
     * A Rise-edge B High --
     * A Rise-edge B Low  ++
     */
    if(ENCRA){
        if(ENCRB)   rotateR--;
        else        rotateR++;
    }
    /*
     * A Fall-edge B High ++
     * A Fall-edge B Low  --
     */
    else{
        if(ENCRB)  rotateR++;
        else       rotateR--;
    }
    Hwi_enable();
}

void portBISR(unsigned int index)
{
    Hwi_disable();

    HWREG(GPIO_PORTB_AHB_BASE | GPIO_O_ICR) |= 0x00000020;      // clear PB5 int
    Hwi_clearInterrupt(INT_GPIOB_TM4C129);
    /*
     * B Rise-edge A High ++
     * B Rise-edge A Low  --
     */
    if(ENCRB){
        if(ENCRA)   rotateR++;
        else        rotateR--;
    }
    /*
     * B Fall-edge A High --
     * B Fall-edge A Low  ++
     */
    else{
        if(ENCRA)  rotateR--;
        else       rotateR++;
    }
    Hwi_enable();
}

void portKISR(unsigned int index)
{
    Hwi_disable();

    if(HWREG(GPIO_PORTK_BASE | GPIO_O_RIS) & 0x00000001)
    {
        HWREG(GPIO_PORTK_BASE | GPIO_O_ICR) |= 0x00000001;      // clear PK0 int
        /*
         * A Rise-edge B High --
         * A Rise-edge B Low  ++
         */
        if(ENCLA){
            if(ENCLB)   rotateL--;
            else        rotateL++;
        }
        /*
         * A Fall-edge B High ++
         * A Fall-edge B Low  --
         */
        else{
            if(ENCLB)  rotateL++;
            else       rotateL--;
        }
    }

    if(HWREG(GPIO_PORTK_BASE | GPIO_O_RIS) & 0x00000002)
    {
        HWREG(GPIO_PORTK_BASE | GPIO_O_ICR) |= 0x00000002;      // clear PK1 int
        /*
         * B Rise-edge A High ++
         * B Rise-edge A Low  --
         */
        if(ENCLB){
            if(ENCLA)   rotateL++;
            else        rotateL--;
        }
        /*
         * B Fall-edge A High --
         * B Fall-edge A Low  ++
         */
        else{
            if(ENCLA)  rotateL--;
            else       rotateL++;
        }
    }
    Hwi_clearInterrupt(INT_GPIOK_TM4C129);
    Hwi_enable();
}

/*
 *PE4 AIN9  ENCRA
 *PB5 AIN11 ENCRB
 *PK0 AIN16 ENCLA
 *PK1 AIN17 ENCLB
 *
 */
Hwi_Handle hwiPortEHandle, hwiPortBHandle, hwiPortKHandle;
Hwi_Params hwiPortEParams, hwiPortBParams, hwiPortKParams;

void WheelEncInit()
{
    HWREG(GPIO_PORTE_AHB_BASE | GPIO_O_DEN)|= 0x00000010;      //GPIO FUNCTION
    GPIO_write(DBMOUSE_LED_1, DBMOUSE_LED_OFF);
    GPIO_write(DBMOUSE_LED_2, DBMOUSE_LED_OFF);
    //PE4 ENCRA
    HWREG(GPIO_PORTE_AHB_BASE | GPIO_O_DIR)&= ~0x00000010;     //DIR INPUT
    HWREG(GPIO_PORTE_AHB_BASE | GPIO_O_AFSEL)&= ~0x00000010;   //GPIO FUNCTION
    HWREG(GPIO_PORTE_AHB_BASE | GPIO_O_DEN)|= 0x00000010;      //GPIO FUNCTION
    HWREG(GPIO_PORTE_AHB_BASE | GPIO_O_IM) &= ~0x00000010;     //mask the corresponding port
    HWREG(GPIO_PORTE_AHB_BASE | GPIO_O_IBE)|=  0x00000010;     //both edge
    HWREG(GPIO_PORTE_AHB_BASE | GPIO_O_IS) &= ~0x00000010;     //both edge

    Hwi_Params_init(&hwiPortEParams);
    hwiPortEParams.priority = 240;
    hwiPortEParams.maskSetting = Hwi_MaskingOption_ALL;
    hwiPortEHandle = Hwi_create(INT_GPIOE_TM4C129, portEISR, &hwiPortEParams, NULL);
    Hwi_clearInterrupt(INT_GPIOE_TM4C129);
    HWREG(GPIO_PORTE_AHB_BASE | GPIO_O_ICR)|= 0x00000010;     // clear
//    HWREG(GPIO_PORTE_AHB_BASE | GPIO_O_IM) |= 0x00000010;     // enable PE4 int

    //PB5 ENCRB

    HWREG(GPIO_PORTB_AHB_BASE | GPIO_O_DIR)&= ~0x00000020;     //DIR INPUT
    HWREG(GPIO_PORTB_AHB_BASE | GPIO_O_AFSEL)&= ~0x00000020;   //GPIO FUNCTION
    HWREG(GPIO_PORTB_AHB_BASE | GPIO_O_DEN)|= 0x00000020;      //GPIO FUNCTION
    HWREG(GPIO_PORTB_AHB_BASE | GPIO_O_IM) &= ~0x00000020;     //mask the corresponding port
    HWREG(GPIO_PORTB_AHB_BASE | GPIO_O_IBE)|=  0x00000020;     //both edge
    HWREG(GPIO_PORTB_AHB_BASE | GPIO_O_IS) &= ~0x00000020;     //both edge

    Hwi_Params_init(&hwiPortBParams);
    hwiPortBParams.priority = 240;
    hwiPortBParams.maskSetting = Hwi_MaskingOption_ALL;
    hwiPortBHandle = Hwi_create(INT_GPIOB_TM4C129, portBISR, &hwiPortBParams, NULL);
    Hwi_clearInterrupt(INT_GPIOB_TM4C129);
    HWREG(GPIO_PORTB_AHB_BASE | GPIO_O_ICR)|= 0x00000020;     // clear
//    HWREG(GPIO_PORTB_AHB_BASE | GPIO_O_IM) |= 0x00000020;     // enable PB5 int

    //PK0 ENCLA  PK1 ENCLB
    HWREG(GPIO_PORTK_BASE | GPIO_O_DIR)&= ~0x00000003;     //DIR INPUT
    HWREG(GPIO_PORTK_BASE | GPIO_O_AFSEL)&= ~0x00000003;   //GPIO FUNCTION
    HWREG(GPIO_PORTK_BASE | GPIO_O_DEN)|= 0x00000003;      //GPIO FUNCTION
    HWREG(GPIO_PORTK_BASE | GPIO_O_IM) &= ~0x00000003;     //mask the corresponding port
    HWREG(GPIO_PORTK_BASE | GPIO_O_IBE)|=  0x00000003;     //both edge
    HWREG(GPIO_PORTK_BASE | GPIO_O_IS) &= ~0x00000003;     //both edge

    Hwi_Params_init(&hwiPortKParams);
    hwiPortKParams.priority = 240;
    hwiPortKParams.maskSetting = Hwi_MaskingOption_ALL;
    hwiPortKHandle = Hwi_create(INT_GPIOK_TM4C129, portKISR, &hwiPortKParams, NULL);
    Hwi_clearInterrupt(INT_GPIOK_TM4C129);
    HWREG(GPIO_PORTK_BASE | GPIO_O_ICR)|= 0x000000003;     // clear
    HWREG(GPIO_PORTK_BASE | GPIO_O_IM) |= 0x000000003;     // enable PK0¡¢PK1 int
}

float WheelEncGetVel(volatile float &r, volatile float &l)
{

    r = (float)rp * EncUnit * PP::EncoderUnitCompensation;
    l = (float)lp * EncUnit * PP::EncoderUnitCompensation;
    return 0.5f * (r + l);
}
//---------------------------------------------------

