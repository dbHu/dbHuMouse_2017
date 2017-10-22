/*
 * MotorEnc.cc
 *
 *  Created on: Aug 3, 2016
 *      Author: loywong
 */

#include "TskMotor/WheelEnc.h"

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_ints.h>
#include <inc/hw_gpio.h>
#include <inc/hw_timer.h>
#include <inc/hw_sysctl.h>
#include "PinConfig/pinout.h"

#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"


#define ENCRA  HWREG(GPIO_PORTE_AHB_BASE + GPIO_O_DATA) & 0x10
#define ENCRB  HWREG(GPIO_PORTB_AHB_BASE + GPIO_O_DATA) & 0x20
#define ENCLA  HWREG(GPIO_PORTK_BASE + GPIO_O_DATA) & 0x01
#define ENCLB  HWREG(GPIO_PORTK_BASE + GPIO_O_DATA) & 0x02

volatile int rotateR = 0,rotateL = 0;
int rp,lp;

SemaphoreHandle_t SemMotTick;
SemaphoreHandle_t SemIrTick;
SemaphoreHandle_t SemActTick;

int val = 0;

void portEISR(void);
void portBISR(void);
void portKISR(void);

void BaseTimerISR(void)
{
    BaseType_t rtn, Wake;

    HWREG(TIMER3_BASE | TIMER_O_ICR) |= 0x00000001;      // clear TimerA timeout int
    rtn = xSemaphoreGiveFromISR(SemMotTick, &Wake);
    configASSERT(rtn == pdPASS || rtn == errQUEUE_FULL);
    rtn = xSemaphoreGiveFromISR(SemIrTick, &Wake);
    configASSERT(rtn == pdPASS || rtn == errQUEUE_FULL);
    rtn = xSemaphoreGiveFromISR(SemActTick, &Wake);
    configASSERT(rtn == pdPASS || rtn == errQUEUE_FULL);
    lp = rotateL;
    rp = rotateR;
    rotateL = 0;
    rotateR = 0;
}

void BaseTimeInit()
{
    SemMotTick = xSemaphoreCreateBinary();
    configASSERT(SemMotTick != NULL);
    SemIrTick = xSemaphoreCreateBinary();
    configASSERT(SemIrTick != NULL);
    SemActTick = xSemaphoreCreateBinary();
    configASSERT(SemActTick != NULL);

    HWREG(SYSCTL_RCGCTIMER) |= 0x000000008;          // enable Timer3
    HWREG(TIMER3_BASE | TIMER_O_CTL) &= ~0x00000001; // GPTM Timer A clear
    HWREG(TIMER3_BASE | TIMER_O_CFG) = 0x00000000;   // 32-bit timer configuration
    HWREG(TIMER3_BASE | TIMER_O_TAMR) = 0x00000002;  // Timer A Periodic mode.
    HWREG(TIMER3_BASE | TIMER_O_TAILR) = 0x0001D4BF; // Timer A count down 120000
    HWREG(TIMER3_BASE | TIMER_O_IMR) = 0x00000001;  // Timer A timeout interrupt
    IntRegister(INT_TIMER3A_TM4C129, BaseTimerISR);
    IntEnable(INT_TIMER3A_TM4C129);
    HWREG(TIMER3_BASE | TIMER_O_CTL) = 0x00000001;  // GPTM Timer A enable
    HWREG(TIMER3_BASE | TIMER_O_ICR) = 0x00000001;  // clear TimerA timeout int
    IntMasterEnable();
}

/*
 *  ======== portEISR ========
 *  Callback function for the GPIO interrupt on Board_ENCRA.
 *  
 */

void portEISR(void)
{
    HWREG(GPIO_PORTE_AHB_BASE | GPIO_O_ICR) |= 0x00000010;      // clear PE4 int
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
}

void portBISR(void)
{
    HWREG(GPIO_PORTB_AHB_BASE | GPIO_O_ICR) |= 0x00000020;      // clear PB5 int
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
}

void portKISR(void)
{
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
}

/*
 *PE4 AIN9  ENCRA
 *PB5 AIN11 ENCRB
 *PK0 AIN16 ENCLA
 *PK1 AIN17 ENCLB
 *
 */

void WheelEncInit()
{
    HWREG(GPIO_PORTE_AHB_BASE | GPIO_O_DEN)|= 0x00000010;      //GPIO FUNCTION
//    GPIO_write(DBMOUSE_LED_1, DBMOUSE_LED_OFF);
//    GPIO_write(DBMOUSE_LED_2, DBMOUSE_LED_OFF);
    //PE4 ENCRA
    HWREG(GPIO_PORTE_AHB_BASE | GPIO_O_DIR)&= ~0x00000010;     //DIR INPUT
    HWREG(GPIO_PORTE_AHB_BASE | GPIO_O_AFSEL)&= ~0x00000010;   //GPIO FUNCTION
    HWREG(GPIO_PORTE_AHB_BASE | GPIO_O_DEN)|= 0x00000010;      //GPIO FUNCTION
    HWREG(GPIO_PORTE_AHB_BASE | GPIO_O_IM) &= ~0x00000010;     //mask the corresponding port
    HWREG(GPIO_PORTE_AHB_BASE | GPIO_O_IBE)|=  0x00000010;     //both edge
    HWREG(GPIO_PORTE_AHB_BASE | GPIO_O_IS) &= ~0x00000010;     //both edge
    IntRegister(INT_GPIOE_TM4C129, portEISR);
    IntEnable(INT_GPIOE_TM4C129);
    HWREG(GPIO_PORTE_AHB_BASE | GPIO_O_ICR)|= 0x00000010;     // clear
    HWREG(GPIO_PORTE_AHB_BASE | GPIO_O_IM) |= 0x00000010;     // enable PE4 int

    //PB5 ENCRB

    HWREG(GPIO_PORTB_AHB_BASE | GPIO_O_DIR)&= ~0x00000020;     //DIR INPUT
    HWREG(GPIO_PORTB_AHB_BASE | GPIO_O_AFSEL)&= ~0x00000020;   //GPIO FUNCTION
    HWREG(GPIO_PORTB_AHB_BASE | GPIO_O_DEN)|= 0x00000020;      //GPIO FUNCTION
    HWREG(GPIO_PORTB_AHB_BASE | GPIO_O_IM) &= ~0x00000020;     //mask the corresponding port
    HWREG(GPIO_PORTB_AHB_BASE | GPIO_O_IBE)|=  0x00000020;     //both edge
    HWREG(GPIO_PORTB_AHB_BASE | GPIO_O_IS) &= ~0x00000020;     //both edge
    IntRegister(INT_GPIOB_TM4C129, portBISR);
    IntEnable(INT_GPIOB_TM4C129);
    HWREG(GPIO_PORTB_AHB_BASE | GPIO_O_ICR)|= 0x00000020;     // clear
    HWREG(GPIO_PORTB_AHB_BASE | GPIO_O_IM) |= 0x00000020;     // enable PB5 int

    //PK0 ENCLA  PK1 ENCLB
    HWREG(GPIO_PORTK_BASE | GPIO_O_DIR)&= ~0x00000003;     //DIR INPUT
    HWREG(GPIO_PORTK_BASE | GPIO_O_AFSEL)&= ~0x00000003;   //GPIO FUNCTION
    HWREG(GPIO_PORTK_BASE | GPIO_O_DEN)|= 0x00000003;      //GPIO FUNCTION
    HWREG(GPIO_PORTK_BASE | GPIO_O_IM) &= ~0x00000003;     //mask the corresponding port
    HWREG(GPIO_PORTK_BASE | GPIO_O_IBE)|=  0x00000003;     //both edge
    HWREG(GPIO_PORTK_BASE | GPIO_O_IS) &= ~0x00000003;     //both edge
    IntRegister(INT_GPIOK_TM4C129, portKISR);
    IntEnable(INT_GPIOK_TM4C129);
    HWREG(GPIO_PORTK_BASE | GPIO_O_ICR)|= 0x000000003;     // clear
    HWREG(GPIO_PORTK_BASE | GPIO_O_IM) |= 0x000000003;     // enable PK0-PK1 int
}

float WheelEncGetVel(volatile float &r, volatile float &l)
{

    r = (float)rp * EncUnit * PP::EncoderUnitCompensation;
    l = (float)lp * EncUnit * PP::EncoderUnitCompensation;
    return 0.5f * (r + l);
}
//---------------------------------------------------

