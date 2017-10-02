/*
 * MotorEnc.cc
 *
 *  Created on: Aug 3, 2016
 *      Author: loywong
 */

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/hal/Timer.h>

#include <xdc/runtime/System.h>

#include <inc/hw_qei.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_ints.h>
#include <inc/hw_sysctl.h>
#include <inc/hw_gpio.h>

#include "../dbmouse_chassis.h"
#include "WheelEnc.h"
#include "../Board.h"
#include "../physparams.h"

#include <ti/drivers/GPIO.h>

#define ENCRA  HWREG(GPIO_PORTE_AHB_BASE + GPIO_O_DATA) & 0x10
#define ENCRB  HWREG(GPIO_PORTB_AHB_BASE + GPIO_O_DATA) & 0x20
#define ENCLA  HWREG(GPIO_PORTK_BASE + GPIO_O_DATA) & 0x01
#define ENCLB  HWREG(GPIO_PORTK_BASE + GPIO_O_DATA) & 0x02

volatile int rotateR = 0,rotateL = 0;
ixnt rp,lp;
3
Semaphore_Handle SemMotTick;
Semaphore_Handle SemIrTick;
Semaphore_Handle SemActTick;

const short BaseTime = 1000;    // unit : microSec
Timer_Handle BaseTimer;
Timer_Params BaseTimerParams;
const int BaseTimerId = 3;//Timer_ANY;

void BaseTimerHooker(UArg arg)
{
    Semaphore_post(SemMotTick);
    Semaphore_post(SemIrTick);
    Semaphore_post(SemActTick);
    lp = rotateR;
    rp = rotateL;
    rotateL = 0;
    rotateR = 0;
}

//void WheelEncInit()
void BaseTimeInit()
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

    Timer_Params_init(&BaseTimerParams);
    BaseTimerParams.period = BaseTime;
    BaseTimerParams.runMode = Timer_RunMode_CONTINUOUS;//Timer_RunMode_DYNAMIC;
    BaseTimerParams.startMode = Timer_StartMode_AUTO;
    BaseTimer = Timer_create(BaseTimerId, BaseTimerHooker, &BaseTimerParams, NULL);

    if(BaseTimer == NULL)
        System_abort("create irTimer failed!\n");
}

/*
 *  ======== gpioFxn0 ========
 *  Callback function for the GPIO interrupt on Board_ENCRA.
 *  
 */

void gpioFxn0(unsigned int index)
{
    /*
     * A Rise-edge B High ++
     * A Rise-edge B Low  --
     */
    if(ENCRA){
        if(ENCRB)   rotateR++;
        else        rotateR--;
    }
    /*
     * A Fall-edge B High --
     * A Fall-edge B Low  ++
     */
    else{
        if(ENCRB)  rotateR--;
        else       rotateR++;  
    }
}

void gpioFxn1(unsigned int index)
{
    /*
     * B Rise-edge A High --
     * B Rise-edge A Low  ++
     */
    if(ENCRB){
        if(ENCRA)   rotateR--;
        else        rotateR++;
    }
    /*
     * B Fall-edge A High ++
     * B Fall-edge A Low  --
     */
    else{
        if(ENCRA)  rotateR++;
        else       rotateR--;  
    }
}

void gpioFxn2(unsigned int index)
{
    /*
     * A Rise-edge B High ++
     * A Rise-edge B Low  --
     */
    if(ENCLA){
        if(ENCLB)   rotateL++;
        else        rotateL--;
    }
    /*
     * A Fall-edge B High --
     * A Fall-edge B Low  ++
     */
    else{
        if(ENCLB)  rotateL--;
        else       rotateL++;
    }
}

void gpioFxn3(unsigned int index)
{
    /*
     * B Rise-edge A High --
     * B Rise-edge A Low  ++
     */
    if(ENCLB){
        if(ENCLA)   rotateL--;
        else        rotateL++;
    }
    /*
     * B Fall-edge A High ++
     * B Fall-edge A Low  --
     */
    else{
        if(ENCLA)  rotateL++;
        else       rotateL--;
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
    /* install ENCRA callback */
    GPIO_setCallback(Board_ENCRA, gpioFxn0);

    /* Enable interrupts */
    GPIO_enableInt(Board_ENCRA);

    /* install ENCRA callback */
    GPIO_setCallback(Board_ENCRB, gpioFxn1);

    /* Enable interrupts */
    GPIO_enableInt(Board_ENCRB);

    /* install ENCLA callback */
    GPIO_setCallback(Board_ENCLA, gpioFxn2);

    /* Enable interrupts */
    GPIO_enableInt(Board_ENCLA);

    /* install ENCLB callback */
    GPIO_setCallback(Board_ENCLB, gpioFxn3);

    /* Enable interrupts */
    GPIO_enableInt(Board_ENCLB);
}

float WheelEncGetVel(volatile float &r, volatile float &l)
{

    r = (float)rp * EncUnit * CP::EncoderUnitCompensation;
    l = (float)lp * EncUnit * CP::EncoderUnitCompensation;
    return 0.5f * (r + l);
}
//---------------------------------------------------

