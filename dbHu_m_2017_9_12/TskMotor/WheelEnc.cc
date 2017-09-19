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
#include "../physparams.h"

#include <ti/drivers/GPIO.h>

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

