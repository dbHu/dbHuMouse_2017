/*
 * MotorEnc.h
 *
 *  Created on: Aug 3, 2016
 *      Author: loywong
 */


#ifndef MOTORENCIMU_WHEELENC_H_
#define MOTORENCIMU_WHEELENC_H_

#include <ti/sysbios/knl/Semaphore.h>
#include "../physparams.h"

extern Semaphore_Handle SemMotTick;
extern Semaphore_Handle SemIrTick;
extern Semaphore_Handle SemActTick;


const float EncRes = 28.f;
const float EncUnit = PP::RWheel * 2.f * 3.1415926536f / EncRes / PP::Ts;

void WheelEncInit();
void BaseTimeInit();
float WheelEncGetVel(volatile float &r, volatile float &l);

#endif /* MOTORENCIMU_WHEELENC_H_ */
