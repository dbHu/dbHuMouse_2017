/*
 * MotorEnc.h
 *
 *  Created on: Aug 3, 2016
 *      Author: loywong
 */


#ifndef MOTORENCIMU_WHEELENC_H_
#define MOTORENCIMU_WHEELENC_H_

#include "FreeRTOS.h"
#include "semphr.h"
#include "../physparams.h"

extern SemaphoreHandle_t SemMotTick;
extern SemaphoreHandle_t SemIrTick;
extern SemaphoreHandle_t SemActTick;


const float EncRes = 28.f;
const float EncUnit = PP::RWheel * 2.f * 3.1415926536f / EncRes / PP::Ts;

void WheelEncInit();
void BaseTimeInit();
float WheelEncGetVel(volatile float &r, volatile float &l);

#endif /* MOTORENCIMU_WHEELENC_H_ */
