/*
 * Imu.h
 *
 *  Created on: Aug 3, 2016
 *      Author: loywong
 */

#include "../physparams.h"

#ifndef MOTORENCIMU_IMU_H_
#define MOTORENCIMU_IMU_H_

const float GyroUnit = 1.0642252e-3f * CP::GyroUnitCompensation;  // rad/s/LSB @ fs=2000deg/s
const float AcclUnit = 5.9855042e-4f * CP::AcclUnitCompensation;  // m/sq.s/LSB @ fs=2g

typedef struct __ImuValues
{
    float acclX, acclY, acclZ;
    float temp;
    float angvX, angvY, angvZ;
} ImuValues;

void ImuInit();
void ImuStartRead();
bool ImuGetValues(ImuValues &imuVal);

#endif /* MOTORENCIMU_IMU_H_ */
