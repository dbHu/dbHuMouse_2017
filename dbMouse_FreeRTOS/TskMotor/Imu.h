/*
 * Imu.h
 *
 *  Created on: Aug 3, 2016
 *      Author: loywong
 */

#ifndef MOTORENCIMU_IMU_H_
#define MOTORENCIMU_IMU_H_

typedef struct __ImuValues
{
    float acclX, acclY, acclZ;
    float temp;
    float angvX, angvY, angvZ;
} ImuValues;

const float GyroUnit = 1.0642252e-3f;  // rad/s/LSB @ fs=2000deg/s
const float AcclUnit = 5.9855042e-4f;  // m/sq.s/LSB @ fs=2g

void ImuInit();
void ImuStartRead();
bool ImuGetValues(ImuValues &imuVal);

#endif /* MOTORENCIMU_IMU_H_ */
