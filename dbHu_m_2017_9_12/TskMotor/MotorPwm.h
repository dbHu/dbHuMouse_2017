/*
 * MotorPwm.h
 *
 *  Created on: Aug 3, 2016
 *      Author: loywong
 */

#ifndef MOTORENCIMU_MOTORPWM_H_
#define MOTORENCIMU_MOTORPWM_H_

void MotorPwmCoast();

// r,l -> [-479, 479]
void MotorPwmSetDuty(short r, short l);

void MotorPwmInit();

#endif /* MOTORENCIMU_MOTORPWM_H_ */
