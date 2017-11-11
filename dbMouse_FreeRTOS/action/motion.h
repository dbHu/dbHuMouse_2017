/*
 * motion.h
 *
 *  Created on: Aug 5, 2014
 *      Author: loywong
 */

#ifndef MOTION_H_
#define MOTION_H_

#include "../physparams.h"
#include "FreeRTOS.h"
#include "queue.h"

//#ifdef __cplusplus
//extern "C"
//{
//#endif /* __cplusplus */


namespace TskAction{

int MotionSeqLen(void);
void MotionSeqFlush(void);
void MotionSeqWrite(float vel, float omg);
int MotionCalcFwd(float v0, float v1, float s, float *vs, float *accl);
int MotionCalcRotate(float ang, float mu, float *omgs);
int MotionCalcTurn(float v, float ang, float mu, float *omgs, float *requ);
void Init();
}

//#ifdef __cplusplus
//}
//#endif /* __cplusplus */

#endif /* MOTION_H_ */
