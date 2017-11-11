/*
 * motor.h
 *
 *  Created on: Aug 1, 2014
 *      Author: loywong
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "Queue/Queue.h"
#include "Physparams.h"

namespace TskMotor{

// task motor: motor, encoder & imu acq

extern volatile float OmgAdj,LvAdj;    // omega correction from TskCorrection

class MotorMsg
{
public:
    enum MsgType
	{
		EnableMotors    = 0x00010000,
		DisableMotors   = 0x00020000,
//		GetZeros        = 0x00030000,
		EnableGyro      = 0x00040000,
		DisableGyro     = 0x00050000,
		EnableAccl      = 0x00060000,
		DisableAccl     = 0x00070000,
		EnableAcqZeros  = 0x00080000,
		DisableAcqZeros = 0x00090000
	};
};

struct VelOmega
{
    float Velocity;
    float Omega;
    VelOmega(float vel = 0.f, float omg = 0.f)
    {
        Velocity = vel;
        Omega = omg;
    }
//    VelOmega & operator=(const VelOmega &r)
//    {
//        this->Velocity = r.Velocity;
//        this->Omega = r.Omega;
//        return *this;
//    }
};
extern volatile float EncLVel;
extern volatile float EncRVel;
extern volatile float EncVel; // avg velocity from 2 encoders
extern volatile float GyroZZero, AcclXZero;
extern volatile float AcclX, GyroZ; // y-axis acceleration & z-axis angular velocity from IMU
extern volatile float LV, AV;   // lv & av feedback for PIDs
extern volatile float DistanceAcc,AngleAcc,DistanceAcc_en,DesireDistance;
extern volatile float CurrentV;	// Current Desired Linear Velocity
extern float dist_en[240], vel_de[240], lv[240];
extern QueueHandle_t MbCmd;
extern Queue<VelOmega, true> *QMotor;
extern VelOmega desire;    // desired lv & av from queue
void Init();

extern PidParam pidparam;
}


#endif /* MOTOR_H_ */
