/*
 * motor.c
 *
 *  Created on: Aug 1, 2014
 *      Author: loywong
 */

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <stdio.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>

#include "../Pid/Pid.h"
#include "../Kalman/Kalman1Var.h"
#include <math.h>
//#include "../indicator/indicator.h"
//#include "../action/action.h"
//#include "../solve/solve.h"
//#include "../ir/ir.h"
//#include "../debug/debug.h"
#include "../physparams.h"

#include <inc/hw_sysctl.h>
#include <inc/hw_pwm.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>

#include <ti/drivers/GPIO.h>

#include "TskMotor.h"
#include "MotorPwm.h"
#include "Imu.h"
#include "WheelEnc.h"

#include "../Queue/Queue.h"
#include "../dbmouse_chassis.h"
#include "../TskTop/DbgUart.h"
#include "../TskTop/TskTop.h"

#include"action/action.h"
#include "../physparams.h"

namespace TskMotor
{

// 16ms, 0.71
const float lvPidP = 1404.04554410204f;
const float lvPidI = 26851.5069191926f;
const float lvPidD = -3.23234751122807f;
const float lvPidN = 95.3137536849951f;
//
const float avPidP = 42.3592176502007f;
const float avPidI = 792.518926283065f;
const float avPidD = -0.0609730852951907f;
const float avPidN = 192.064591913073f;
//

//const float lvPidP = 600.f;					//100  150  200  500  600  600  600
//const float lvPidI = 3000.f;                //1000 2000 1000 1000 1200 2000 3000
//const float lvPidD = 0.f;
//const float lvPidN = 0.f;

//const float avPidP = 60.f;
//const float avPidI = 600.f;
//const float avPidD = 0.f;
//const float avPidN = 0.f;

const float dLvDefault = 0.0f;
const float dAvDefault = 0.0f;
//
//const float gyro_gain = (70e-3f * 3.1415927f / 180.0f / GYRO_UNIT_COMPENSATION);
//const float accl_gain = (598.55e-6f / ACCL_UNIT_COMPENSATION);
//const float enc_unit  = (14.23534171e-3f / ENCODER_UNIT_COMPENSATION);
//const float wheel_dist = 64.0e-3f;
//
//const unsigned char GYRO_RD_ADDR = 0xD7
//const unsigned char GYRO_WR_ADDR = 0xD6
//const unsigned char ACCM_RD_ADDR = 0x3B
//const unsigned char ACCM_WR_ADDR = 0x3A

volatile float EncRVel = 0;
volatile float EncLVel = 0;
volatile float EncVel; // avg velocity from 2 encoders
volatile float AcclY, GyroZ; // y-axis acceleration & z-axis angular velocity from IMU
volatile float DistanceAcc = 0.0f,AngleAcc = 0.0f,DistanceAcc_en = 0.f;
volatile float CurrentV = 0.0f;
volatile float OmgAdj = 0.0f,LvAdj = 0.0f;    // omega correction from TskAction
volatile float GyroZZero = 0.0f, AcclYZero = 0;
volatile float LV, AV;   // LV & AV feedback for PIDs
volatile float Temperature = 0.0f;

bool MotorEnabled = false;
bool GyroEnabled = false;
bool AcclEnabled = false;
bool AcqZerosEnabled = false;

const int tskPrio = 12;
const int tskStkSize = 1536;
Task_Handle tsk;
//Error_Block eb;

Mailbox_Handle MbCmd;

Queue<VelOmega> *QMotor;

inline float saturate(float v, float max, float min)
{
    return  v > max ? max :
            v < min ? min : v;
}

void task(UArg arg0, UArg arg1)
{
    MotorMsg::MsgType msg;
//    INT32S cnt = 0;//, tick;

    ImuValues imuVals;

    // vars for zero acq
    int staticCnt = 0, staticCntCyced;//,i = 0;
    float gyroZZeroAcc = 0;
    float acclYZeroAcc = 0;

    VelOmega desire(dLvDefault, dAvDefault);    // desired lv & av from queue
    float lvPidOut, avPidOut;
    float lPwm, rPwm;
    Int16 leftPwm, rightPwm;

    Pid lvPid(lvPidP, lvPidI, lvPidD, lvPidN, PP::Ts, -319.f, 319.f);
    Pid avPid(avPidP, avPidI, avPidD, avPidN, PP::Ts, -319.f, 319.f);
    Kalman1Var lvEstKal(
            1.0f,   // A
            PP::Ts, // B
            1.0f,   // H
            //                  rms noise         quantify noise            drift?  enlarge to anti drift
            PP::Ts * PP::Ts * (0.00307745f + AcclUnit * AcclUnit / 12.f + 0.117649f * 256.f),   // Q
            EncUnit * EncUnit / 12.f,   // R
            0.f,    // x0
            0.f     // P0
            );

    // initial motor pwm
    MotorPwmInit();
    // initial qei
    WheelEncInit();
    // initial imu
    ImuInit();
    // imu start 1st read
    ImuStartRead();
    Task_sleep(2);

    ////////////////////////////////
//    int iii = 0;
//    char sss[20];
    ////////////////////////////////

    while(1)
    {
//        GPIO_write(DBMOUSE_LED_0, DBMOUSE_LED_OFF);
        if(!Semaphore_pend(SemMotTick, 2))
            System_abort("pend SemMotTick failed!\n");
//        GPIO_write(DBMOUSE_LED_0, DBMOUSE_LED_ON);

        // read encder
        EncVel = WheelEncGetVel(EncRVel, EncLVel);

        // read gyro, accm
        if(!ImuGetValues(imuVals))
            System_abort("ImuGetValues failed!\n");

        // start next read
        ImuStartRead();

        AcclY = imuVals.acclY - AcclYZero;
        GyroZ = imuVals.angvZ - GyroZZero;

//////////////////////////////////////////////////
//        if(iii < 499)
//            iii++;
//        else
//        {
//            iii = 0;
//            sprintf(sss, "%4f\t\t%4f\n", AcclY, GyroZ);
//            System_printf(sss);
//            System_flush();
//        }
//////////////////////////////////////////////////

        if(AcclEnabled)
        {
            lvEstKal.Predict(AcclY);
            LV = lvEstKal.Correct(EncVel);
        }
        else
        {
            LV = EncVel;
        }

        if(GyroEnabled)
        {
            AV = GyroZ;
        }
        else
        {
            AV = (EncRVel - EncLVel) * (.5f / PP::W);
        }

        DistanceAcc += LV * PP::Ts;
        DistanceAcc_en +=  EncVel * PP::Ts;
        AngleAcc += AV * PP::Ts;
        if(MotorEnabled)
        {

            // read dlv&dav from fifo
            QMotor->De(desire); // dequeue, if empty desire will not change
            if(desire.Omega)
			{
//            	if(i > 511)
//				{
//				i=0;
//				TskAction::Info_LV[i] = desire.Velocity;
//            	i++;
//				}

					//TskAction::info.inf[i] = desire.Velocity;
					//TskAction::info.cnt = i;
			}


            CurrentV = desire.Velocity + LvAdj;
            lvPidOut = lvPid.Tick(desire.Velocity + LvAdj - LV);
            avPidOut = avPid.Tick(desire.Omega + OmgAdj - AV);
            rPwm = (lvPidOut + avPidOut);
            lPwm = (lvPidOut - avPidOut);
            rightPwm = (int)saturate(rPwm, 318.f, -318.f);
            leftPwm = (int)saturate(lPwm, 318.f, -318.f);
            if(staticCnt > 63)
                MotorPwmSetDuty(0, 0);
            else{
                MotorPwmSetDuty(rightPwm, leftPwm);
            }
        }

        // imu adj zeros, auto adj zeros if static 0.64s
        if(AcqZerosEnabled && desire.Velocity + LvAdj == 0.0f && desire.Omega + OmgAdj == 0.0f)
        {
            staticCnt++;
            staticCntCyced = (staticCnt & 0x1FFF);  // cyced per 8.192s(@Ts=1ms)
            if(staticCntCyced == 127)               // desire static 127ms(@Ts=1ms), adj zero start
            {
                gyroZZeroAcc = 0;
                acclYZeroAcc = 0;
                //TskTop::SetLeds(0x5);
            }
            else if(staticCntCyced >= 128 && staticCntCyced < 640)
            {
                gyroZZeroAcc += imuVals.angvZ;
                acclYZeroAcc += imuVals.acclY;
            }
            else if(staticCntCyced == 640)   // adj zero finish
            {
                GyroZZero = gyroZZeroAcc * (1.f / 512.f);
                AcclYZero = acclYZeroAcc * (1.f / 512.f);
                //TskTop::SetLeds(0xA);
//                    DbgUartPutLine("ImuAcq0s.\n", true);
//                    err = OSQPost(IndicQ, (void *)INDIC_MSG_FINISH);
#if DBG_PRINT_IMU_ZEROS > 0
                sprintf(motorDbgString, "Imu Zeros, G: %5d; AX: %5d; AY: %5d; AZ: %5d\n", gyroZero, accxZero, accyZero, acczZero);
                //err = DbgPrintString(&motorDbgInfo, motorDbgString);
                DbgPuts(motorDbgString);
#endif
            }
        }
        else
            staticCnt = 0;

        if(Mailbox_pend(MbCmd, &msg, BIOS_NO_WAIT))
        {
            switch(msg & 0xFFFF0000)
            {
            case MotorMsg::EnableMotors:
                QMotor->Clear();
                MotorPwmSetDuty(0, 0);
                MotorEnabled = true;
                break;
            case MotorMsg::DisableMotors:
                MotorPwmCoast();
                lvPid.Reset();
                avPid.Reset();
                MotorEnabled = false;
                break;
//            case MotorMsg::GetZeros:
//                alt_ic_irq_disable(ENC_MOTOR_IRQ_INTERRUPT_CONTROLLER_ID, ENC_MOTOR_IRQ);
//                err = OSQPost(IndicQ, (void *)INDIC_MSG_GETZEROS);
//                getImuZeros(&imuiic);
////                err = OSQPost(IndicQ, (void *)INDIC_MSG_FINISH);
//#if DBG_PRINT_IMU_ZEROS > 0
//                sprintf(motorDbgString, "Imu Zeros, G: %5d; AX: %5d; AY: %5d; AY: %5d\n", gyroZero, accxZero, accyZero, acczZero);
//                //err = DbgPrintString(&motorDbgInfo, motorDbgString);
//                DbgPuts(motorDbgString);
//#endif
//                alt_ic_irq_enable(ENC_MOTOR_IRQ_INTERRUPT_CONTROLLER_ID, ENC_MOTOR_IRQ);
//                break;
            case MotorMsg::EnableGyro:
                GyroEnabled = true;
                break;
            case MotorMsg::DisableGyro:
                GyroEnabled = false;
                break;
            case MotorMsg::EnableAccl:
                AcclEnabled = true;
                break;
            case MotorMsg::DisableAccl:
                AcclEnabled = false;
                break;
            case MotorMsg::EnableAcqZeros:
                AcqZerosEnabled = true;
                break;
            case MotorMsg::DisableAcqZeros:
                AcqZerosEnabled = false;
                break;
            default:
                break;
            }
        }
//////test
//        if(cnt == 0)
//        {
//            //printf("%f, %f, %f, %d\n", LV, (70e-3f * 3.1415927f / 180.0f) * (gyro), (70e-3f * 3.1415927f / 180.0f) * (gyroZero), accy);
//            //printf("%f, %f, %d\n", LV, (70e-3f * 3.1415927f / 180.0f) * (gyro - gyroZero), accy);
//            printf("%d, %d\n", tick, OSTimeGet());
//        }
//        if(cnt < 249) cnt++;
//        else cnt = 0;
//////end test
    }
}

void Init()
{
    Task_Params tskParams;

    MbCmd = Mailbox_create(4, 4, NULL, NULL);
    if(MbCmd == NULL)
        System_abort("create TskMotor::MbCmd failed.\n");

    QMotor = new Queue<VelOmega>(512);

//    Error_init(&eb);
    Task_Params_init(&tskParams);
    tskParams.priority = tskPrio;
    tskParams.stackSize = tskStkSize;
    tsk = Task_create(task, &tskParams, NULL);

    if(tsk == NULL)
        System_abort("TskMotor failed.");

}

}
