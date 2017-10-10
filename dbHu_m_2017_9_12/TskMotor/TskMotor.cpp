/*
 * motor.c
 *
 *  Created on: Aug 1, 2014
 *      Author: loywong
 */

#include <xdc/std.h>
#include <xdc/runtime/System.h>

#include <stdlib.h>
#include <math.h>

#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/BIOS.h>
#include <ti/drivers/GPIO.h>

#include "../Kalman/Kalman2Var.h"
#include "TskMotor/WheelEnc.h"
#include "../TskMotor/MotorPwm.h"
#include "Pid/pid.h"
#include "Board.h"
#include "TskTop/DbgUart.h"
#include "../TskMotor/Imu.h"
#include "../TskMotor/TskMotor.h"
#include "TskTop/TskTop.h"

namespace TskMotor
{

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
volatile float AcclX, GyroZ; // x-axis acceleration & z-axis angular velocity from IMU
volatile float DistanceAcc = 0.0f,AngleAcc = 0.0f,DistanceAcc_en = 0.f,DesireDistance = 0.f;
volatile float CurrentV = 0.0f;
volatile float OmgAdj = 0.0f,LvAdj = 0.0f;    // omega correction from TskAction
volatile float GyroZZero = 0.0f, AcclXZero = 0;
volatile float LV, AV;   // LV & AV feedback for PIDs
volatile float Temperature = 0.0f;

bool MotorEnabled = false;
bool GyroEnabled = false;
bool AcclEnabled = false;
bool AcqZerosEnabled = false;

const int tskPrio = 12;
const int tskStkSize = 2048;
Task_Handle tsk;
//Error_Block eb;

Mailbox_Handle MbCmd;

Queue<VelOmega> *QMotor;

inline float saturate(float v, float max, float min)
{
    return  v > max ? max :
            v < min ? min : v;
}

VelOmega desire(dLvDefault, dAvDefault);    // desired lv & av from queue

PidParam pidparam;

Matrix2x2  A(1.f, PP::Ts, 0.f, 1.f);
Vector2    B(1/2.f * 1e-6f, 1e-3f);
Matrix2x2  H(1.f, 0.f, 0.f, 1.f);
Matrix2x2  Q(1e-12f * (1.25f * 1e-5f + 0.34f)/ 4.f, 
             1e-9f * (1.25f * 1e-5f + 0.34f) / 2.f, 
             1e-9f * (1.25f * 1e-5f + 0.34f) / 2.f, 
             1e-6f * (1.25f * 1e-5f + 0.34f));
Matrix2x2  R(1e-6f * 0.16654957f, 0, 0, 0.16654957f);
Vector2    x0;
Matrix2x2  P0;
Vector2    kalOut;

void task(UArg arg0, UArg arg1)
{
    MotorMsg::MsgType msg;
//    INT32S cnt = 0;//, tick;

    ImuValues imuVals;

    // vars for zero acq
    int staticCnt = 0, staticCntCyced,i = 0, pwmCnt = 0;
    float gyroZZeroAcc = 0;
    float acclXZeroAcc = 0;

    float lvPidOut, avPidOut;
    float lPwm, rPwm;
    Int16 leftPwm, rightPwm;
    Kalman2Var lvEstKal(A,B,H,Q,R,x0,P0);
//    Pid lvPid(pidparam.lvPidP, pidparam.lvPidI, pidparam.lvPidD, pidparam.lvPidN, PP::Ts, -479.f, 479.f);
    Pid avPid(pidparam.avPidP, pidparam.avPidI, pidparam.avPidD, pidparam.avPidN, PP::Ts, -479.f, 479.f);

    // initial motor pwm
    MotorPwmInit();

    // initial BaseTime
    BaseTimeInit();

    Task_sleep(2);
    
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

        //TODO
        // read encder
        EncVel = WheelEncGetVel(EncRVel, EncLVel);

        // read gyro, accm
        if(!ImuGetValues(imuVals))
            System_abort("ImuGetValues failed!\n");

        // start next read
        ImuStartRead();

        AcclX = imuVals.acclX - AcclXZero;
        GyroZ = imuVals.angvZ - GyroZZero;


        if(pwmCnt > 100)
        {
            MotorPwmCoast();
//            lvPid.Reset();
            avPid.Reset();
            MotorEnabled = false;
            GPIO_write(DBMOUSE_LED_0, DBMOUSE_LED_ON);
            DbgUartPutLine("error\r\n");
            Task_sleep(5000);
        }
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
            lvEstKal.Predict(AcclX);
            kalOut = lvEstKal.Correct(Vector2(DistanceAcc_en,EncVel));
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

        DistanceAcc = kalOut.elem[0];
        DistanceAcc_en +=  EncVel * PP::Ts;
        AngleAcc += AV * PP::Ts;
#if 0
        MotorPwmSetDuty(80, 80);
#endif
        if(MotorEnabled)
        {
            // read dlv&dav from fifo
            QMotor->De(desire); // dequeue, if empty desire will not change
//            if(TskTop::info_flag == 2)
//                desire.Omega = 10.f;
//            else if(TskTop::info_flag == 1) desire.Velocity = 0.2f;
//            else {
//                desire.Omega = 0.f;
//                desire.Velocity = 0.f;
//            }
            if(desire.Omega > 0.f || desire.Velocity > 0.f)
			{
//					TskAction::info.inf[i] = desire.Velocity;
					//TskAction::info.cnt = i;
                if(i < 512)
                {
                    if(TskTop::info_flag == 2)
                    {
                        TskAction::Info[i] = AV;
                        TskAction::Desire[i] = desire.Omega;
                    }
                    else if(TskTop::info_flag == 1)
                    {
                        TskAction::Info[i] = kalOut.elem[0];
                        TskAction::Desire[i] = DesireDistance;
                    }
                    i++;
                }
			}

            CurrentV = desire.Velocity + LvAdj;
            DesireDistance += CurrentV * PP::Ts;
            LV = (DesireDistance - kalOut.elem[0]) * pidparam.posCoff + (CurrentV - kalOut.elem[1]) * pidparam.velCoff;
            lvPidOut = LV *480.f / 3.7f;
            avPidOut = avPid.Tick(desire.Omega + OmgAdj - AV);
            rPwm = (lvPidOut + avPidOut);
            lPwm = (lvPidOut - avPidOut);
            rightPwm = (int)saturate(rPwm, 478.f, -478.f);
            leftPwm = (int)saturate(lPwm, 478.f, -478.f);
            if(fabs(rightPwm) > 320.f || fabs(leftPwm) > 320.f || fabs(imuVals.acclY) > 8.f)
            {
            	pwmCnt++;
            }
            else pwmCnt = 0;
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
                acclXZeroAcc = 0;
                //TskTop::SetLeds(0x5);
            }
            else if(staticCntCyced >= 128 && staticCntCyced < 640)
            {
                gyroZZeroAcc += imuVals.angvZ;
                acclXZeroAcc += imuVals.acclX;
            }
            else if(staticCntCyced == 640)   // adj zero finish
            {
                GyroZZero = gyroZZeroAcc * (1.f / 512.f);
                AcclXZero = acclXZeroAcc * (1.f / 512.f);
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
//                lvPid.Reset();
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

    QMotor = new Queue<VelOmega>(650);

//    Error_init(&eb);
    Task_Params_init(&tskParams);
    tskParams.priority = tskPrio;
    tskParams.stackSize = tskStkSize;
    tsk = Task_create(task, &tskParams, NULL);

    if(tsk == NULL)
        System_abort("TskMotor failed.");

}

}
