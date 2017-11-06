/*
 * top.cc
 *
 *  Created on: Jul 21, 2016
 *      Author: loywong
 */
#include "TskTop.h"
#include "TskTop/DbgUart.h"
#include "TskMotor/TskMotor.h"
#include "PinConfig/pinout.h"
#include "TskIr/TskIr.h"
#include "TskIr/IrCorr.h"
#include "TskTop/cmd.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "physparams.h"
#include "action/action.h"
#include "solve/solve.h"

namespace TskTop{
const int tskPrio = 4;
const int tskStkSize = 1024;
QueueHandle_t MbCmd;

char dbgStr[128];

volatile MouseMode::ModeType Mode;

void doIrCorrection()
{
    bool rtn;
    sprintf(dbgStr, "Starting Ir Correction precedure. Wait seconds plz...\n");
    rtn = xQueuePost(TskPrint::MbCmd, dbgStr, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
    vTaskDelay(50);

    TskMotor::MotorMsg::MsgType motMsg;
    TskIr::IrMsg::MsgType irMsg;

    vTaskDelay(1000);

    motMsg = TskMotor::MotorMsg::DisableMotors;
    rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
    irMsg = TskIr::IrMsg::DisableEmitt;
    rtn = xQueuePost(TskIr::MbCmd, &irMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);

    /*
     *use encoder as the condition to get ir data
     */
    motMsg = TskMotor::MotorMsg::EnableAcqZeros;
    rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);

    vTaskDelay(1000);   // wait for getting imu zeros & ir zeros

    motMsg = TskMotor::MotorMsg::DisableAcqZeros;
    rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);

    irMsg = TskIr::IrMsg::EnableEmitt;
    rtn = xQueuePost(TskIr::MbCmd, &irMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);

    vTaskDelay(200);

    TskIr::doIrCorrection();
}

void encImuMonitor()
{
    int i = 0;
    bool rtn;
    TskMotor::MotorMsg::MsgType motMsg;
    TskIr::IrMsg::MsgType irMsg;

    motMsg = TskMotor::MotorMsg::DisableMotors;
    rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
    irMsg = TskIr::IrMsg::DisableEmitt;
    rtn = xQueuePost(TskIr::MbCmd, &irMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
    motMsg = TskMotor::MotorMsg::EnableAcqZeros;
    rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);

    vTaskDelay(1000);// wait for getting imu zeros & getting ir zeros

    motMsg = TskMotor::MotorMsg::EnableAccl;
    rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
    motMsg = TskMotor::MotorMsg::EnableGyro;
    rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
    irMsg = TskIr::IrMsg::EnableEmitt;
    rtn = xQueuePost(TskIr::MbCmd, &irMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);

    while(true)
    {
        if(!(i++ & 0xf))
        {
            sprintf(dbgStr, "\nGyroZ AXZero AcclX EncVel KalVel DisAcc AngAcc\r\n");
            rtn = xQueuePost(TskPrint::MbCmd, dbgStr, portMAX_DELAY);
            configASSERT(rtn == pdPASS);
            vTaskDelay(50);
        }
        sprintf(dbgStr, "%7.3f,%7.3f,%7.3f,%7.3f,%7.4f,%7.4f,%8.3f,%8.3f\r\n",
                TskMotor::GyroZZero, TskMotor::AV,
                TskMotor::AcclXZero, TskMotor::AcclX,
                TskMotor::EncVel, TskMotor::LV,
                TskMotor::DistanceAcc,
                //TskMotor::DistanceAcc_en,
                TskMotor::AngleAcc);
        rtn = xQueuePost(TskPrint::MbCmd, dbgStr, portMAX_DELAY);
        configASSERT(rtn == pdPASS);
        vTaskDelay(500);
        if(TskIr::TestIrTouch(TskIr::IrCh::FL | TskIr::IrCh::FR, 1600, 1200))
            break;
    }

    Mode = MouseMode::Idle;
}

void irMonitor()
{
    int i=0;
    bool rtn;
    TskMotor::MotorMsg::MsgType motMsg;
    TskIr::IrMsg::MsgType irMsg;

    vTaskDelay(100);

    motMsg = TskMotor::MotorMsg::DisableMotors;
    rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);

    irMsg = TskIr::IrMsg::DisableEmitt;
    rtn = xQueuePost(TskIr::MbCmd, &irMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);

    vTaskDelay(100);

    irMsg = TskIr::IrMsg::EnableEmitt;
    rtn = xQueuePost(TskIr::MbCmd, &irMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);

    while(true)
    {
       if(!(i++ & 0xf))
       {
           sprintf(dbgStr, "\tAve:b\t  LFwd:b\t   RFwd:b\t  LSide:b\t  RSide:b\t  HdbyL\t  HdbyR\tHdbyFLR\r\n");
           rtn = xQueuePost(TskPrint::MbCmd, dbgStr, portMAX_DELAY);
           configASSERT(rtn == pdPASS);
           vTaskDelay(50);
       }
       sprintf(dbgStr, "%7.4f:%1d\t%7.4f:%1d\t%7.4f:%1d",
               TskIr::irDistFwd(), TskIr::IrBins.Fwd,
               TskIr::IrDists.FLns, TskIr::IrBins.FLns,
               TskIr::IrDists.FRns, TskIr::IrBins.FRns
       );
       rtn = xQueuePost(TskPrint::MbCmd, dbgStr, portMAX_DELAY);
       configASSERT(rtn == pdPASS);
       vTaskDelay(50);

       sprintf(dbgStr, "%7.4f:%1d\t%7.4f:%1d\t%7.1f\t%7.1f\t%7.1f\r\n",
               TskIr::IrDists.LS, TskIr::IrBins.LS,
               TskIr::IrDists.RS, TskIr::IrBins.RS,
               TskIr::IrYaw.byLS * 180.f / 3.1415927f,
               TskIr::IrYaw.byRS * 180.f / 3.1415927f,
               TskIr::IrYaw.byFLR * 180.f / 3.1415927f
               // TskIr::IrInts.sl,
               // TskIr::IrInts.sr
        );
//        sprintf(dbgStr, "%4d,%6.4f,%4d,%6.4f,%4d,%6.4f,%4d,%6.4f\r\n",
//             TskIr::IrInts.fl, TskIr::IrDists.FLns,
//             TskIr::IrInts.fr, TskIr::IrDists.FRns,
//             TskIr::IrInts.sl, TskIr::IrDists.LS,
//             TskIr::IrInts.sr, TskIr::IrDists.RS
//        );
        rtn = xQueuePost(TskPrint::MbCmd, dbgStr, portMAX_DELAY);
        configASSERT(rtn == pdPASS);

        vTaskDelay(500);
        if(TskIr::TestIrTouch(TskIr::IrCh::SL | TskIr::IrCh::SR, 2200, 1900))
            break;
    }

    Mode = MouseMode::Idle;
}

void uartCmd()
{
    cmd_shell();
}

void actionTest(void)
{
    bool rtn;
    TskTop::SetLeds(0x7);
    TskAction::Act::ActType actMsg;
    TskAction::ActMsg::MsgType end_Msg;

    while(true)
    {
        TskTop::SetLeds(0x2);
        MotorStart();
        TskIr::WaitIrTouch(TskIr::IrCh::FL | TskIr::IrCh::FR, 1600, 1200);
        {
            actMsg = (TskAction::Act::ActType)(TskAction::Act::Start);
            rtn = xQueuePost(TskAction::MbCmd, &actMsg, portMAX_DELAY);
            configASSERT(rtn == pdPASS);
            xQueuePend(TskTop::MbCmd, &end_Msg, portMAX_DELAY);
            configASSERT(rtn == pdPASS);

            actMsg = (TskAction::Act::ActType)(TskAction::Act::Stop);
            rtn = xQueuePost(TskAction::MbCmd, &actMsg, portMAX_DELAY);
            configASSERT(rtn == pdPASS);
            rtn = xQueuePend(TskTop::MbCmd, &end_Msg, portMAX_DELAY);
            configASSERT(rtn == pdPASS);

            vTaskDelay(50);
            MotorStop();
        }
        for(int i = 0; i < 240; i++){
            sprintf(dbgStr, "%7.5f, %7.5f, %7.5f, %7.5f, %7.5f\n",
                    TskAction::Info[i],
                    TskAction::Desire[i],
                    TskMotor::dist_en[i],
                    TskMotor::vel_de[i],
                    TskMotor::lv[i]);
            rtn = xQueuePost(TskPrint::MbCmd,dbgStr, (TickType_t)0);
            configASSERT(rtn == pdPASS);
            vTaskDelay(50);
        }
        break;
    }
}

void solveTest()
{
    bool rtn;
    TskMotor::MotorMsg::MsgType motMsg;
    solve::Solve::SolveType msg;

    SetLeds(0x00);

    while(true){
        if(TskIr::TestIrTouch(TskIr::IrCh::FL, 1600, 1200))
        {
            SetLeds(0x0f);
            msg = (solve::Solve::SolveType)(solve::Solve::ALGOTEST | solve::Solve::Finish);
            break;
        }
        else if(TskIr::TestIrTouch(TskIr::IrCh::FR, 1600, 1200))
        {
            SetLeds(0x01);
            msg = solve::Solve::ALGOTEST;
            break;
        }
    }
    vTaskDelay(1000);

    motMsg = TskMotor::MotorMsg::DisableMotors;
    rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
    motMsg = TskMotor::MotorMsg::EnableAcqZeros;
    rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);

    vTaskDelay(1000);// wait for getting imu zeros & getting ir zeros

//  motMsg = TskMotor::MotorMsg::DisableAcqZeros;
//  rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    motMsg = TskMotor::MotorMsg::EnableAccl;
    rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
    motMsg = TskMotor::MotorMsg::EnableMotors;
    rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
    motMsg = TskMotor::MotorMsg::EnableGyro;
    rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);

    vTaskDelay(100);// wait for all HW Enable

    SetLeds(0x00);

    rtn = xQueuePost(solve::MbTop, &msg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
    while(true)
    {
        sprintf(dbgStr, "%2.1f %2.1f %7.3f %7.3f %7.3f %7.3f\t\n",TskAction::Info[0],TskAction::Info[1],
                TskAction::Info[2],TskAction::Info[3],TskAction::Info[4],TskAction::Info[5]);
        rtn = xQueuePost(TskPrint::MbCmd, dbgStr, portMAX_DELAY);
        configASSERT(rtn == pdPASS);
        vTaskDelay(200);
    }
}

void clearMazeInfo()
{
    bool rtn;
    TskIr::eraseFlashBlock(62);
    SetLeds(0x1);
    vTaskDelay(500);
    SetLeds(0x0);
    vTaskDelay(500);
    sprintf(dbgStr, "Erase Ok\n");
    rtn = xQueuePost(TskPrint::MbCmd, dbgStr, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
    vTaskDelay(50);

}

void RushTest()
{
    bool rtn;
    TskMotor::MotorMsg::MsgType motMsg;
    solve::Solve::SolveType msg;

    SetLeds(0x00);

    vTaskDelay(1000);
    motMsg = TskMotor::MotorMsg::DisableMotors;
    rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
    motMsg = TskMotor::MotorMsg::EnableAcqZeros;
    rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);

    vTaskDelay(1000);// wait for getting imu zeros & getting ir zeros

//  motMsg = TskMotor::MotorMsg::DisableAcqZeros;
//  rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    motMsg = TskMotor::MotorMsg::EnableAccl;
    rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
    motMsg = TskMotor::MotorMsg::EnableMotors;
    rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
    motMsg = TskMotor::MotorMsg::EnableGyro;
    rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);

    vTaskDelay(100);// wait for all HW Enable

    msg = solve::Solve::RUSHTEST;
    rtn = xQueuePost(solve::MbTop, &msg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);

    while(true)
    {
        vTaskDelay(1000);
    }
}

void SetLeds(unsigned char val)
{
    LED_write(DBMOUSE_LED_0, (val & 0x1) ? DBMOUSE_LED_ON : DBMOUSE_LED_OFF);
    LED_write(DBMOUSE_LED_1, (val & 0x2) ? DBMOUSE_LED_ON : DBMOUSE_LED_OFF);
    LED_write(DBMOUSE_LED_2, (val & 0x4) ? DBMOUSE_LED_ON : DBMOUSE_LED_OFF);
}

void actPrint(TskAction::Act::ActType act)
{
    bool rtn;
    switch(act)
    {
    case TskAction::Act::Start | TskAction::Act::Corr:
        rtn = xQueuePost(TskPrint::MbCmd, "S\r\n", 0);
        break;
    case TskAction::Act::TBackR | TskAction::Act::Corr:
        rtn = xQueuePost(TskPrint::MbCmd, "T\r\n", 0);
        break;
    case TskAction::Act::Fwd | TskAction::Act::Corr:
        rtn = xQueuePost(TskPrint::MbCmd, "^\r\n", 0);
        break;
    case TskAction::Act::L90 | TskAction::Act::Corr:
        rtn = xQueuePost(TskPrint::MbCmd, "<\r\n", 0);
        break;
    case TskAction::Act::R90 | TskAction::Act::Corr:
        rtn = xQueuePost(TskPrint::MbCmd, ">\r\n", 0);
        break;
    }
    configASSERT(rtn == pdPASS);
//    vTaskDelay(50);
}

void dbgPutModeName(MouseMode::ModeType mode)
{
    BaseType_t rtn;
    switch(mode)
    {
    case MouseMode::EncImuMonitor:
        sprintf(dbgStr, "Mode Enc & Imu Monitor.Touch Ir to start\r\n");
        break;
    case MouseMode::IrMonitor:
        sprintf(dbgStr, "Mode Ir Monitor.Touch Ir to start\r\n");
        break;
    case MouseMode::IrCorrection:
        sprintf(dbgStr, "Mode Ir Correction.Touch Ir to start.\r\n");
        break;
    case MouseMode::UartCmd:
        sprintf(dbgStr, "Mode Listen Uart Command.\r\n");
        break;
    case MouseMode::ActionTest:
        sprintf(dbgStr, "Mode ActionTest.Touch Ir to start.\r\n");
        break;
    case MouseMode::SolveTest:
        sprintf(dbgStr, "Mode SolveTest.Touch Ir to start.\r\n");
        break;
    case MouseMode::ClearMaze:
        sprintf(dbgStr, "Mode Clear Maze Info.Touch Ir start!\n");
        break;
    default:
        sprintf(dbgStr, "Undefined mode.\r\n");
        break;
    }
    rtn = xQueuePost(TskPrint::MbCmd, dbgStr, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
    vTaskDelay(50);
}

void task(void *pvParameters)
{
    BaseType_t rtn;

	vTaskDelay(5000);
	SetLeds(0x01);
    vTaskDelay(100);
    SetLeds(0x02);
    vTaskDelay(100);
    SetLeds(0x04);
    vTaskDelay(100);
    SetLeds(0x00);
    vTaskDelay(1000);

    TskPrint::Init();
    TskMotor::Init();
    TskIr::Init();
    TskAction::Init();
    solve::Init();

    sprintf(dbgStr, "Hello from dbmouse! %s %s\n", VERSION, __TIME__);
    rtn = xQueuePost(TskPrint::MbCmd, dbgStr, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
    vTaskDelay(50);

    TskIr::IrMsg::MsgType irMsg;
    irMsg = TskIr::IrMsg::EnableEmitt;

//    sprintf(dbgStr, "%d\n", irMsg);
//    rtn = xQueuePost(TskPrint::MbCmd, dbgStr, portMAX_DELAY);
//    configASSERT(rtn == pdPASS);
//    vTaskDelay(50);

    rtn = xQueuePost(TskIr::MbCmd, &irMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);

    sprintf(dbgStr, "Roll wheel to change mode...\r\n");
    rtn = xQueuePost(TskPrint::MbCmd, dbgStr, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
    vTaskDelay(50);

    MouseMode::ModeType lastMode = Mode = MouseMode::Idle;
    SetLeds(Mode);
    dbgPutModeName(Mode);
    while(1)
    {
        Mode = (MouseMode::ModeType)(lroundf(TskMotor::DistanceAcc_en * 100.f) & 7);
        if(Mode != lastMode)
        {
            SetLeds(Mode);
            dbgPutModeName(Mode);
        }
        lastMode = Mode;
        if(TskIr::TestIrTouch(TskIr::IrCh::FL | TskIr::IrCh::FR, 1600, 1200))
        {
            SetLeds(0x00);
            switch(Mode)
            {
            case MouseMode::EncImuMonitor:
                encImuMonitor();
                break;
            case MouseMode::IrMonitor:
                irMonitor();
                break;
            case MouseMode::IrCorrection:
                doIrCorrection();
                break;
             case MouseMode::UartCmd:
                 uartCmd();
                 break;
             case MouseMode::ActionTest:
                 actionTest();
                break;
              case MouseMode::SolveTest:
                 solveTest();
                 break;
              case MouseMode::ClearMaze:
                 clearMazeInfo();
                 break;
              case MouseMode::Gaming1:
                 RushTest();
                 break;
             default:
                Mode = MouseMode::Idle;
                break;
             }

        }
        vTaskDelay(50);
    }
}

void Init()
{
	BaseType_t rtn;

    MbCmd = xQueueCreate(4, sizeof(TskAction::ActMsg::MsgType));
	configASSERT(MbCmd);
    // Create tasks
    rtn = xTaskCreate(task, (const portCHAR *)"TopTask",
                tskStkSize, NULL, tskPrio, NULL);
    configASSERT(rtn == pdPASS);
}

}

