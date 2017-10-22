/*
 * top.cc
 *
 *  Created on: Jul 21, 2016
 *      Author: loywong
 */

#include <string.h>
#include <stdio.h>
#include <math.h>

#include <xdc/runtime/System.h>
#include <xdc/runtime/Assert.h>

#include <ti/sysbios/BIOS.h>
#include <ti/drivers/GPIO.h>

#include "../physparams.h"
#include "TskTop/DbgUart.h"
#include "dbmouse_chassis.h"
#include "action/action.h"
#include "TskTop/TskTop.h"
#include "../TskMotor/TskMotor.h"
#include "../TskIr/TskIr.h"
#include "../TskIr/IrCorr.h"
#include "solve/solve.h"
#include "../TskIr/IrCorr.h"
#include "TskTop/TskTop.h"
#include "TskTop/cmd.h"

namespace TskTop
{

const int tskPrio = 4;
const int tskStkSize = 4096;
Task_Params tskParams;
Task_Handle tsk;
//Error_Block eb;
#define testBaseAction 1
#define testRushAction 0
#define testRushOAct   0
Mailbox_Handle MbCmd;

char dbgStr[128];

volatile MouseMode::ModeType Mode;

void doIrCorrection()
{
    bool rtn;
    sprintf(dbgStr, "Starting Ir Correction precedure. Wait seconds plz...\n");
    rtn=Mailbox_post(TskPrint::MbCmd, dbgStr, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);
    Task_sleep(50);

    TskMotor::MotorMsg::MsgType motMsg;
    TskIr::IrMsg::MsgType irMsg;

    Task_sleep(1000);

    motMsg = TskMotor::MotorMsg::DisableMotors;
    rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);
    irMsg = TskIr::IrMsg::DisableEmitt;
    rtn=Mailbox_post(TskIr::MbCmd, &irMsg, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);

    /*
     *use encoder as the condition to get ir data
     */
    motMsg = TskMotor::MotorMsg::EnableAcqZeros;
    rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);

    Task_sleep(1000);   // wait for getting imu zeros & ir zeros

    motMsg = TskMotor::MotorMsg::DisableAcqZeros;
    rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);

    irMsg = TskIr::IrMsg::EnableEmitt;
    rtn=Mailbox_post(TskIr::MbCmd, &irMsg, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);

    Task_sleep(200);

    TskIr::doIrCorrection();
}

void encImuMonitor()
{
    int i = 0;
    bool rtn;
    TskMotor::MotorMsg::MsgType motMsg;
    TskIr::IrMsg::MsgType irMsg;

	sprintf(dbgStr, "verson 2.5.2\n");
    Task_sleep(1000);

    motMsg = TskMotor::MotorMsg::DisableMotors;
    rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_WAIT_FOREVER);
    Assert_isTrue(rtn,NULL);
    irMsg = TskIr::IrMsg::DisableEmitt;
    rtn=Mailbox_post(TskIr::MbCmd, &irMsg, BIOS_WAIT_FOREVER);
    Assert_isTrue(rtn,NULL);
    motMsg = TskMotor::MotorMsg::EnableAcqZeros;
    rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_WAIT_FOREVER);
    Assert_isTrue(rtn,NULL);

    Task_sleep(1000);// wait for getting imu zeros & getting ir zeros

    motMsg = TskMotor::MotorMsg::EnableAccl;
    rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_WAIT_FOREVER);
    Assert_isTrue(rtn,NULL);
    motMsg = TskMotor::MotorMsg::EnableGyro;
    rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_WAIT_FOREVER);
    Assert_isTrue(rtn,NULL);
    irMsg = TskIr::IrMsg::EnableEmitt;
    rtn=Mailbox_post(TskIr::MbCmd, &irMsg, BIOS_WAIT_FOREVER);
    Assert_isTrue(rtn,NULL);

    while(true)
    {
        if(!(i++ & 0xf))
        {
            sprintf(dbgStr, "\nGyroZ AXZero AcclX EncVel KalVel DisAcc AngAcc\r\n");
            rtn=Mailbox_post(TskPrint::MbCmd, dbgStr, BIOS_WAIT_FOREVER);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
        }
        sprintf(dbgStr, "%7.3f,%7.3f,%7.3f,%7.3f,%7.4f,%7.4f,%8.3f,%8.3f\r\n",
                TskMotor::GyroZZero, TskMotor::AV,
                TskMotor::AcclXZero, TskMotor::AcclX,
                TskMotor::EncVel, TskMotor::LV,
                TskMotor::DistanceAcc,
                //TskMotor::DistanceAcc_en,
                TskMotor::AngleAcc);
        rtn=Mailbox_post(TskPrint::MbCmd, dbgStr, BIOS_WAIT_FOREVER);
        Assert_isTrue(rtn,NULL);
        Task_sleep(500);
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

    Task_sleep(1000);

    motMsg = TskMotor::MotorMsg::DisableMotors;
    rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_WAIT_FOREVER);
    Assert_isTrue(rtn,NULL);

    irMsg = TskIr::IrMsg::DisableEmitt;
    rtn=Mailbox_post(TskIr::MbCmd, &irMsg, BIOS_WAIT_FOREVER);
    Assert_isTrue(rtn,NULL);

    Task_sleep(100);

    irMsg = TskIr::IrMsg::EnableEmitt;
    rtn=Mailbox_post(TskIr::MbCmd, &irMsg, BIOS_WAIT_FOREVER);
    Assert_isTrue(rtn,NULL);

    while(true)
    {
       if(!(i++ & 0xf))
       {
           sprintf(dbgStr, "\tAve:b\t  LFwd:b\t   RFwd:b\t  LSide:b\t  RSide:b\t  HdbyL\t  HdbyR\tHdbyFLR\r\n");
           rtn=Mailbox_post(TskPrint::MbCmd, dbgStr, BIOS_WAIT_FOREVER);
           Assert_isTrue(rtn,NULL);
           Task_sleep(50);
       }
       sprintf(dbgStr, "%7.4f:%1d\t%7.4f:%1d\t%7.4f:%1d",
               .5f * (TskIr::IrDists.FLns + TskIr::IrDists.FRns), TskIr::IrBins.Fwd,
               TskIr::IrDists.FLns, TskIr::IrBins.FLns,
               TskIr::IrDists.FRns, TskIr::IrBins.FRns
       );
       rtn=Mailbox_post(TskPrint::MbCmd, dbgStr, BIOS_WAIT_FOREVER);
       Assert_isTrue(rtn,NULL);
       Task_sleep(50);
       sprintf(dbgStr, "%7.4f:%1d\t%7.4f:%1d\t%7.1f\t%7.1f\t%7.1f\r\n",
               TskIr::IrDists.LS, TskIr::IrBins.LS,
               TskIr::IrDists.RS, TskIr::IrBins.RS,
               TskIr::IrYaw.byLS * 180.f / 3.1415927f,
               TskIr::IrYaw.byRS * 180.f / 3.1415927f,
               TskIr::IrYaw.byFLR * 180.f / 3.1415927f
               // TskIr::IrInts.sl,
               // TskIr::IrInts.sr
        );
        rtn=Mailbox_post(TskPrint::MbCmd, dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
		// sprintf(dbgStr, "%4d,%6.4f,%4d,%6.4f,%4d,%6.4f,%4d,%6.4f\r\n",
		// 		TskIr::IrInts.fl, TskIr::IrDists.FLns,
		// 		TskIr::IrInts.fr, TskIr::IrDists.FRns,
		// 		TskIr::IrInts.sl, TskIr::IrDists.LS,
		// 		TskIr::IrInts.sr, TskIr::IrDists.RS
		// );
        Task_sleep(500);
        if(TskIr::TestIrTouch(TskIr::IrCh::FL | TskIr::IrCh::FR, 1600, 1200))
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
            rtn=Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
            Assert_isTrue(rtn,NULL);
            actMsg = (TskAction::Act::ActType)(TskAction::Act::L90);
            rtn=Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            rtn=Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
            Assert_isTrue(rtn,NULL);
            actMsg = (TskAction::Act::ActType)(TskAction::Act::Stop);
            rtn=Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            rtn=Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
            Assert_isTrue(rtn,NULL);
            TskAction::WaitQEnd();
            Task_sleep(50);
            MotorStop();
        }
        sprintf(dbgStr, "%6.3f,%6.3f,%6.3f\r\n",
            TskMotor::DistanceAcc,
            TskMotor::DistanceAcc_en,
            TskMotor::AngleAcc);
        rtn=Mailbox_post(TskPrint::MbCmd, dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(50);
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
	Task_sleep(1000);

	motMsg = TskMotor::MotorMsg::DisableMotors;
	rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);
	motMsg = TskMotor::MotorMsg::EnableAcqZeros;
	rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);

	Task_sleep(1000);// wait for getting imu zeros & getting ir zeros

//	motMsg = TskMotor::MotorMsg::DisableAcqZeros;
//	rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	motMsg = TskMotor::MotorMsg::EnableAccl;
	rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);
	motMsg = TskMotor::MotorMsg::EnableMotors;
	rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);
	motMsg = TskMotor::MotorMsg::EnableGyro;
	rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);

	Task_sleep(100);// wait for all HW Enable

	TskMotor::DistanceAcc = 0.f;
	TskMotor::AngleAcc = 0.f;
	TskMotor::DistanceAcc_en = 0.f;

	SetLeds(0x00);

	rtn=Mailbox_post(solve::MbTop, &msg, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);
	while(true)
	{
        sprintf(dbgStr, "%2.1f %2.1f %7.3f %7.3f %7.3f %7.3f\t\n",TskAction::Info[0],TskAction::Info[1],
        		TskAction::Info[2],TskAction::Info[3],TskAction::Info[4],TskAction::Info[5]);
        rtn=Mailbox_post(TskPrint::MbCmd, dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
		Task_sleep(200);
	}
}

void clearMazeInfo()
{
    bool rtn;
    TskIr::eraseFlashBlock(62);
    SetLeds(0x1);
    Task_sleep(500);
    SetLeds(0x0);
    Task_sleep(500);
    sprintf(dbgStr, "Erase Ok\n");
    rtn=Mailbox_post(TskPrint::MbCmd, dbgStr, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);
    Task_sleep(50);

}

void RushTest()
{
    bool rtn;
	TskMotor::MotorMsg::MsgType motMsg;
	solve::Solve::SolveType msg;

	SetLeds(0x00);

	Task_sleep(1000);
	motMsg = TskMotor::MotorMsg::DisableMotors;
	rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);
	motMsg = TskMotor::MotorMsg::EnableAcqZeros;
	rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);

	Task_sleep(1000);// wait for getting imu zeros & getting ir zeros

//	motMsg = TskMotor::MotorMsg::DisableAcqZeros;
//	rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	motMsg = TskMotor::MotorMsg::EnableAccl;
	rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);
	motMsg = TskMotor::MotorMsg::EnableMotors;
	rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);
	motMsg = TskMotor::MotorMsg::EnableGyro;
	rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);

	Task_sleep(100);// wait for all HW Enable

	TskMotor::DistanceAcc = 0.f;
	TskMotor::AngleAcc = 0.f;
	TskMotor::DistanceAcc_en = 0.f;

	msg = solve::Solve::RUSHTEST;
	rtn=Mailbox_post(solve::MbTop, &msg, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);

	while(true)
	{
		Task_sleep(1000);
	}
}

void SetLeds(unsigned char val)
{
    GPIO_write(DBMOUSE_LED_0, (val & 0x4) ? DBMOUSE_LED_ON : DBMOUSE_LED_OFF);
    GPIO_write(DBMOUSE_LED_1, (val & 0x2) ? DBMOUSE_LED_ON : DBMOUSE_LED_OFF);
    GPIO_write(DBMOUSE_LED_2, (val & 0x1) ? DBMOUSE_LED_ON : DBMOUSE_LED_OFF);
}

void actPrint(TskAction::Act::ActType act)
{
    bool rtn;
    switch(act)
    {
    case TskAction::Act::Start:
        sprintf(dbgStr, "Act Start.\n");
        rtn=Mailbox_post(TskPrint::MbCmd, dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(50);
        break;
    case TskAction::Act::Stop:
        sprintf(dbgStr, "Act Stop.\n");
        rtn=Mailbox_post(TskPrint::MbCmd, dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(50);
        break;
    case TskAction::Act::Back:
        sprintf(dbgStr, "Act Back.\n");
        rtn=Mailbox_post(TskPrint::MbCmd, dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(50);
        break;
    case TskAction::Act::Restart:
        sprintf(dbgStr, "Act Restart.\n");
        rtn=Mailbox_post(TskPrint::MbCmd, dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(50);
        break;
    case TskAction::Act::Fwd:
        sprintf(dbgStr, "Act Fwd.\n");
        rtn=Mailbox_post(TskPrint::MbCmd, dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(50);
        break;
    case TskAction::Act::L90:
        sprintf(dbgStr, "Act L90.\n");
        rtn=Mailbox_post(TskPrint::MbCmd, dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(50);
        break;
    case TskAction::Act::R90:
        sprintf(dbgStr, "Act R90.\n");
        rtn=Mailbox_post(TskPrint::MbCmd, dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(50);
        break;
    case TskAction::Act::Null:
        sprintf(dbgStr, "Act Null.\n");
        rtn=Mailbox_post(TskPrint::MbCmd, dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(50);
        break;
    }
}

void dbgPutModeName(MouseMode::ModeType mode)
{
    bool rtn;
    switch(mode)
    {
    case MouseMode::EncImuMonitor:
        sprintf(dbgStr, "Mode Enc & Imu Monitor.Touch Ir to start\r\n");
        rtn=Mailbox_post(TskPrint::MbCmd, dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(50);
        break;
    case MouseMode::IrMonitor:
        sprintf(dbgStr, "Mode Ir Monitor.Touch Ir to start\r\n");
        rtn=Mailbox_post(TskPrint::MbCmd, dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(50);
        break;
    case MouseMode::IrCorrection:
        sprintf(dbgStr, "Mode Ir Correction.Touch Ir to start.\r\n");
        rtn=Mailbox_post(TskPrint::MbCmd, dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(50);
        break;
    case MouseMode::UartCmd:
        sprintf(dbgStr, "Mode Listen Uart Command.\r\n");
        rtn=Mailbox_post(TskPrint::MbCmd, dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(50);
        break;
    case MouseMode::ActionTest:
        sprintf(dbgStr, "Mode ActionTest.Touch Ir to start.\r\n");
        rtn=Mailbox_post(TskPrint::MbCmd, dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(50);
        break;
    case MouseMode::SolveTest:
        sprintf(dbgStr, "Mode SolveTest.Touch Ir to start.\r\n");
        rtn=Mailbox_post(TskPrint::MbCmd, dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(50);
        break;
    case MouseMode::ClearMaze:
        sprintf(dbgStr, "Mode Clear Maze Info.Touch Ir start!\n");
        rtn=Mailbox_post(TskPrint::MbCmd, dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(50);
        break;
    default:
        sprintf(dbgStr, "Undefined mode.\r\n");
        rtn=Mailbox_post(TskPrint::MbCmd, dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(50);
        break;
    }
}

void task(UArg arg0, UArg arg1)
{
    bool rtn;
    Task_sleep(1000);

    SetLeds(0x01);
    Task_sleep(100);
    SetLeds(0x02);
    Task_sleep(100);
    SetLeds(0x04);
    Task_sleep(100);
    SetLeds(0x00);

    Task_sleep(1000);

    TskPrint::Init();
    TskMotor::Init();
    TskIr::Init();
    TskAction::Init();
    solve::Init();

    sprintf(dbgStr, "Hello from dbmouse!\n");
    rtn=Mailbox_post(TskPrint::MbCmd, dbgStr, BIOS_WAIT_FOREVER);
    Assert_isTrue(rtn,NULL);
    Task_sleep(50);

    TskIr::IrMsg::MsgType irMsg;
    irMsg = TskIr::IrMsg::EnableEmitt;
    rtn=Mailbox_post(TskIr::MbCmd, &irMsg, BIOS_WAIT_FOREVER);
    Assert_isTrue(rtn,NULL);

    Task_sleep(500);
    sprintf(dbgStr, "Roll wheel to change mode...\r\n");
    rtn=Mailbox_post(TskPrint::MbCmd, dbgStr, BIOS_WAIT_FOREVER);
    Assert_isTrue(rtn,NULL);
    Task_sleep(50);

    MouseMode::ModeType lastMode = Mode = MouseMode::Idle;
    SetLeds(Mode);
    dbgPutModeName(Mode);
    while(true)
    {
//   	     Mode = (MouseMode::ModeType)1;
         Mode = (MouseMode::ModeType)(lroundf(TskMotor::DistanceAcc_en * 100.f) & 7);
         if(Mode != lastMode)
         {
             SetLeds(Mode);
             dbgPutModeName(Mode);
         }
         lastMode = Mode;
         if(TskIr::TestIrTouch(TskIr::IrCh::FL | TskIr::IrCh::FR, 1600, 1200))
         {
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
        Task_sleep(50);
    }

//    while(true)
//    {
//        Task_sleep(500);
////        System_sprintf(dbgStr, "%d\n", lroundf(TskMotor::DistanceAcc * 10000.f));
////        System_flush();
//        sprintf(dbgStr, "%4d\t%4d\t%4d\t%4d\n",
//                TskIr::IrInts.fl,
//                TskIr::IrInts.sl,
//                TskIr::IrInts.sr,
//                TskIr::IrInts.fr);
//        DbgUartPutLine(dbgStr);
//    }
}

void Init()
{
//    Error_init(&eb);
    Task_Params_init(&tskParams);
    tskParams.priority = tskPrio;
    tskParams.stackSize = tskStkSize;

    MbCmd = Mailbox_create(sizeof(TskAction::ActMsg::MsgType), 4, NULL, NULL);
    if(MbCmd == NULL)
        System_abort("create TskTop::MbCmd failed.\n");

    tsk = Task_create((Task_FuncPtr)task, &tskParams, NULL);
    if(tsk == NULL)
    {
        System_abort("Task Top failed");
    }
}

}


