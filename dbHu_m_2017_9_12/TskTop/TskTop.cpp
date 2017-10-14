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
const int tskStkSize = 2048;
Task_Params tskParams;
Task_Handle tsk;
//Error_Block eb;
#define testBaseAction 1
#define testRushAction 0
#define testRushOAct   0
Mailbox_Handle MbCmd;

char dbgStr[100];
short info_flag = 0;

volatile MouseMode::ModeType Mode;

void doIrCorrection()
{
    DbgUartPutLine("Starting Ir Correction precedure. Wait seconds plz...\n", true);

    TskMotor::MotorMsg::MsgType motMsg;
    TskIr::IrMsg::MsgType irMsg;

    Task_sleep(1000);

    motMsg = TskMotor::MotorMsg::DisableMotors;
    Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
    irMsg = TskIr::IrMsg::DisableEmitt;
    Mailbox_post(TskIr::MbCmd, &irMsg, BIOS_NO_WAIT);

    /*
     *use encoder as the condition to get ir data
     */
    motMsg = TskMotor::MotorMsg::EnableAcqZeros;
    Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);

    Task_sleep(1000);   // wait for getting imu zeros & ir zeros

    motMsg = TskMotor::MotorMsg::DisableAcqZeros;
    Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);

    irMsg = TskIr::IrMsg::EnableEmitt;
    Mailbox_post(TskIr::MbCmd, &irMsg, BIOS_NO_WAIT);

    Task_sleep(200);

    TskIr::doIrCorrection();
}

void encImuMonitor()
{
    int i = 0;
    TskMotor::MotorMsg::MsgType motMsg;
    TskIr::IrMsg::MsgType irMsg;

	DbgUartPutLine("verson 2.5.2\n", true);
    Task_sleep(1000);

    motMsg = TskMotor::MotorMsg::DisableMotors;
    Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
    irMsg = TskIr::IrMsg::DisableEmitt;
    Mailbox_post(TskIr::MbCmd, &irMsg, BIOS_NO_WAIT);
    motMsg = TskMotor::MotorMsg::EnableAcqZeros;
    Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);

    Task_sleep(1000);// wait for getting imu zeros & getting ir zeros

    motMsg = TskMotor::MotorMsg::EnableAccl;
    Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
    motMsg = TskMotor::MotorMsg::EnableGyro;
    Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
    irMsg = TskIr::IrMsg::EnableEmitt;
    Mailbox_post(TskIr::MbCmd, &irMsg, BIOS_NO_WAIT);

    TskMotor::DistanceAcc = 0.f;
    TskMotor::DistanceAcc_en = 0.f;
    TskMotor::AngleAcc = 0.f;

    while(true)
    {
        if(!(i++ & 0xf))
        {
            DbgUartPutLine( "\nGyroZ AXZero AcclX EncVel KalVel DisAcc AngAcc\r\n", true);
        }
        sprintf(dbgStr, "%7.3f,%7.3f,%7.3f,%7.3f,%7.4f,%7.4f,%8.3f,%8.3f\r\n",
                TskMotor::GyroZZero, TskMotor::AV,
                TskMotor::AcclXZero, TskMotor::AcclX,
                TskMotor::EncVel, TskMotor::LV,
                TskMotor::DistanceAcc,
                //TskMotor::DistanceAcc_en,
                TskMotor::AngleAcc);
        Task_sleep(300);
        DbgUartPutLine(dbgStr, true);
        Task_sleep(200);
        if(TskIr::TestIrTouch(TskIr::IrCh::FL | TskIr::IrCh::FR, 1600, 1200))
            break;
    }

    Mode = MouseMode::Idle;
}

void irMonitor()
{
    TskMotor::MotorMsg::MsgType motMsg;
    TskIr::IrMsg::MsgType irMsg;

    Task_sleep(1000);

    motMsg = TskMotor::MotorMsg::DisableMotors;
    Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);

    irMsg = TskIr::IrMsg::DisableEmitt;
    Mailbox_post(TskIr::MbCmd, &irMsg, BIOS_NO_WAIT);

    Task_sleep(100);

    irMsg = TskIr::IrMsg::EnableEmitt;
    Mailbox_post(TskIr::MbCmd, &irMsg, BIOS_NO_WAIT);

    while(true)
    {
       // if(!(i++ & 0xf))
       // {
       //     DbgUartPutLine( "\n AvgFwd:b\t   LFwd:b\t   RFwd:b\t  LSide:b\t  RSide:b\t  HdbyL\t  HdbyR\tHdbyFLR\t slInt\t srInt\r\n", true);
       // }
//        sprintf(dbgStr, "%7.4f:%1d\t%7.4f:%1d\t%7.4f:%1d\t%7.4f:%1d\t%7.4f:%1d\t%7.1f\t%7.1f\t%7.1f\t%4d\t%4d\r\n",
//                .5f * (TskIr::IrDists.FLns + TskIr::IrDists.FRns), TskIr::IrBins.Fwd,
//                TskIr::IrDists.FLns, TskIr::IrBins.FLns,
//                TskIr::IrDists.FRns, TskIr::IrBins.FRns,
//                TskIr::IrDists.LS, TskIr::IrBins.LS,
//                TskIr::IrDists.RS, TskIr::IrBins.RS,
//                TskIr::IrYaw.byLS * 180.f / 3.1415927f,
//                TskIr::IrYaw.byRS * 180.f / 3.1415927f,
//                TskIr::IrYaw.byFLR * 180.f / 3.1415927f,
//              TskIr::IrInts.sl,
//              TskIr::IrInts.sr
//        );
		sprintf(dbgStr, "%4d,%6.4f,%4d,%6.4f,%4d,%6.4f,%4d,%6.4f\r\n",
				TskIr::IrInts.fl, TskIr::IrDists.FLns,
				TskIr::IrInts.fr, TskIr::IrDists.FRns,
				TskIr::IrInts.sl, TskIr::IrDists.LS,
				TskIr::IrInts.sr, TskIr::IrDists.RS
		);
        Task_sleep(300);
        DbgUartPutLine(dbgStr, true);
        Task_sleep(200);
//        if(TskIr::TestIrTouch(TskIr::IrCh::FL | TskIr::IrCh::FR, 1600, 1200))
//            break;
    }

//    Mode = MouseMode::Idle;
}

void uartCmd()
{

	while(true)
	{
        cmd_shell();
	}

}

void solveTest()
{
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
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	motMsg = TskMotor::MotorMsg::EnableAcqZeros;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);

	Task_sleep(1000);// wait for getting imu zeros & getting ir zeros

//	motMsg = TskMotor::MotorMsg::DisableAcqZeros;
//	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	motMsg = TskMotor::MotorMsg::EnableAccl;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	motMsg = TskMotor::MotorMsg::EnableMotors;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	motMsg = TskMotor::MotorMsg::EnableGyro;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);

	Task_sleep(100);// wait for all HW Enable

	TskMotor::DistanceAcc = 0.f;
	TskMotor::AngleAcc = 0.f;
	TskMotor::DistanceAcc_en = 0.f;

	SetLeds(0x00);
	DbgUartPutLine("verson 2.7.0\n", true);

	Mailbox_post(solve::MbTop, &msg, BIOS_NO_WAIT);
	while(true)
	{
        sprintf(dbgStr, "%2.1f %2.1f %7.3f %7.3f %7.3f %7.3f\t\n",TskAction::Info[0],TskAction::Info[1],
        		TskAction::Info[2],TskAction::Info[3],TskAction::Info[4],TskAction::Info[5]);
        DbgUartPutLine(dbgStr, true);
		Task_sleep(200);
	}
}

void clearMazeInfo()
{
    TskIr::eraseFlashBlock(254);
    SetLeds(0x1);
    Task_sleep(500);
    SetLeds(0x0);
    Task_sleep(500);
    DbgUartPutLine("Erase Ok\n", true);

}

void RushTest()
{
	TskMotor::MotorMsg::MsgType motMsg;
	solve::Solve::SolveType msg;

	SetLeds(0x00);

	Task_sleep(1000);
	motMsg = TskMotor::MotorMsg::DisableMotors;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	motMsg = TskMotor::MotorMsg::EnableAcqZeros;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);

	Task_sleep(1000);// wait for getting imu zeros & getting ir zeros

//	motMsg = TskMotor::MotorMsg::DisableAcqZeros;
//	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	motMsg = TskMotor::MotorMsg::EnableAccl;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	motMsg = TskMotor::MotorMsg::EnableMotors;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	motMsg = TskMotor::MotorMsg::EnableGyro;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);

	Task_sleep(100);// wait for all HW Enable

	TskMotor::DistanceAcc = 0.f;
	TskMotor::AngleAcc = 0.f;
	TskMotor::DistanceAcc_en = 0.f;

	msg = solve::Solve::RUSHTEST;
	Mailbox_post(solve::MbTop, &msg, BIOS_NO_WAIT);

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

void actPrint(TskAction::Act::ActType act, char *str)
{
	switch(act)
	{
	case TskAction::Act::Start:
        sprintf(str, "Act Start.\n");
        break;
	case TskAction::Act::Stop:
        sprintf(str, "Act Stop.\n");
        break;
	case TskAction::Act::Back:
        sprintf(str, "Act Back.\n");
        break;
	case TskAction::Act::Restart:
        sprintf(str, "Act Restart.\n");
        break;
	case TskAction::Act::Fwd:
        sprintf(str, "Act Fwd.\n");
        break;
	case TskAction::Act::L90:
        sprintf(str, "Act L90.\n");
        break;
	case TskAction::Act::R90:
        sprintf(str, "Act R90.\n");
        break;
	case TskAction::Act::Null:
        sprintf(str, "Act Null.\n");
        break;
	}
}

void modeName(MouseMode::ModeType mode, char *str)
{
    switch(mode)
    {
    case MouseMode::EncImuMonitor:
        sprintf(str, "Mode Enc & Imu Monitor.Touch Ir to start\r\n");
        break;
    case MouseMode::IrMonitor:
        sprintf(str, "Mode Ir Monitor.Touch Ir to start\r\n");
        break;
    case MouseMode::IrCorrection:
        sprintf(str, "Mode Ir Correction.Touch Ir to start.\r\n");
        break;
    case MouseMode::UartCmd:
        sprintf(str, "Mode Listen Uart Command.\r\n");
        break;
    case MouseMode::SolveTest:
        sprintf(str, "Mode SolveTest.Touch Ir to start.\r\n");
        break;
    case MouseMode::Undef1:
        sprintf(str, "Mode Undef1.\n");
        break;
    case MouseMode::Undef2:
        sprintf(str, "Mode Undef2.\n");
        break;
    case MouseMode::Undef3:
        sprintf(str, "Mode Undef3.\n");
        break;
    case MouseMode::Gaming1:
        sprintf(str, "Mode Gaming1.Touch Ir & GO!\n");
        break;
    case MouseMode::Gaming2:
        sprintf(str, "Mode Gaming2.Touch Ir & GO!\n");
        break;
    case MouseMode::Gaming3:
        sprintf(str, "Mode Gaming3.Touch Ir & GO!\n");
        break;
    case MouseMode::Gaming4:
        sprintf(str, "Mode Gaming4.Touch Ir & GO!\n");
        break;
    case MouseMode::ClearMaze:
        sprintf(str, "Mode Clear Maze Info.Touch Ir start!\n");
        break;
    default:
        sprintf(str, "Undefined mode.\r\n");
        break;
    }
}

inline void dbgPutModeName(MouseMode::ModeType mode)
{
    modeName(mode, dbgStr);
    DbgUartPutLine(dbgStr, true);
}

void task(UArg arg0, UArg arg1)
{

    TskIr::IrMsg::MsgType irMsg;

    InitDbgUart();

    SetLeds(0x01);
    Task_sleep(100);
    SetLeds(0x02);
    Task_sleep(100);
    SetLeds(0x04);
    Task_sleep(100);
    SetLeds(0x00);

    printf("Hello from dbmouse!\r\n");
    fflush(stdout);
    Task_sleep(1000);
    TskAction::Init();
    TskMotor::Init();
    TskIr::Init();
    solve::Init();

    irMsg = TskIr::IrMsg::EnableEmitt;
    Mailbox_post(TskIr::MbCmd, &irMsg, BIOS_NO_WAIT);

    Task_sleep(500);
    printf("Roll wheel to change mode...\r\n");
    fflush(stdout);

    MouseMode::ModeType lastMode = Mode = MouseMode::Idle;
    SetLeds(Mode);
    dbgPutModeName(Mode);
    while(true)
    {
   	    //Mode = (MouseMode::ModeType)2;
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
////        System_printf("%d\n", lroundf(TskMotor::DistanceAcc * 10000.f));
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

    MbCmd = Mailbox_create(4, 4, NULL, NULL);
    if(MbCmd == NULL)
        System_abort("create TskTop::MbCmd failed.\n");

    tsk = Task_create((Task_FuncPtr)task, &tskParams, NULL);
    if(tsk == NULL)
    {
        System_abort("Task Top failed");
    }
}

}


