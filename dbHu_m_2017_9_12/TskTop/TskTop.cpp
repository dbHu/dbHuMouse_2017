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

volatile MouseMode::ModeType Mode;

void actPrint(TskAction::Act::ActType act, char *str);

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
        DbgUartPutLine(dbgStr, true);
        Task_sleep(500);
    }
}

void irMonitor()
{
    int i = 0;

    TskMotor::MotorMsg::MsgType motMsg;
    TskIr::IrMsg::MsgType irMsg;

    Task_sleep(1000);

    motMsg = TskMotor::MotorMsg::DisableMotors;
    Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
    irMsg = TskIr::IrMsg::DisableEmitt;
    Mailbox_post(TskIr::MbCmd, &irMsg, BIOS_NO_WAIT);
    motMsg = TskMotor::MotorMsg::EnableAcqZeros;
    Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);

    Task_sleep(1000);// wait for getting imu zeros & getting ir zeros

    motMsg = TskMotor::MotorMsg::DisableAcqZeros;
    Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
    motMsg = TskMotor::MotorMsg::EnableAccl;
    Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
    motMsg = TskMotor::MotorMsg::EnableGyro;
    Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
    irMsg = TskIr::IrMsg::EnableEmitt;
    Mailbox_post(TskIr::MbCmd, &irMsg, BIOS_NO_WAIT);

    while(true)
    {
//        if(!(i++ & 0xf))
//        {
//            DbgUartPutLine( "\n AvgFwd:b\t   LFwd:b\t   RFwd:b\t  LSide:b\t  RSide:b\t  HdbyL\t  HdbyR\tHdbyFLR\t slInt\t srInt\r\n", true);
//        }
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

		sprintf(dbgStr, "%4d,%7.4f,%4d,%7.4f,%4d,%7.4f,%4d,%7.4f\r\n",
				TskIr::IrInts.fl, TskIr::IrDists.FLns,
				TskIr::IrInts.fr, TskIr::IrDists.FRns,
				TskIr::IrInts.sl, TskIr::IrDists.LS,
				TskIr::IrInts.sr, TskIr::IrDists.RS
		);
        DbgUartPutLine(dbgStr, true);
        Task_sleep(500);
    }

}

void actionTest()
{
	int i=0,actLen=0;
	char str[50] = {0};
	SetLeds(0x7);
	TskMotor::MotorMsg::MsgType motMsg;
	TskIr::IrMsg::MsgType irMsg;
	TskAction::Act::ActType actMsg;
	TskAction::ActMsg::MsgType end_Msg;
	TskAction::Act::ActType actArray[10];
	Task_sleep(1000);
	SetLeds(0x0);
	motMsg = TskMotor::MotorMsg::DisableMotors;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	motMsg = TskMotor::MotorMsg::EnableAcqZeros;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);

	Task_sleep(1000);// wait for getting imu zeros & getting ir zeros
	SetLeds(0x0);
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

    DbgUartPutLine("str[0]  how many times to act,don't more than 5     default:1\r\n",true);
    DbgUartPutLine("str[1]  l r f choose which action to adjust         default:f\r\n",true);
    DbgUartPutLine("str[2]  how many times to act,don't more than 5     default:0\r\n",true);
    DbgUartPutLine("str[3]  1,2 :L pre -+ 3,4: post -+ \r\n     5,6 :R pre -+ 7,8: post -+  default:0\r\n",true);
    Task_sleep(10);

	while(true)
	{
	    while(str[0] <= '0' || str[0] >= '9')
	    {
	        DbgUartGetLine(str);
	        Task_sleep(10);
	    }
	    actLen = cmd(str,actArray);
        sprintf(dbgStr, "%1.4f,%1.4f,%1.4f,%1.4f\r\n",
              CP.TURNL90_PRE_ADJ,
              CP.TURNL90_POST_ADJ,
              CP.TURNR90_PRE_ADJ,
              CP.TURNR90_POST_ADJ);
        DbgUartPutLine(dbgStr, true);
	    for(int a = 0; a < actLen; a++)
	    {
	    	actMsg = actArray[a];
	        actPrint(actMsg, dbgStr);
	        DbgUartPutLine(dbgStr, true);
	        Task_sleep(1);
	   		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
       		Mailbox_pend(MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
	    }
	    Task_sleep(5000);
        sprintf(dbgStr, "%8.3f,%8.3f,%8.3f\r\n",
              TskMotor::DistanceAcc,
              TskMotor::DistanceAcc_en,
              TskMotor::AngleAcc);
        DbgUartPutLine(dbgStr, true);
        motMsg = TskMotor::MotorMsg::DisableMotors;
        Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
        irMsg = TskIr::IrMsg::DisableEmitt;
        Mailbox_post(TskIr::MbCmd, &irMsg, BIOS_NO_WAIT);
        motMsg = TskMotor::MotorMsg::DisableAcqZeros;
        Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
        motMsg = TskMotor::MotorMsg::DisableAccl;
        Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
        motMsg = TskMotor::MotorMsg::DisableGyro;
        Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
        SetLeds(0x00);
        TskMotor::DistanceAcc = 0.f;
        TskMotor::DistanceAcc_en = 0.f;
        TskMotor::AngleAcc = 0.f;
        Task_sleep(1000);
        SetLeds(0x07);
        Task_sleep(2000);
        irMsg = TskIr::IrMsg::EnableEmitt;
        Mailbox_post(TskIr::MbCmd, &irMsg, BIOS_NO_WAIT);
	    break;
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

int info_flag = 0;
void PhysparamCorr()
{
	int i=0;
	SetLeds(0x7);
	TskMotor::MotorMsg::MsgType motMsg;
	TskAction::Act::ActType actMsg;
	TskAction::ActMsg::MsgType end_Msg;
	Task_sleep(1000);

	motMsg = TskMotor::MotorMsg::DisableMotors;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	motMsg = TskMotor::MotorMsg::EnableAcqZeros;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);

	Task_sleep(1000);// wait for getting imu zeros & getting ir zeros
	SetLeds(0x0);
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

	DbgUartPutLine("triggle IR L for V pid, triggle IR R for W PID\r\n", true);
	Task_sleep(1000);

    while(true){
#if 1
    	while(1){
    	    SetLeds(0x1);
        	if(TskIr::TestIrTouch(TskIr::IrCh::FL, 1600, 1200))
        	{
        	    SetLeds(0x0);
        		info_flag = 1;
        		DbgUartPutLine("start Velocity pid test\n", true);
           		actMsg = (TskAction::Act::ActType)(TskAction::Act::Start);
           	    Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
           	    Mailbox_pend(MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
           		actMsg = (TskAction::Act::ActType)(TskAction::Act::Stop);
           	    Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
           	    Mailbox_pend(MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
            	break;
        	}
        	else if(TskIr::TestIrTouch(TskIr::IrCh::FR, 1600, 1200))
        	{
        	    SetLeds(0x0);
        		info_flag = 2;
        		DbgUartPutLine("start Omega pid test\n", true);
           		for(int j=0;j<8;j++)
           		{
                       actMsg = (TskAction::Act::ActType)(TskAction::Act::Back);
                       Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
                       Mailbox_pend(MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
           		}
            	break;
        	}
    	}
        Task_sleep(500);
        info_flag = 0;
#endif
        while(i < 512)
        {
            sprintf(dbgStr, "%4.3f,%4.3f\n",
                TskAction::Desire[i],
                TskAction::Info[i]);
            DbgUartPutLine(dbgStr, true);
            Task_sleep(10);
            i++;
        }
        if(i == 512) {
        	Task_sleep(100);
        	DbgUartPutLine("over\r\n", true);
        	i = 0;
        	sprintf(dbgStr, "%8.3f,%8.3f,%8.3f\r\n",
                  TskMotor::DistanceAcc,
                  TskMotor::DistanceAcc_en,
                  TskMotor::AngleAcc);
        	DbgUartPutLine(dbgStr, true);
        }
    }
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
    case MouseMode::PhysparamCorr:
        sprintf(str, "Mode PhysparamCorr.\r\n");
        break;
    case MouseMode::EncImuMonitor:
        sprintf(str, "Mode Enc & Imu Monitor.Touch Ir to start\r\n");
        break;
    case MouseMode::IrMonitor:
        sprintf(str, "Mode Ir Monitor.Touch Ir to start\r\n");
        break;
    case MouseMode::IrCorrection:
        sprintf(str, "Mode Ir Correction.Touch Ir to start.\r\n");
        break;
    case MouseMode::MotionCorrection:
        sprintf(str, "Mode MotionCorr.\r\n");
        break;
    case MouseMode::ListenUartCmd:
        sprintf(str, "Mode Listen Uart Command.\r\n");
        break;
    case MouseMode::ActionTest:
        sprintf(str, "Mode ActionTest.Touch Ir to start.\r\n");
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

    DbgUartPutLine("Hello from dbmouse!\r\n", true);

    Task_sleep(1000);
    TskAction::Init();
    TskMotor::Init();
    TskIr::Init();
    solve::Init();

    irMsg = TskIr::IrMsg::EnableEmitt;
    Mailbox_post(TskIr::MbCmd, &irMsg, BIOS_NO_WAIT);

    Task_sleep(500);
    DbgUartPutLine("Roll wheel to change mode...\r\n", true);

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
             case MouseMode::PhysparamCorr:
            	 PhysparamCorr();
                 break;
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
                UartCmd();
                break;
             case MouseMode::SolveTest:
             	solveTest();
             	break;
             case MouseMode::ActionTest:
             	actionTest();
             	break;
             case MouseMode::ClearMaze:
             	clearMazeInfo();
             	break;
             case MouseMode::Gaming1:
             	RushTest();
             	break;
             default:
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


