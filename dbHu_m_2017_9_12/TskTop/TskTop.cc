/*
 * top.cc
 *
 *  Created on: Jul 21, 2016
 *      Author: loywong
 */

#include "includes.h"

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

char dbgStr[80];

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
//    motMsg = TskMotor::MotorMsg::EnableAccl;
//    Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
//    motMsg = TskMotor::MotorMsg::EnableGyro;
//    Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
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
            DbgUartPutLine( "\n GYZero\t  GyroZ\t AYZero\t  AcclY\t EncVel\t KalVel\t DisAcc\tDisAcc_en\tAngAcc\n\r", true);
        }
        sprintf(dbgStr, "%7.3f\t%7.3f\t%7.3f\t%7.3f\t%7.4f\t%7.4f\t%8.3f%8.3f%8.3f\n\r",
                TskMotor::GyroZZero, TskMotor::AV,
                TskMotor::AcclYZero, TskMotor::AcclY,
                TskMotor::EncVel, TskMotor::LV,
                TskMotor::DistanceAcc,
				TskMotor::DistanceAcc_en,
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
        if(!(i++ & 0xf))
        {
            DbgUartPutLine( "\n AvgFwd:b\t   LFwd:b\t   RFwd:b\t  LSide:b\t  RSide:b\t  HdbyL\t  HdbyR\tHdbyFLR\t slInt\t srInt\n", true);
        }
        sprintf(dbgStr, "%7.4f:%1d\t%7.4f:%1d\t%7.4f:%1d\t%7.4f:%1d\t%7.4f:%1d\t%7.1f\t%7.1f\t%7.1f\t%4d\t%4d\n",
                .5f * (TskIr::IrDists.FLns + TskIr::IrDists.FRns), TskIr::IrBins.Fwd,
                TskIr::IrDists.FLns, TskIr::IrBins.FLns,
                TskIr::IrDists.FRns, TskIr::IrBins.FRns,
                TskIr::IrDists.LS, TskIr::IrBins.LS,
                TskIr::IrDists.RS, TskIr::IrBins.RS,
                TskIr::IrYaw.byLS * 180.f / 3.1415927f,
                TskIr::IrYaw.byRS * 180.f / 3.1415927f,
                TskIr::IrYaw.byFLR * 180.f / 3.1415927f,
				TskIr::IrInts.sl,
				TskIr::IrInts.sr
        );
//		if(!(i++ & 0xf))
//		{
//			DbgUartPutLine( "\nLFwd:b\t   RFwd:b\t  LSide:b\t  RSide:b\t\n", true);
//		}
//		sprintf(dbgStr, "%4d\t%4d\t%4d\t%4d\n",
//				TskIr::IrInts.fl, TskIr::IrInts.fr,
//				TskIr::IrInts.sl, TskIr::IrInts.sr
//		);
        DbgUartPutLine(dbgStr, true);
        Task_sleep(500);
    }

}

void actionTest()
{
//	int i=0,randint = 0;;
	SetLeds(0xf);
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

	DbgUartPutLine("verson 2.6.14\n", true);
#if testBaseAction
	actMsg = (TskAction::Act::ActType)(TskAction::Act::Start);
	Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
	Mailbox_pend(MbCmd, &end_Msg, BIOS_WAIT_FOREVER);

	//correct TBackR
//	actMsg = (TskAction::Act::ActType)(TskAction::Act::TBackR | TskAction::Act::Corr);
	//correct L90
//	actMsg = (TskAction::Act::ActType)(TskAction::Act::L90 | TskAction::Act::Corr);
	//correct R90
	actMsg = (TskAction::Act::ActType)(TskAction::Act::R90 | TskAction::Act::Corr);
	Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
	Mailbox_pend(MbCmd, &end_Msg, BIOS_WAIT_FOREVER);

	//correct Fwd
//	for(int i=0;i<2;i++)
//	{
//		actMsg = (TskAction::Act::ActType)(TskAction::Act::Fwd | TskAction::Act::Corr);
//		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
//		Mailbox_pend(MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
//	}

//	correct Stop
	actMsg = (TskAction::Act::ActType)(TskAction::Act::Stop | TskAction::Act::Corr);
	Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
	Mailbox_pend(MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
#endif
#if testRushAction
		actMsg = (TskAction::Act::ActType)(TskAction::Act::RushIn);
		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
		Mailbox_pend(MbCmd, &end_Msg, BIOS_WAIT_FOREVER);

//		actMsg = (TskAction::Act::ActType)(TskAction::Act::R45i); //combinate with L45o,L135o,TRush,TRush+R90t
//		actMsg = (TskAction::Act::ActType)(TskAction::Act::L45i); //combinate with R45o,R135o,TRush,TRush+L90t
		actMsg = (TskAction::Act::ActType)(TskAction::Act::L135i);
//		actMsg = (TskAction::Act::ActType)(TskAction::Act::R135i);
//		actMsg = (TskAction::Act::ActType)(TskAction::Act::L90r);
//		actMsg = (TskAction::Act::ActType)(TskAction::Act::R90r);
//		actMsg = (TskAction::Act::ActType)(TskAction::Act::L180);
//		actMsg = (TskAction::Act::ActType)(TskAction::Act::R180);

		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
		Mailbox_pend(MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
#if testRushOAct
//		actMsg = (TskAction::Act::ActType)(TskAction::Act::L45o);
//		actMsg = (TskAction::Act::ActType)(TskAction::Act::R45o);
//		actMsg = (TskAction::Act::ActType)(TskAction::Act::L135o);
//		actMsg = (TskAction::Act::ActType)(TskAction::Act::R135o);
//		actMsg = (TskAction::Act::ActType)(TskAction::Act::L90t);
//		actMsg = (TskAction::Act::ActType)(TskAction::Act::L90t);
//		actMsg = (TskAction::Act::ActType)(TskAction::Act::SRush);
//		actMsg = (TskAction::Act::ActType)(TskAction::Act::TRush);
		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
		Mailbox_pend(MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
#endif
		actMsg = (TskAction::Act::ActType)(TskAction::Act::RushOut);
		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
		Mailbox_pend(MbCmd, &end_Msg, BIOS_WAIT_FOREVER);

#endif
	while(true)
    {
        sprintf(dbgStr, "%8.3f%8.3f%8.3f\n\r",
                TskMotor::DistanceAcc,
				TskMotor::DistanceAcc_en,
				TskMotor::AngleAcc);
        DbgUartPutLine(dbgStr, true);
        Task_sleep(500);
#if 0
		if(i < 100)
		{
			randint = 2;

			if(randint == 2)
			{
				if(!TskIr::IrBins.LS)
				{
					actMsg = (TskAction::Act::ActType)(TskAction::Act::L90 | TskAction::Act::Corr);
					Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
					Mailbox_pend(MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
				}
				else if(!TskIr::IrBins.Fwd)
				{
					actMsg = (TskAction::Act::ActType)(TskAction::Act::Fwd | TskAction::Act::Corr);
					Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
					Mailbox_pend(MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
				}
				else if(!TskIr::IrBins.RS)
				{
					actMsg = (TskAction::Act::ActType)(TskAction::Act::R90 | TskAction::Act::Corr);
					Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
					Mailbox_pend(MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
				}
				else
				{
					actMsg = (TskAction::Act::ActType)(TskAction::Act::TBackR | TskAction::Act::Corr);
					Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
					Mailbox_pend(MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
				}

			}
			i++;
		}

		else if(i==100)
		{
			actMsg = (TskAction::Act::ActType)(TskAction::Act::Stop | TskAction::Act::Corr);
			Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
			Mailbox_pend(MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
			i++;
		}
#endif
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
        sprintf(dbgStr, "%2.1f %2.1f %7.3f %7.3f %7.3f %7.3f\n\t",TskAction::Info_LV[0],TskAction::Info_LV[1],
        		TskAction::Info_LV[2],TskAction::Info_LV[3],TskAction::Info_LV[4],TskAction::Info_LV[5]);
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
    GPIO_write(DBMOUSE_LED_0, (val & 0x8) ? DBMOUSE_LED_ON : DBMOUSE_LED_OFF);
    GPIO_write(DBMOUSE_LED_1, (val & 0x4) ? DBMOUSE_LED_ON : DBMOUSE_LED_OFF);
    GPIO_write(DBMOUSE_LED_2, (val & 0x2) ? DBMOUSE_LED_ON : DBMOUSE_LED_ON);
    GPIO_write(DBMOUSE_LED_3, (val & 0x1) ? DBMOUSE_LED_ON : DBMOUSE_LED_OFF);
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
    case MouseMode::Idle:
        sprintf(str, "Mode Idle.\n");
        break;
    case MouseMode::EncImuMonitor:
        sprintf(str, "Mode Enc & Imu Monitor.\n\tTouch Ir to start\n");
        break;
    case MouseMode::IrMonitor:
        sprintf(str, "Mode Ir Monitor.\n\tTouch Ir to start\n");
        break;
    case MouseMode::IrCorrection:
        sprintf(str, "Mode Ir Correction.\n\tTouch Ir to start.\n");
        break;
    case MouseMode::MotionCorrection:
        sprintf(str, "Mode MotionCorr.\n");
        break;
    case MouseMode::ListenUartCmd:
        sprintf(str, "Mode Listen Uart Command.\n");
        break;
    case MouseMode::ActionTest:
        sprintf(str, "Mode ActionTest.\n\tTouch Ir to start.\n");
        break;
    case MouseMode::SolveTest:
        sprintf(str, "Mode SolveTest.\n\tTouch Ir to start.\n");
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
        sprintf(str, "Mode Gaming1.\n\tTouch Ir & GO!\n");
        break;
    case MouseMode::Gaming2:
        sprintf(str, "Mode Gaming2.\n\tTouch Ir & GO!\n");
        break;
    case MouseMode::Gaming3:
        sprintf(str, "Mode Gaming3.\n\tTouch Ir & GO!\n");
        break;
    case MouseMode::Gaming4:
        sprintf(str, "Mode Gaming4.\n\tTouch Ir & GO!\n");
        break;
    case MouseMode::ClearMaze:
        sprintf(str, "Mode Clear Maze Info.\n\tTouch Ir start!\n");
        break;
    default:
        sprintf(str, "Undefined mode.\n");
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
    SetLeds(0x08);
    Task_sleep(100);
    SetLeds(0x00);

    DbgUartPutLine("Hello from dbmouse!\t\n", true);

    Task_sleep(1000);
    TskMotor::Init();
    TskIr::Init();
    TskAction::Init();
    solve::Init();

    irMsg = TskIr::IrMsg::EnableEmitt;
    Mailbox_post(TskIr::MbCmd, &irMsg, BIOS_NO_WAIT);

    Task_sleep(500);
    DbgUartPutLine("Roll wheel to change mode...\n", true);

    MouseMode::ModeType lastMode = Mode = MouseMode::Idle;
    SetLeds(Mode);
    dbgPutModeName(Mode);

    while(true)
    {
//    	Mode = (MouseMode::ModeType)3;
        Mode = (MouseMode::ModeType)(lroundf(TskMotor::DistanceAcc * 100.f) & 15);
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
            case MouseMode::Idle:
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


