/*
 * solve.cc
 *
 *  Created on: 2016Äê9ÔÂ1ÈÕ
 *      Author: db_Hu
 */

#include <xdc/runtime/System.h>
#include <xdc/runtime/Assert.h>

#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include "action/action.h"
#include "mouse/mouse.h"
#include "mmaze/mmaze.h"
#include "solve/solve.h"
#include "TskTop/TskTop.h"
#include "../TskMotor/TskMotor.h"
#include "../TskIr/IrCorr.h"
#include "../TskIr/TskIr.h"

namespace solve
{
#define RushTest 0
const int tskPrio = 6;
const int tskStkSize = 4096;
Task_Params tskParams;
Task_Handle tsk;

Mailbox_Handle MbAct,MbTop;
Queue<TskAction::Act::ActType> *QAct;

int GetNextWall()
{
    bool rtn;
	TskAction::WallStatus wall;
	rtn=Mailbox_pend(MbAct, &wall, BIOS_WAIT_FOREVER);
    Assert_isTrue(rtn,NULL);
	return wall.msk;
}

void breakBeforeRun()
{
    bool rtn;
	TskMotor::MotorMsg::MsgType motMsg;

	Task_sleep(3000);
	TskTop::SetLeds(0x0f);
	motMsg = TskMotor::MotorMsg::DisableMotors;
	rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);
	motMsg = TskMotor::MotorMsg::DisableAccl;
	rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);
	motMsg = TskMotor::MotorMsg::DisableGyro;
	rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);
	motMsg = TskMotor::MotorMsg::DisableAcqZeros;
	rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);
	Task_sleep(1000);

	TskTop::SetLeds(0x00);
	while(true){
		if(TskIr::TestIrTouch(TskIr::IrCh::FL | TskIr::IrCh::FR, 1600, 1200))
		{
			TskTop::SetLeds(0x0f);
			TskMotor::DistanceAcc = 0.f;
			TskMotor::AngleAcc = 0.f;
			TskMotor::DistanceAcc_en = 0.f;

			motMsg = TskMotor::MotorMsg::EnableAcqZeros;
			rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	        Assert_isTrue(rtn,NULL);
			Task_sleep(1000);// wait for getting imu zeros & getting ir zeros

			motMsg = TskMotor::MotorMsg::EnableAccl;
			rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	        Assert_isTrue(rtn,NULL);

			motMsg = TskMotor::MotorMsg::EnableGyro;
			rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	        Assert_isTrue(rtn,NULL);

			motMsg = TskMotor::MotorMsg::EnableMotors;
			rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	        Assert_isTrue(rtn,NULL);

			Task_sleep(100);// wait for all HW Enable
			break;
		}
	}
}

int i=0;
void SearchAndRun(bool flag)
{
    bool rtn;
	TskAction::Act::ActType actMsg;
	Micromouse::GridCoor Target(8, 8);
    Micromouse::Mouse m(16, 16, Target);
    Micromouse::Turning::Type turning;
    Micromouse::WallType::Type wFwd = Micromouse::WallType::Open,
    		wLeft = Micromouse::WallType::Blocked, wRight = Micromouse::WallType::Blocked;
    bool searchFinish = false;
	if(flag) searchFinish = true;
    TskTop::SetLeds(0x00);
    turning = m.Step(wFwd, wLeft, wRight, &searchFinish);
#if 0
    wFwd = Micromouse::WallType::Open,
    		wLeft = Micromouse::WallType::Blocked, wRight = Micromouse::WallType::Blocked;
    turning = m.Step(wFwd, wLeft, wRight, &searchFinish);
    wFwd = Micromouse::WallType::Open,
    		wLeft = Micromouse::WallType::Blocked, wRight = Micromouse::WallType::Blocked;
    turning = m.Step(wFwd, wLeft, wRight, &searchFinish);
    wFwd = Micromouse::WallType::Open,
    		wLeft = Micromouse::WallType::Blocked, wRight = Micromouse::WallType::Blocked;
    turning = m.Step(wFwd, wLeft, wRight, &searchFinish);
    wFwd = Micromouse::WallType::Blocked,
    		wLeft = Micromouse::WallType::Blocked, wRight = Micromouse::WallType::Open;
    turning = m.Step(wFwd, wLeft, wRight, &searchFinish);
    wFwd = Micromouse::WallType::Open,
    		wLeft = Micromouse::WallType::Blocked, wRight = Micromouse::WallType::Blocked;
	turning = m.Step(wFwd, wLeft, wRight, &searchFinish);
    wFwd = Micromouse::WallType::Open,
    		wLeft = Micromouse::WallType::Blocked, wRight = Micromouse::WallType::Open;
	turning = m.Step(wFwd, wLeft, wRight, &searchFinish);
    wFwd = Micromouse::WallType::Open,
    		wLeft = Micromouse::WallType::Blocked, wRight = Micromouse::WallType::Open;
	turning = m.Step(wFwd, wLeft, wRight, &searchFinish);
#endif
    actMsg = (TskAction::Act::ActType)(TskAction::Act::Start | TskAction::Act::Corr);
	rtn=Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);
	int nextWall = GetNextWall();
    TskTop::SetLeds(0x0f);
	while(true){
        wFwd = (nextWall & 0x1) ? Micromouse::WallType::Blocked : Micromouse::WallType::Open;
        wLeft = (nextWall & 0x2) ? Micromouse::WallType::Blocked : Micromouse::WallType::Open;
        wRight = (nextWall & 0x4) ? Micromouse::WallType::Blocked : Micromouse::WallType::Open;
    	TskTop::SetLeds(nextWall);
		TskAction::Info[1] = nextWall;
		TskAction::Info[2] = TskIr::IrDists.FLns;
		TskAction::Info[3] = TskIr::IrDists.FRns;
		TskAction::Info[4] = TskIr::IrDists.LS;
		TskAction::Info[5] = TskIr::IrDists.RS;
        turning = m.Step(wFwd, wLeft, wRight, &searchFinish);
		switch(turning)
		{
			case Micromouse::Turning::Forward:
				actMsg = (TskAction::Act::ActType)(TskAction::Act::Fwd | TskAction::Act::Corr);
				rtn=Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
		        Assert_isTrue(rtn,NULL);
			    TskAction::Info[0] = 1;
				nextWall = GetNextWall();
				break;
			case Micromouse::Turning::Left:
				actMsg = (TskAction::Act::ActType)(TskAction::Act::L90 | TskAction::Act::Corr);
				rtn=Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
		        Assert_isTrue(rtn,NULL);
				TskAction::Info[0] = 2;
				nextWall = GetNextWall();
				break;
			case Micromouse::Turning::Right:
				actMsg = (TskAction::Act::ActType)(TskAction::Act::R90 | TskAction::Act::Corr);
				rtn=Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
		        Assert_isTrue(rtn,NULL);
				TskAction::Info[0] = 3;
				nextWall = GetNextWall();
				break;
			case Micromouse::Turning::Backward:
				if(searchFinish)
				{
					actMsg = (TskAction::Act::ActType)(TskAction::Act::TBack | TskAction::Act::Corr);
					rtn=Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
			        Assert_isTrue(rtn,NULL);
					searchFinish = false;
					breakBeforeRun();
					actMsg = (TskAction::Act::ActType)(TskAction::Act::Start | TskAction::Act::Corr);
					Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
					TskAction::Info[0] = 4;
					nextWall = GetNextWall();
				}
				else
				{
					actMsg = (TskAction::Act::ActType)(TskAction::Act::TBackR | TskAction::Act::Corr);
					rtn=Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
			        Assert_isTrue(rtn,NULL);
					TskAction::Info[0] = 5;
					nextWall = GetNextWall();
				}
				break;
			default:
				break;
		}
	}
}

#if RushTest

void Rush()
{
	TskAction::Act::ActType actMsg;
	TskAction::ActMsg::MsgType end_Msg;

	Micromouse::GridCoor Target(7, 7);
    Micromouse::Mouse m(16, 16, Target);
    TskTop::SetLeds(0x0f);
    Task_sleep(100);
    TskTop::SetLeds(0x00);
    m.GetQAct();
    while(QAct->Len())
    {
		if(!QAct->De(actMsg))
		    TskTop::SetLeds(0x0f);
		rtn=Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
		rtn=Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
        Assert_isTrue(rtn,NULL);
    }
}

#endif
void task(UArg arg0, UArg arg1)
{
	Solve::SolveType msg;
	bool flag,rtn;
	while(true){

		rtn=Mailbox_pend(MbTop, &msg, BIOS_WAIT_FOREVER);
        Assert_isTrue(rtn,NULL);
		switch(msg & 0xFF000000)
		{
			case Solve::ALGOTEST:
				flag = (bool)(msg & 0x00000001);
				{
					SearchAndRun(flag);
					TskTop::SetLeds(0x00);
				}
				break;
#if RushTest
			case Solve::RUSHTEST:
				Rush();
				break;
#endif
			default:
				break;
		}
	}
}

void Init()
{
//    Error_init(&eb);
    Task_Params_init(&tskParams);
    tskParams.priority = tskPrio;
    tskParams.stackSize = tskStkSize;

    MbAct = Mailbox_create(sizeof(TskAction::WallStatus), 4, NULL, NULL);
    MbTop = Mailbox_create(sizeof(Solve::SolveType), 4, NULL, NULL);
    if(MbAct == NULL || MbTop == NULL )
        System_abort("create solve::MbCmd failed.\n");

#if RushTest
    QAct = new Queue<TskAction::Act::ActType>(256);
#endif
    tsk = Task_create((Task_FuncPtr)task, &tskParams, NULL);
    if(tsk == NULL)
    {
        System_abort("Task solve failed");
    }
}
}
