/*
 * solve.cc
 *
 *  Created on: 2016Äê9ÔÂ1ÈÕ
 *      Author: db_Hu
 */
// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"

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

QueueHandle_t MbAct,MbTop;
Queue<TskAction::Act::ActType> *QAct;

int GetNextWall()
{
    bool rtn;
	TskAction::WallStatus wall;
	rtn = xQueuePend(MbAct, &wall, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
	return wall.msk;
}

void breakBeforeRun()
{
    bool rtn;
	TskMotor::MotorMsg::MsgType motMsg;

	vTaskDelay(3000);
	TskTop::SetLeds(0x0f);
	motMsg = TskMotor::MotorMsg::DisableMotors;
	rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
	motMsg = TskMotor::MotorMsg::DisableAccl;
	rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
	motMsg = TskMotor::MotorMsg::DisableGyro;
	rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
	motMsg = TskMotor::MotorMsg::DisableAcqZeros;
	rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
	vTaskDelay(1000);

	TskTop::SetLeds(0x00);
	while(true){
		if(TskIr::TestIrTouch(TskIr::IrCh::FL | TskIr::IrCh::FR, 1600, 1200))
		{
			TskTop::SetLeds(0x0f);

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

			motMsg = TskMotor::MotorMsg::EnableMotors;
			rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
	        configASSERT(rtn == pdPASS);

			vTaskDelay(100);// wait for all HW Enable
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
	rtn = xQueuePost(TskAction::MbCmd, &actMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
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
				rtn = xQueuePost(TskAction::MbCmd, &actMsg, portMAX_DELAY);
		        configASSERT(rtn == pdPASS);
			    TskAction::Info[0] = 1;
				nextWall = GetNextWall();
				break;
			case Micromouse::Turning::Left:
				actMsg = (TskAction::Act::ActType)(TskAction::Act::L90 | TskAction::Act::Corr);
				rtn = xQueuePost(TskAction::MbCmd, &actMsg, portMAX_DELAY);
		        configASSERT(rtn == pdPASS);
				TskAction::Info[0] = 2;
				nextWall = GetNextWall();
				break;
			case Micromouse::Turning::Right:
				actMsg = (TskAction::Act::ActType)(TskAction::Act::R90 | TskAction::Act::Corr);
				rtn = xQueuePost(TskAction::MbCmd, &actMsg, portMAX_DELAY);
		        configASSERT(rtn == pdPASS);
				TskAction::Info[0] = 3;
				nextWall = GetNextWall();
				break;
			case Micromouse::Turning::Backward:
				if(searchFinish)
				{
					actMsg = (TskAction::Act::ActType)(TskAction::Act::TBack | TskAction::Act::Corr);
					rtn = xQueuePost(TskAction::MbCmd, &actMsg, portMAX_DELAY);
			        configASSERT(rtn == pdPASS);
					searchFinish = false;
					breakBeforeRun();
					actMsg = (TskAction::Act::ActType)(TskAction::Act::Start | TskAction::Act::Corr);
					xQueuePost(TskAction::MbCmd, &actMsg, portMAX_DELAY);
					TskAction::Info[0] = 4;
					nextWall = GetNextWall();
				}
				else
				{
					actMsg = (TskAction::Act::ActType)(TskAction::Act::TBackR | TskAction::Act::Corr);
					rtn = xQueuePost(TskAction::MbCmd, &actMsg, portMAX_DELAY);
			        configASSERT(rtn == pdPASS);
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
    vTaskDelay(100);
    TskTop::SetLeds(0x00);
    m.GetQAct();
    while(QAct->Len())
    {
		if(!QAct->De(actMsg))
		    TskTop::SetLeds(0x0f);
		rtn = xQueuePost(TskAction::MbCmd, &actMsg, portMAX_DELAY);
        configASSERT(rtn == pdPASS);
		rtn = xQueuePend(TskTop::MbCmd, &end_Msg, portMAX_DELAY);
        configASSERT(rtn == pdPASS);
    }
}

#endif

void task(void *pvParameters)
{
	Solve::SolveType msg;
	bool flag,rtn;
	while(true){

		rtn = xQueuePend(MbTop, &msg, portMAX_DELAY);
        configASSERT(rtn == pdPASS);
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
    BaseType_t rtn;
//    BaseType_t hpsize;

    MbAct = xQueueCreate(4, sizeof(TskAction::WallStatus));
	configASSERT(MbAct);

    MbTop = xQueueCreate(4, sizeof(Solve::SolveType));
	configASSERT(MbTop);

//	hpsize = xPortGetFreeHeapSize();
#if RushTest
    QAct = new Queue<TskAction::Act::ActType, true>(256);
#endif
    rtn = xTaskCreate(task, (const portCHAR *)"SolveTask",
                tskStkSize, NULL, tskPrio, NULL);
//    hpsize = xPortGetFreeHeapSize();
    configASSERT(rtn == pdPASS);
}
}
