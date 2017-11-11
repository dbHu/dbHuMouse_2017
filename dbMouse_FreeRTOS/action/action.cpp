/*
 * motion.cpp
 *
 *  Created on: Aug 5, 2014
 *      Author: loywong
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"

#include "../physparams.h"
#include "../TskMotor/TskMotor.h"
#include "../TskIr/TskIr.h"
#include "action/action.h"
#include "TskTop/DbgUart.h"
#include "TskTop/TskTop.h"
#include "Pid/pid.h"
#include "TskMotor/WheelEnc.h"
#include "solve/solve.h"
#include "action/correction.h"
#include "action/motion.h"
#include "PinConfig/pinout.h"

#define BACK_CORR_PID_P     100.0f
#define CENTIPEDE_CORR_GAIN 0.05f

RushParam   SP;
SeachParam  CP;

namespace TskAction
{

char dbgStr[128];
float v_s[512],o_s[512];
//float v_s[PP::SeqArrayLen],o_s[PP::SeqArrayLen];

const int tskPrio = 8;
const int tskStkSize = 2048;
//Error_Block eb;

ActMsg::MsgType end_msg;

volatile float Info[640],Desire[640];
float IRint,IMin;
QueueHandle_t MbCmd;

#define SIDEIR_CORR_PID_P   3.f
#define SIDEIR_CORR_PID_I   0.f
#define SIDEIR_CORR_PID_D   0.f
#define SIDEIR_CORR_PID_N   0.f

#define CORR_BACKCENTER_ADJ (wall->left? CP.LBACKCENTER_ADJ : CP.RBACKCENTER_ADJ)
Pid *actHDirPid;

void getWall(WallStatus &w)
{
    w.fwd = TskIr::IrBins.Fwd;
    w.left = TskIr::IrBins.LS;
    w.right = TskIr::IrBins.RS;
}

void WaitSeqEmpty(void)
{
    bool rtn;

    while(TskMotor::QMotor->Len() > 0)
    {
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);
    }
}

float actHeadingDirCorrByFwdIr(WallStatus *wall)
{

    float yaw;
    float rtn;
    TskTop::SetLeds(0x2);
    yaw = TskIr::IrYaw.byFLR;
    if(fabs(CP.FLRYAWERROR-yaw) < 0.5 * PP::PI / 180.f)
        rtn = 0.f;
    else rtn = CP.FLRYAWERROR-yaw;
    TskTop::SetLeds(0x2);
    return actHDirPid->Tick(rtn);
}

float actFwdDistCorrByFwdIr(WallStatus *wall)
{
#if ENABLE_CORRECTION == 0
    return 0.0f;
#endif
    TskTop::SetLeds(0x2);
    float dist;
//    bool rtn;
//    dist = 0.5f * (TskIr::IrDists.FLns + TskIr::IrDists.FRns) - PP::CenterToWall - CP.FWDDISERROR;
    dist = CP.FWDDISADJ - (PP::CenterToWall - 0.5f * (TskIr::IrDists.FLns + TskIr::IrDists.FRns));

    if(dist > 0.02f)
        dist = 0.02f;
    else if(dist < -0.02f)
        dist = -0.02f;
    if(fabs(dist) < 0.001f)
        dist = 0.f;
# if 0
        sprintf(dbgStr, "dist:%6.3f\r\n", dist);
        rtn = xQueuePost(TskPrint::MbCmd, dbgStr, (TickType_t)0);
        configASSERT(rtn == pdPASS);
#endif

    return (dist * CORR_BACKCENTER_ADJ);
}

void actStart()
{
	int i, len;
    bool rtn;
    float accl;
    WallStatus wall, nxtWall;

    TskTop::SetLeds(0x2);

    getWall(wall);

    /*init wall info report*/
    Reporter report(PP::StartTotalDist - PP::GetWallDist);

    len = MotionCalcFwd(0.0f, PP::SearchSpeed, PP::StartAcclDist, v_s, &accl);

    /*wait seq empty and record dist zero postion*/
    WaitSeqEmpty();
    float pos0 = TskMotor::DistanceAcc;

    for(i = 0; i < len; i++) MotionSeqWrite(v_s[i],0.f);

    len = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed,
    		PP::StartTotalDist - PP::StartAcclDist, v_s, &accl);
    for(i = 0; i < len; i++) MotionSeqWrite(v_s[i],0.f);

    TskTop::SetLeds(0x0);

    /*wait Qmotor left 4, if need correct, */
    while(1)
    {
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);

        float pos = TskMotor::DistanceAcc - pos0;
        report.Tick(pos, nxtWall);

        if(MotionSeqLen() <= 4)
        {
            TskTop::SetLeds(0x0);
            break;
        }
    }
}

unsigned int actStop(bool corr = false)
{
	int i, len;
    bool rtn;
	float stopDist;
	float accl;

    WallStatus wall;

    TskTop::SetLeds(0x2);

    getWall(wall);   //use for back corr
    CorrEndByFwd corrEnd(0.f, PP::CenterToWall + PP::StopAccDist + CP.STOPEND_DIST_ADJ, PP::StopAccDist);
    /*init wall info report*/
    Reporter report(PP::StopTotalDist - PP::GetWallDist);

    stopDist = PP::StopTotalDist-PP::StopAccDist;
    len = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed, stopDist, v_s, &accl);
    
    /*wait seq empty and record dist zero postion*/
    WaitSeqEmpty();
    float pos0 = TskMotor::DistanceAcc;

    for(i = 0; i < len; i++) MotionSeqWrite(v_s[i],0.f);

    stopDist = PP::StopAccDist;
    len = MotionCalcFwd(PP::SearchSpeed, 0.0f, stopDist, v_s, &accl);
    for(i = 0; i < len; i++) MotionSeqWrite(v_s[i],0.f);

    TskTop::SetLeds(0x0);

    /*wait Qmotor left 4, if need correct, */
    while(1)
    {
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);
        float pos = TskMotor::DistanceAcc - pos0;
        if(corr)
        {
            corrEnd.Tick(pos);
        }
        if(MotionSeqLen() <= 4)
        {
            TskTop::SetLeds(0x0);
            break;
        }
    }
    
    vTaskDelay(700);    //get zero
    return wall.msk;
}

void actBack(bool corr, WallStatus *wall)
{
    int i, len;
    bool rtn;

    TskTop::SetLeds(0x2);

    float tht = wall->left? PP::PI_2 : -PP::PI_2;

    len = MotionCalcRotate(tht, 0.5f * PP::Mu, o_s);

    /*wait seq empty and record dist zero postion*/
    WaitSeqEmpty();
    float pos0 = TskMotor::DistanceAcc;

    for(i = 0; i < len; i++) MotionSeqWrite(0.f,o_s[i]);

    if(corr){
        TskTop::SetLeds(0x0);
        WaitSeqEmpty();
        if(wall->left || wall->right)
        {
            i = 0;
            while(fabsf(CP.FLRYAWERROR - TskIr::IrYaw.byFLR) > 0.5 * PP::PI / 180.f
                    || fabsf(CP.FWDDISADJ - (PP::CenterToWall - 0.5f * (TskIr::IrDists.FLns + TskIr::IrDists.FRns))) > 0.001f)
            {
                rtn = xSemaphorePend(SemActTick, 2);
                configASSERT(rtn == pdPASS);
                TskMotor::OmgAdj = actHeadingDirCorrByFwdIr(wall);
                TskMotor::LvAdj = actFwdDistCorrByFwdIr(wall);
                if(i > 1000) break;
                else i++;
            }
            TskMotor::OmgAdj = 0.0f;
            TskMotor::LvAdj = 0.0f;
            actHDirPid->Reset();
        }
    }

    len = MotionCalcRotate(tht + (wall->left?CP.LRBACKANGLE_ADJ:-CP.LRBACKANGLE_ADJ), 0.5f * PP::Mu, o_s);
    for(i = 0; i < len; i++) MotionSeqWrite(0.f,o_s[i]);

    TskTop::SetLeds(0x0);
    while(1){
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);
        // report(PP::StartTotalDist - PP::GetWallDist, distZero);

        if(MotionSeqLen() <= 4)
        {
            TskTop::SetLeds(0x0);
            break;
        }
    }
}

void actRestart()
{
    int i, len;
    bool rtn;
    float accl;
    WallStatus wall, nxtWall;

    TskTop::SetLeds(0x2);

    /*init wall info report*/
    Reporter report(PP::RestartDist - PP::GetWallDist);

    getWall(wall);
    len = MotionCalcFwd(0.0f, PP::SearchSpeed, PP::RestartDist, v_s, &accl);

    /*wait seq empty and record dist zero postion*/
    WaitSeqEmpty();
    float pos0 = TskMotor::DistanceAcc;

    for(i = 0; i < len; i++) MotionSeqWrite(v_s[i],0.f);

    len = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed, CP.RESTART_DIST_ADJ, v_s, &accl);
    for(i = 0; i < len; i++) MotionSeqWrite(v_s[i],0.f);

    TskTop::SetLeds(0x0);
    while(1){
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);

        float pos = TskMotor::DistanceAcc - pos0;
        report.Tick(pos, nxtWall);

        if(MotionSeqLen() <= 4)
        {
            TskTop::SetLeds(0x0);
            break;
        }
    }
}

void actStopBackRestart(bool corr)
{
    WallStatus stopWall;
    stopWall.msk = actStop(corr);
    actBack(corr, &stopWall);
    actRestart();
}

void actFwd(bool corr)
{
    int i, len;
    bool rtn;
    float accl;
    WallStatus wall, nxtWall;

    TskTop::SetLeds(0x2);

    /*init wall info report*/
    Reporter report(PP::GridSize - PP::GetWallDist);

    getWall(wall);
    len = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed, PP::GridSize, v_s, &accl);

    CorrYawBySide corrYaw(CP.HEADING_BY_SIRSIDE_START_DIST, CP.HEADING_BY_SIRSIDE_STOP_DIST - 0.01f,
                          CP.HEADING_BY_SIRSIDE_STOP_DIST, 0.1f);//, 0.00f);
    //                    start        end    l app   r app
    CorrEndBySide corrEnd(0.000f, 0.6f * PP::GridSize,
            CP.LFWDEND_DIST_W2NW, CP.RFWDEND_DIST_W2NW, PP::SearchSpeed);

    CorrCentipede corrCent(CP.HEADING_BY_SIRFWD_BGNSTAT_POS, CP.HEADING_BY_SIRFWD_BEGIN_POS,
                           CP.HEADING_BY_SIRFWD_END_POS, PP::GridSize, accl);

    /*wait seq empty and record dist zero postion*/
    WaitSeqEmpty();
    float pos0 = TskMotor::DistanceAcc;

    for(i = 0; i < len; i++) MotionSeqWrite(v_s[i],0.f);
    TskTop::SetLeds(0x0);
    while(1){
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);
        float pos = TskMotor::DistanceAcc - pos0;
        if(corr)
        {
            corrYaw.Tick(pos);
            corrEnd.Tick(pos);
            corrCent.Tick(pos);
        }

        report.Tick(pos, nxtWall);

        if(MotionSeqLen() <= 4)
        {
            TskTop::SetLeds(0x0);
            break;
        }
    }

}

void actLR90(bool corr, Act::ActType act)
{
    float requ,total, accl;
    int i, vslen, oslen;
    bool rtn;
    WallStatus wall, nxtWall;

    TskTop::SetLeds(0x2);

    getWall(wall);

    oslen = MotionCalcTurn(PP::SearchSpeed, act == Act::L90 ?
    		(float)PP::PI_2 : -(float)PP::PI_2, PP::Mu, o_s, &requ);
    // instantiate turn time corr
    unsigned char ch = (act == Act::L90 ? DBMOUSE_IR_FL : DBMOUSE_IR_FR);

    float straightPre = (PP::GridSize / 2.f - requ)
            + (act == Act::L90 ? CP.TURNL90_PRE_ADJ : CP.TURNR90_PRE_ADJ);

    float straightPost = (PP::GridSize / 2.f - requ)
            + (act == Act::L90 ?  CP.TURNL90_POST_ADJ : CP.TURNR90_POST_ADJ);

    float adj = (act == Act::L90 ?  CP.TURNLWAIT_DIST_ADJ : CP.TURNRWAIT_DIST_ADJ);

    total = straightPre + oslen * PP::SearchSpeed * PP::Ts + straightPost;
    /*init wall info report*/
    Reporter report(total - PP::GetWallDist);

    /*wait seq empty and record dist zero postion*/
    WaitSeqEmpty();
    float pos0 = TskMotor::DistanceAcc;

    vslen = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed, straightPre, v_s, &accl);


    for(i = 0; i < vslen; i++) MotionSeqWrite(v_s[i],0.f);

    vslen = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed, straightPost, v_s, &accl);

    CorrTurnTime    corrTurn(ch, straightPre, PP::SearchSpeed, o_s, oslen, v_s, vslen);
    CorrYawByFwd    corrYaw(0.f, 0.005f);
    for(i = 0; i < oslen; i++) MotionSeqWrite(PP::SearchSpeed,o_s[i]);
    for(i = 0; i < vslen; i++) MotionSeqWrite(v_s[i],0.f);

    TskTop::SetLeds(0x0);
    while(1){
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);
        float pos = TskMotor::DistanceAcc - pos0;
        if(corr)
        {
            corrTurn.WallDis(pos, adj);
            corrYaw.Tick(pos);
        }

        report.Tick(pos, nxtWall);

        if(MotionSeqLen() <= 4)
        {
            TskTop::SetLeds(0x0);
            break;
        }
    }
}

void actRushStart()
{
    int i, len;
    bool rtn;
    float accl;
    WallStatus wall, nxtWall;

    TskTop::SetLeds(0x2);

    getWall(wall);

    /*init wall info report*/
    Reporter report(PP::RushInAcclDist - PP::GetWallDist);

    len = MotionCalcFwd(0.0f, SP.RushTurnSpeed, PP::RushInAcclDist, v_s, &accl);

    /*wait seq empty and record dist zero postion*/
    WaitSeqEmpty();
    float pos0 = TskMotor::DistanceAcc;

    for(i = 0; i < len; i++) MotionSeqWrite(v_s[i],0.f);

    TskTop::SetLeds(0x0);

    /*wait Qmotor left 4, if need correct, */
    while(1)
    {
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);

        float pos = TskMotor::DistanceAcc - pos0;
        report.Tick(pos, nxtWall);

        if(MotionSeqLen() <= 4)
        {
            TskTop::SetLeds(0x0);
            break;
        }
    }
}


void actRushStop()
{
    int i, len;
    bool rtn;
    float accl;
    WallStatus wall, nxtWall;

    TskTop::SetLeds(0x2);

    getWall(wall);

    /*init wall info report*/
    Reporter report(PP::RushStopAcclDist - PP::GetWallDist);

    len = MotionCalcFwd(SP.RushTurnSpeed, 0.f, PP::RushStopAcclDist, v_s, &accl);

    /*wait seq empty and record dist zero postion*/
    WaitSeqEmpty();
    float pos0 = TskMotor::DistanceAcc;

    for(i = 0; i < len; i++) MotionSeqWrite(v_s[i],0.f);

    TskTop::SetLeds(0x0);

//    TskTop::SetLeds(0x2);
//    len = MotionCalcFwd(0.f, SP.BackSpeed, PP::RushStopBackDist / 2.f, v_s);
//    for(i = 0; i < len; i++) MotionSeqWrite(v_s[i],0.f);
//
//    len = MotionCalcFwd(SP.BackSpeed, 0.f, PP::RushStopBackDist / 2.f, v_s);
//    for(i = 0; i < len; i++) MotionSeqWrite(v_s[i],0.f);
//
//    TskTop::SetLeds(0x0);
    /*wait Qmotor left 4, if need correct, */
    while(1)
    {
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);

        float pos = TskMotor::DistanceAcc - pos0;
        report.Tick(pos, nxtWall);

        if(MotionSeqLen() <= 4)
        {
            TskTop::SetLeds(0x0);
            break;
        }
    }
}

void actORush(bool corr, Act::ActType act)
{
    int len;
    bool rtn;
    float accl;
    WallStatus wall, nxtWall;

    TskTop::SetLeds(0x2);
    getWall(wall);

    bool accel = ((uint32_t)act & (uint32_t)Act::MaskAccl) == (uint32_t)Act::AcclPos;
    bool deacc = ((uint32_t)act & (uint32_t)Act::MaskAccl) == (uint32_t)Act::AcclNeg;
    float vend = accel ? SP.RushDiagSpeed :
                 deacc ? SP.RushTurnSpeed :
                         TskMotor::desire.Velocity;
    //                     start   rback    end   rbRatio
    CorrYawBySide corrYaw(0.035f, 0.070f, 0.090f);//, 0.00f);
    //                    start        end    l app   r app
    CorrEndBySide corrEnd(0.000f, 0.25f * PP::GridSize,
            SP.ORushEndLDist, SP.ORushEndRDist, vend);

    Reporter report(PP::GridSize - PP::GetWallDist);
    // calc seq
    len = MotionCalcFwd(TskMotor::desire.Velocity, vend, PP::GridSize, v_s, &accl);

    CorrCentipede corrCent(0.f, 0.03f, 0.05f, 0.07f, accl);

    WaitSeqEmpty(); // wait pre action seq over
    float pos0 = TskMotor::DistanceAcc;
    // write seq
    for(int i = 0; i < len; i++) MotionSeqWrite(v_s[i], 0.0f);

    TskTop::SetLeds(0x0);
    /*wait Qmotor left 4, if need correct, */
    while(1)
    {
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);

        float pos = TskMotor::DistanceAcc - pos0;
        report.Tick(pos, nxtWall);

        if(corr)
        {
            corrYaw.Tick(pos);
            // probability of success detection by TickORush() is larger then Tick()
            corrEnd.TickORush(pos);
            corrCent.Tick(pos);
        }

        if(MotionSeqLen() <= 4)
        {
            TskTop::SetLeds(0x0);
            // TskMotor::VoAdj.Omega = 0.f;
            break;
        }
    }
}

void actDRush(bool corr, Act::ActType act)
{
    int len;
    bool rtn;
    float accl;
    WallStatus wall, nxtWall;

    TskTop::SetLeds(0x2);
    getWall(wall);

    bool accel = ((uint32_t)act & (uint32_t)Act::MaskAccl) == (uint32_t)Act::AcclPos;
    bool deacc = ((uint32_t)act & (uint32_t)Act::MaskAccl) == (uint32_t)Act::AcclNeg;
    float vend = accel ? SP.RushDiagSpeed :
                 deacc ? SP.RushTurnSpeed :
                         TskMotor::desire.Velocity;

    CorrRushYaw corrYaw(0.000f, PP::RushDiagDist - 0.005f, PP::RushDiagDist, 0.75f);
    CorrDiagRushEnd corrEnd(PP::RushDiagDist * 0.5f, SP.DRushEndLDist, SP.DRushEndRDist);
    Reporter report(PP::RushDiagDist - PP::GetWallDist);
    // calc seq
    len = MotionCalcFwd(TskMotor::desire.Velocity, vend, PP::RushDiagDist, v_s, &accl);

    WaitSeqEmpty(); // wait pre action seq over
    float pos0 = TskMotor::DistanceAcc;
    // write seq
    for(int i = 0; i < len; i++) MotionSeqWrite(v_s[i], 0.0f);

    TskTop::SetLeds(0x0);
    /*wait Qmotor left 4, if need correct, */
    while(1)
    {
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);

        float pos = TskMotor::DistanceAcc - pos0;
        report.Tick(pos, nxtWall);

        if(corr)
        {
            corrYaw.Tick(pos);
            corrEnd.Tick(pos);
        }

        if(MotionSeqLen() <= 4)
        {
            TskTop::SetLeds(0x0);
            // TskMotor::VoAdj.Omega = 0.f;
            break;
        }
    }
}

void actLR45io(bool corr, Act::ActType act)
{
    bool rtn;
    int oslen, vslen;
    float accl;
    using namespace TskIr;
    WallStatus wall, nxtWall;
    
    TskTop::SetLeds(0x2);
    getWall(wall);

    bool isLeft =  ((uint32_t)act & (uint32_t)Act::MaskDir) == (uint32_t)Act::DirLeft;
    bool isIn = ((uint32_t)act & (uint32_t)Act::MaskIo) == (uint32_t)Act::IoIn;

    TskMotor::OmgAdj = 0.0f;

    // calc turn seq
    float requ;
    oslen = MotionCalcTurn(SP.RushTurnSpeed,
            isLeft ? PP::PI * .25f : -PP::PI * .25f,
            SP.RushMu, o_s, &requ);
    // calc pre, post and total distance
    float x = PP::GridSize * 0.5f - 0.414214f * requ;
    configASSERT(x > 0.f);
    float y = PP::GridSize * 0.707107f - 0.414214f * requ;
    configASSERT(y > 0.f);
    float straightPre  = isIn ? x + (isLeft ? SP.TURNLI45_PRE_ADJ  : SP.TURNRI45_PRE_ADJ)
                              : y + (isLeft ? SP.TURNLO45_PRE_ADJ  : SP.TURNRO45_PRE_ADJ);
    float straightPost = isIn ? y + (isLeft ? SP.TURNLI45_POST_ADJ : SP.TURNRI45_POST_ADJ)
                              : x + (isLeft ? SP.TURNLO45_POST_ADJ : SP.TURNRO45_POST_ADJ);
    float total = straightPre + oslen * (SP.RushTurnSpeed * PP::Ts) + straightPost;

    Reporter report(total - PP::GetWallDist);

    // calc pre seq
    vslen = MotionCalcFwd(SP.RushTurnSpeed, SP.RushTurnSpeed, straightPre, v_s, &accl);
    WaitSeqEmpty();

    float pos0 = TskMotor::DistanceAcc;
    // write pre seq
    for(int i = 0; i < vslen; i++) MotionSeqWrite(v_s[i], 0.0f);
    // calc post seq
    vslen = MotionCalcFwd(SP.RushTurnSpeed, SP.RushTurnSpeed, straightPost, v_s, &accl);

    // instantiate turn time corr
    unsigned int ch = isIn ? (isLeft ? DBMOUSE_IR_SL : DBMOUSE_IR_SR)
                           : (isLeft ? DBMOUSE_IR_SR : DBMOUSE_IR_SL);

    float adj = act == Act::L45i ? SP.TURNLI45TT_ADJ :
                act == Act::R45i ? SP.TURNRI45TT_ADJ :
                act == Act::L45o ? SP.TURNLO45TT_ADJ :
                act == Act::R45o ? SP.TURNRO45TT_ADJ : 0.0f;

    CorrTurnTime corrTurn(ch, straightPre, SP.RushTurnSpeed, o_s, oslen, v_s, vslen);

    if(!corr || !(!isIn && (isLeft ? wall.right : wall.left))){
        for(int i = 0; i < oslen; i++) MotionSeqWrite(SP.RushTurnSpeed, o_s[i]);
        for(int i = 0; i < vslen; i++) MotionSeqWrite(v_s[i], 0.0f);
    }

    TskTop::SetLeds(0x0);
    while(true)
    {
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);

        float pos = TskMotor::DistanceAcc - pos0;
        report.Tick(pos, nxtWall);

        if(corr)
        {
            if(isIn)  corrTurn.WallDis(pos, adj);
            else      corrTurn.Tick(adj);
        }
        if(MotionSeqLen() <= 4)
        {
            TskTop::SetLeds(0x0);
            break;
        }
    }
}

void actLRO90(bool corr, Act::ActType act)
{
    float requ, total;
    int i, vslen, oslen;
    bool rtn;
    float accl;
    WallStatus wall, nxtWall;

    TskTop::SetLeds(0x2);

    getWall(wall);

    oslen = MotionCalcTurn(SP.RushTurnSpeed, act == Act::L90o ?
            (float)PP::PI_2 : -(float)PP::PI_2, SP.RushMu, o_s, &requ);
    // instantiate turn time corr
    unsigned char ch = (act == Act::L90o ? DBMOUSE_IR_SL : DBMOUSE_IR_SR);

    float straightPre = (PP::GridSize - requ)
            + (act == Act::L90o ? SP.TURNLO90_PRE_ADJ : SP.TURNRO90_PRE_ADJ);

    float straightPost = (PP::GridSize - requ)
            + (act == Act::L90o ?  SP.TURNLO90_POST_ADJ : SP.TURNRO90_POST_ADJ);

    float adj = (act == Act::L90o ?  SP.TURNLO90TT_ADJ : SP.TURNRO90TT_ADJ);

    total = straightPre + oslen * SP.RushTurnSpeed * PP::Ts + straightPost;
    /*init wall info report*/
    Reporter report(total - PP::GetWallDist);

    /*wait seq empty and record dist zero postion*/
    WaitSeqEmpty();
    float pos0 = TskMotor::DistanceAcc;

    vslen = MotionCalcFwd(SP.RushTurnSpeed, SP.RushTurnSpeed, straightPre, v_s, &accl);
    for(i = 0; i < vslen; i++) MotionSeqWrite(v_s[i],0.f);

    vslen = MotionCalcFwd(SP.RushTurnSpeed, SP.RushTurnSpeed, straightPost, v_s, &accl);

    CorrTurnTime    corrTurn(ch, straightPre, SP.RushTurnSpeed, o_s, oslen, v_s, vslen);

    if(!corr){
        for(i = 0; i < oslen; i++) MotionSeqWrite(SP.RushTurnSpeed,o_s[i]);
        for(i = 0; i < vslen; i++) MotionSeqWrite(v_s[i],0.f);
    }

    TskTop::SetLeds(0x0);
    while(1){
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);
        float pos = TskMotor::DistanceAcc - pos0;
        if(corr)
        {
            corrTurn.WallDis(pos, adj);
        }

        report.Tick(pos, nxtWall);

        if(MotionSeqLen() <= 4)
        {
            TskTop::SetLeds(0x0);
            break;
        }
    }
}

void actLRD90(bool corr, Act::ActType act)
{
    float requ, total;
    int i, vslen, oslen;
    bool rtn;
    float accl;
    WallStatus wall, nxtWall;

    TskTop::SetLeds(0x2);

    getWall(wall);

    oslen = MotionCalcTurn(SP.RushTurnSpeed, act == Act::L90d ?
            (float)PP::PI_2 : -(float)PP::PI_2, SP.RushMu, o_s, &requ);
    // instantiate turn time corr
    unsigned char ch = (act == Act::L90d ? DBMOUSE_IR_FL : DBMOUSE_IR_FR);

    float straightPre = (PP::RushDiagDist - requ)
            + (act == Act::L90d ? SP.TURNLD90_PRE_ADJ : SP.TURNRD90_PRE_ADJ);

    float straightPost = (PP::RushDiagDist - requ)
            + (act == Act::L90d ?  SP.TURNLD90_POST_ADJ : SP.TURNRD90_POST_ADJ);

    float adj = (act == Act::L90d ?  SP.TURNLD90TT_ADJ : SP.TURNRD90TT_ADJ);

    total = straightPre + oslen * SP.RushTurnSpeed * PP::Ts + straightPost;
    /*init wall info report*/
    Reporter report(total - PP::GetWallDist);

    /*wait seq empty and record dist zero postion*/
    WaitSeqEmpty();
    float pos0 = TskMotor::DistanceAcc;

    vslen = MotionCalcFwd(SP.RushTurnSpeed, SP.RushTurnSpeed, straightPre, v_s, &accl);
    for(i = 0; i < vslen; i++) MotionSeqWrite(v_s[i],0.f);

    vslen = MotionCalcFwd(SP.RushTurnSpeed, SP.RushTurnSpeed, straightPost, v_s, &accl);

    CorrTurnTime    corrTurn(ch, straightPre, SP.RushTurnSpeed, o_s, oslen, v_s, vslen);

    if(!corr)
    {
        for(i = 0; i < oslen; i++) MotionSeqWrite(SP.RushTurnSpeed,o_s[i]);
        for(i = 0; i < vslen; i++) MotionSeqWrite(v_s[i],0.f);
    }

    TskTop::SetLeds(0x0);
    while(1){
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);
        float pos = TskMotor::DistanceAcc - pos0;
        if(corr)
        {
            corrTurn.WallDis(pos, adj);
        }

        report.Tick(pos, nxtWall);

        if(MotionSeqLen() <= 4)
        {
            TskTop::SetLeds(0x0);
            break;
        }
    }
}

void actLR135io(bool corr, Act::ActType act)
{
    bool rtn;
    int oslen, vslen;
    float accl;
    using namespace TskIr;
    WallStatus wall, nxtWall;

    TskTop::SetLeds(0x2);
    getWall(wall);

    bool isLeft =  ((uint32_t)act & (uint32_t)Act::MaskDir) == (uint32_t)Act::DirLeft;
    bool isIn = ((uint32_t)act & (uint32_t)Act::MaskIo) == (uint32_t)Act::IoIn;

    TskMotor::OmgAdj = 0.0f;

    // calc turn seq
    float requ;
    oslen = MotionCalcTurn(SP.RushTurnSpeed,
            isLeft ? PP::PI * .75f : -PP::PI * .75f,
            SP.RushMu, o_s, &requ);
    // calc pre, post and total distance
    float x = PP::GridSize * 1.5f - 2.414214f * requ;
    configASSERT(x > 0.f);
    float y = PP::GridSize * 1.414214f - 2.414214f * requ;
    configASSERT(y > 0.f);
    float straightPre  = isIn ? x + (isLeft ? SP.TURNLI135_PRE_ADJ  : SP.TURNRI135_PRE_ADJ)
                              : y + (isLeft ? SP.TURNLO135_PRE_ADJ  : SP.TURNRO135_PRE_ADJ);
    float straightPost = isIn ? y + (isLeft ? SP.TURNLI135_POST_ADJ : SP.TURNRI135_POST_ADJ)
                              : x + (isLeft ? SP.TURNLO135_POST_ADJ : SP.TURNRO135_POST_ADJ);
    float total = straightPre + oslen * (SP.RushTurnSpeed * PP::Ts) + straightPost;

    Reporter report(total - PP::GetWallDist);

    // calc pre seq
    vslen = MotionCalcFwd(SP.RushTurnSpeed, SP.RushTurnSpeed, straightPre, v_s, &accl);
    WaitSeqEmpty();

    float pos0 = TskMotor::DistanceAcc;
    // write pre seq
    for(int i = 0; i < vslen; i++) MotionSeqWrite(v_s[i], 0.0f);
    // calc post seq
    vslen = MotionCalcFwd(SP.RushTurnSpeed, SP.RushTurnSpeed, straightPost, v_s, &accl);

    // instantiate turn time corr
    unsigned int ch = isIn ? (isLeft ? DBMOUSE_IR_FL : DBMOUSE_IR_FR)
                           : (isLeft ? DBMOUSE_IR_SR : DBMOUSE_IR_SL);

    float adj = act == Act::L135i ? SP.TURNLI135TT_ADJ :
                act == Act::R135i ? SP.TURNRI135TT_ADJ :
                act == Act::L135o ? SP.TURNLO135TT_ADJ :
                act == Act::R135o ? SP.TURNRO135TT_ADJ : 0.0f;

    CorrTurnTime corrTurn(ch, straightPre, SP.RushTurnSpeed, o_s, oslen, v_s, vslen);

    if(!corr || !(isIn ? wall.fwd : isLeft ? wall.right : wall.left)){
        for(int i = 0; i < oslen; i++) MotionSeqWrite(SP.RushTurnSpeed, o_s[i]);
        for(int i = 0; i < vslen; i++) MotionSeqWrite(v_s[i], 0.0f);
    }
//    else if(!isIn)
//    {
//        for(int i = 0; i < oslen + vslen; i++) MotionSeqWrite(SP.RushTurnSpeed, 0.f);
//    }
    TskTop::SetLeds(0x0);
    while(true)
    {
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);

        float pos = TskMotor::DistanceAcc - pos0;
        report.Tick(pos, nxtWall);

        if(corr)
        {
            corrTurn.Tick(adj);
        }

        if(MotionSeqLen() <= 4)
        {
            TskTop::SetLeds(0x0);
            break;
        }
    }
}

void actLR180(bool corr, Act::ActType act)
{
    float requ, total;
    int i, vslen, oslen;
    bool rtn;
    float accl;
    WallStatus wall, nxtWall;

    TskTop::SetLeds(0x2);

    getWall(wall);

    oslen = MotionCalcTurn(SP.RushTurnSpeed, act == Act::L180 ?
            (float)PP::PI : -(float)PP::PI, (act == Act::L180 ? SP.TL180Mu : SP.TR180Mu), o_s, &requ);
    // instantiate turn time corr
    unsigned char ch = (act == Act::L180 ? DBMOUSE_IR_FL : DBMOUSE_IR_FR);

    float straightPre = (PP::GridSize - requ)
            + (act == Act::L180 ? SP.TURNL180_PRE_ADJ : SP.TURNR180_PRE_ADJ);

    float straightPost = (PP::GridSize - requ)
            + (act == Act::L180 ?  SP.TURNL180_POST_ADJ : SP.TURNR180_POST_ADJ);

    float adj = (act == Act::L180 ?  SP.TURNL180TT_ADJ : SP.TURNL180TT_ADJ);

    total = straightPre + oslen * SP.RushTurnSpeed * PP::Ts + straightPost;
    /*init wall info report*/
    Reporter report(total - PP::GetWallDist);

    /*wait seq empty and record dist zero postion*/
    WaitSeqEmpty();
    float pos0 = TskMotor::DistanceAcc;

    vslen = MotionCalcFwd(SP.RushTurnSpeed, SP.RushTurnSpeed, straightPre, v_s, &accl);
    for(i = 0; i < vslen; i++) MotionSeqWrite(v_s[i],0.f);

    vslen = MotionCalcFwd(SP.RushTurnSpeed, SP.RushTurnSpeed, straightPost, v_s, &accl);

    CorrTurnTime    corrTurn(ch, straightPre - adj, SP.RushTurnSpeed, o_s, oslen, v_s, vslen);

    for(i = 0; i < oslen; i++) MotionSeqWrite(SP.RushTurnSpeed,o_s[i]);
    for(i = 0; i < vslen; i++) MotionSeqWrite(v_s[i],0.f);

    TskTop::SetLeds(0x0);
    while(1){
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);
        float pos = TskMotor::DistanceAcc - pos0;
        if(corr)
        {
            corrTurn.Tick(straightPre - adj);
        }

        report.Tick(pos, nxtWall);

        if(MotionSeqLen() <= 4)
        {
            TskTop::SetLeds(0x0);
            break;
        }
    }
}

Act::ActType actCurrAct;

void task(void *pvParameters)
{
    WallStatus stopWall;

    actCurrAct = Act::Null;
    Act::ActType act;
    bool flag,rtn;

    actHDirPid = new Pid(
            SIDEIR_CORR_PID_P, SIDEIR_CORR_PID_I, SIDEIR_CORR_PID_D,
            SIDEIR_CORR_PID_N, PP::Ts, -120.0f, 120.0f);

    while(true)
    {
    	rtn = xQueuePend(MbCmd, &act, portMAX_DELAY);
    	configASSERT(rtn == pdPASS);
        {
//            actCurrWall.msk = ((INT32U)msg & 0x00700000) >> 20;
//            wall.msk = ((INT32U)msg & 0x00700000) >> 20;
        	actCurrAct = (Act::ActType)((uint32_t)act & 0xFFF00000);
        	flag = (bool)((uint32_t)act & 0x00000001);
//            actCorrsInfo.Reset();
            switch(actCurrAct)
            {
            case Act::Null:
                break;
            case Act::Start:
                actStart();
                break;
            case Act::Stop:
                stopWall.msk = actStop(flag);
                break;
            case Act::Back:
                actBack(flag, &stopWall);
                break;
            case Act::TBackR:
                actStopBackRestart(flag);
                break;
            case Act::Restart:
                actRestart();
                break;
            case Act::Fwd:
                actFwd(flag);
                break;
            case Act::L90:
            case Act::R90:
                actLR90(flag, actCurrAct);
                break;
            case Act::RushStart:
                actRushStart();
                break;
            case Act::RushStop:
                actRushStop();
                break;
            case Act::ORush:
            case Act::ORushAcc:
            case Act::ORushDea:
                 actORush(flag, actCurrAct);
                 break;
            case Act::DRush:
            case Act::DRushAcc:
            case Act::DRushDea:
                 actDRush(flag, actCurrAct);
                 break;
            case Act::L45i:
            case Act::R45i:
            case Act::L45o:
            case Act::R45o:
                actLR45io(flag, actCurrAct);
                break;
            case Act::L90o:
            case Act::R90o:
                actLRO90(flag, actCurrAct);
                break;
            case Act::L90d:
            case Act::R90d:
                actLRD90(flag, actCurrAct);
                break;
            case Act::L135i:
            case Act::R135i:
            case Act::L135o:
            case Act::R135o:
                actLR135io(flag, actCurrAct);
                break;
            case Act::L180:
            case Act::R180:
                actLR180(flag, actCurrAct);
                break;
            default:
                break;
            }

        }
		end_msg = ActMsg::Action_ed;
		rtn = xQueuePost(TskTop::MbCmd, &end_msg, (TickType_t)0);
        configASSERT(rtn == pdPASS);
    }
}

void Init()
{
    BaseType_t rtn;

    MbCmd = xQueueCreate(4, sizeof(Act::ActType));
    configASSERT(MbCmd);

//    Error_init(&eb);
    // Create tasks
    rtn = xTaskCreate(task, (const portCHAR *)"PrintTask",
                   tskStkSize, NULL, tskPrio, NULL);
    configASSERT(rtn == pdPASS);
}
}
