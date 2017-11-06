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

#define BACK_CORR_PID_P     100.0f

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

WallStatus cur_wall;
WallStatus nextWall;
float distZero;
bool reported;
// word: |        2        |                                        1                                             |     0   |
// byte: | 11..10 | 09..08 |                07                |                    06                  |  05..04  |  03..00 |
// bit:  | 31..16 | 15..00 | 31..28 |  27 |  26 |  25  |  24  | 23..20 |  19  |   18  |   17  |   16   |  15..00  |  31..00 |
// write:|  afth  |  aeth  |    X   | fim | eim | afim | aeim |    X   |   X  |   X   |   X   |    X   | clr fifo | wr data |
// read: |  afth  |  aeth  |    X   | fim | eim | afim | aeim |    X   | full | empty | afull | aempty |   usedw  | rx data |

//DbgInfo actionDbgInfo;

void GetWallInfo(WallStatus *wall)
{
	wall->fwd = TskIr::IrBins.Fwd;
	wall->left = TskIr::IrBins.LS;
	wall->right = TskIr::IrBins.RS;
}

int i = 0;

void WaitQEnd(void)
{
    bool rtn;

    while(TskMotor::QMotor->Len() > 0)
    {
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);
    }
}

void report(float pos, float distzero)
{
    bool rtn;
    //Get wall info
    if(!reported && TskMotor::DistanceAcc - distZero > pos)
    {
        GetWallInfo(&nextWall);
        rtn = xQueuePost(solve::MbAct, &nextWall, (TickType_t)0);
        configASSERT(rtn == pdPASS || rtn == errQUEUE_FULL);
        reported = 1;
    }
}

int MotionSeqLen(void)
{
    return (TskMotor::QMotor->Len());
}

int MotionCalcFwd(float v0, float v1, float s, float *vs)
{
    s *= 2.0f;
    float t = s / (v0 + v1);
    float a = (v1*v1-v0*v0)/s;
    int i, imax = t / PP::Ts;

//    t_a = Timestamp_get32();

    for(i = 1; i <= imax; i++)
    {
        vs[i - 1] = v0 + i * PP::Ts * a;
    }
    vs[i - 1] = v1;
//    t_a = Timestamp_get32() - t_a;
    return imax + 1;
}

int MotionCalcRotate(float ang, float mu, float *omgs)
{
    float beta = PP::g * mu / PP::W;
    float ha = ang > 0.0f ? ang / 2.0f : ang / -2.0f;
    float omg = 0.0f, tht = 0.0f;
    int n = 0, i;
    do
    {
        omg += beta * PP::Ts;
        omgs[n++] = ang > 0.0f ? omg : -omg;
        tht += omg * PP::Ts;
    }while(tht < ha);
    for(i = n - 2; i >= 0; i--)
    {
        omgs[2 * n - 2 - i] = omgs[i];
    }
    omgs[2 * n - 1] = 0.0f;
    return 2 * n;
}

int MotionCalcTurn(float v, float ang, float mu, float *omgs, float *requ)
{
    if(v < 0.001f && v > -0.001f)
        return MotionCalcRotate(ang, mu, omgs);

        float ha = ang > 0.0f ? ang / 2.0f : ang / -2.0f;
        float omg = 0.0f, tht = 0.0f, x = 0.0f, y = 0.0f;
        float k = sqrt(PP::W*PP::W - mu*mu * PP::H*PP::H);
        float tv = PP::W*PP::W/k/v * acosf(mu*PP::H/PP::W);
        float c = mu*PP::g*PP::W/v/k;
        float t, l;
        int n = 0, i, imax = tv/PP::Ts;
        for(i = 1; i <= imax; i++)
        {
            t = PP::Ts * i;
            l = t*v*k/PP::W/PP::W;
            omg = mu*c*PP::H/k*(cosf(l)-1.0f)+c*sinf(l);
            omgs[n++] = ang > 0.0f ? omg : -omg;
            tht += omg * PP::Ts;
            x += v * PP::Ts * cosf(tht);
            y += v * PP::Ts * sinf(tht);
            if(tht >= ha)
                break;
        }
        if(i > imax)   // large turn
        {
            do
            {
                omgs[n++] = ang > 0.0f ? omg : -omg;
                tht += omg * PP::Ts;
                x += v * PP::Ts * cosf(tht);
                y += v * PP::Ts * sinf(tht);
            } while(tht < ha);
        }
        for(i = n - 2; i >= 0; i--)
        {
            omgs[2 * n - 2 - i] = omgs[i];
        }
        omgs[2 * n - 1] = 0.0f;
         *requ = x / tanf(tht) + y;
        return 2 * n;
}

float actHeadingDirCorrBySideIrSide(WallStatus *wall)
{
    TskTop::SetLeds(0x2);
    float yaw;
    if(wall->left)
    {
        yaw = TskIr::IrYaw.byLS;
        if(wall->right)
        {
            yaw += TskIr::IrYaw.byRS;
            yaw *= 0.5f;
        }
    }
    else if(wall->right)
        yaw = TskIr::IrYaw.byRS;
    else
        yaw = 0.0f;

    TskTop::SetLeds(0x4);
    return actHDirPid->Tick(-yaw);
}

float actHeadingDirCorrByFwdIr(WallStatus *wall)
{

    float yaw;
    TskTop::SetLeds(0x2);
    yaw = TskIr::IrYaw.byFLR;
    TskTop::SetLeds(0x2);
    return actHDirPid->Tick(CP.FLRYAWERROR-yaw);
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

// detect whether wall disappeared, if yes, flush
int actFwdEndCorrBySideWallDisappear(WallStatus *wall, float v0, float v1,  float *d)
{
    int len, i;
//    bool rtn;
    float s;

    if((wall->left && (TskIr::IrBins.LS == 0)) || (wall->right && (TskIr::IrBins.RS == 0)))
    {
        TskTop::SetLeds(0x02);

    	*d = s = ((wall->left && (TskIr::IrBins.LS == 0))? CP.LFWDEND_DIST_W2NW : CP.RFWDEND_DIST_W2NW)
    	        + TskIr::SideWallDisPos - TskMotor::DistanceAcc;
        # if 0
                sprintf(dbgStr, "s:%6.3f\r\n", s);
                rtn = xQueuePost(TskPrint::MbCmd, dbgStr, (TickType_t)0);
                configASSERT(rtn == pdPASS);
                sprintf(dbgStr, "%6.3f %6.3f\r\n", TskIr::SideWallDisPos, TskMotor::DistanceAcc);
                rtn = xQueuePost(TskPrint::MbCmd, dbgStr, (TickType_t)0);
                configASSERT(rtn == pdPASS);
        #endif
        // clear motion seq
        TskMotor::QMotor->Clear();
        // add
        len = MotionCalcFwd(v0, v1, s, v_s);
        # if 0
                sprintf(dbgStr, "%d\r\n", len);
                rtn = xQueuePost(TskPrint::MbCmd, dbgStr, (TickType_t)0);
                configASSERT(rtn == pdPASS);
        #endif
        for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

        return 1;
    }
    return 0;
}

int actLRDistCorrByFwdIr(Act::ActType act, float pos)
{
    bool rtn;
    float dist;
    dist = (act == Act::L90 ? TskIr::IrDists.FLns : TskIr::IrDists.FRns);
//    int i = 0;
    if(cur_wall.fwd)
    {
        while(dist > pos)
        {
            rtn = xSemaphorePend(SemActTick, 2);
            configASSERT(rtn == pdPASS);
            # if 0
                if(i == 4){
                    sprintf(dbgStr, "%6.3f %6.3f\r\n", dist, pos);
                    rtn = xQueuePost(TskPrint::MbCmd, dbgStr, (TickType_t)0);
                    configASSERT(rtn == pdPASS);
                    i = 0;
                }
                else i++;
            #endif
            dist = (act == Act::L90 ? TskIr::IrDists.FLns : TskIr::IrDists.FRns);
        }
        TskTop::SetLeds(0x02);
        TskMotor::QMotor->Clear();
    }
    return 0;
}

void actCRush(void)
{
    int i, len;
    bool rtn;
    TskTop::SetLeds(0x2);
    len = MotionCalcFwd(SP.RushSpeed, SP.RushSpeed, PP::GridSize, v_s);
    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    TskTop::SetLeds(0x0);
    while(1){
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);

        report(PP::GridSize - PP::GetWallDist, distZero);

        if(MotionSeqLen() <= 4)
        {
            break;
        }
    }
}

//TODO
void actTRush(void){
    int i, len;
//    bool flag;
    bool rtn;
	TskTop::SetLeds(0x2);
    distZero = TskMotor::DistanceAcc;
    len = MotionCalcFwd(SP.RushSpeed, SP.RushSpeed, SP.TRushDist, v_s);

    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

//     if(TskIr::IrDists.LS < TskIr::IrDists.RS)	flag = 1;
//     else flag = 0;
//     while(TskMotor::DistanceAcc - distZero < SP.TRushDist)
//     {
//         rtn = xSemaphorePend(SemActTick, 2);
//         configASSERT(rtn == pdPASS);
// 		IRint = (flag?TskIr::IrDists.LS : TskIr::IrDists.RS);
// 		if(IMin < 0.001f) IMin = IRint;
// //		IRint = TskIr::IrDists.LS;
// 		if(fabs(IRint - (flag?SP.TRushL_MAX_DIST : SP.TRushR_MAX_DIST)) < SP.ERRDist
// 					&& IMin < (flag?SP.TRushL_MIN_DIST : SP.TRushR_MIN_DIST))
// //		if(IRint > SP.TRushL_MAX_DIST)
// 		{
// 			TskMotor::QMotor->Clear();
// 			TskTop::SetLeds(0xf);
// 			len = MotionCalcFwd(SP.RushSpeed, SP.RushSpeed, SP.TRushComDist, v_s);
// 			for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
// 			break;
// 		}
// 		if(IMin > IRint) IMin = IRint;
//     }
//     IMin = 0.f;
    TskTop::SetLeds(0x0);
    while(1){
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);

        report(SP.TRushDist - PP::GetWallDist, distZero);

        if(MotionSeqLen() <= 4)
        {
            break;
        }
    }
}

//start
void actRushIn(void){
	int i, len;
	bool rtn;
    TskTop::SetLeds(0x2);
    len = MotionCalcFwd(0.0f, SP.RushSpeed, SP.RushInAcclDist, v_s);
    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    TskTop::SetLeds(0x0);
    while(1){
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);

        report(SP.RushInAcclDist - PP::GetWallDist, distZero);

        if(MotionSeqLen() <= 4)
        {
            break;
        }
    }
}

//stop
void actRushOut(void){
	int i, len;

    TskTop::SetLeds(0x2);
	len = MotionCalcFwd(SP.RushSpeed, 0.0f, SP.RushOutAcclDist, v_s);
    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    TskTop::SetLeds(0x0);
    WaitQEnd();
    vTaskDelay(750);
}

//void actLR45io(bool corr, Act::ActType act)
//{
//    int i, len;
//    const float irSideDistAt45oBegin = 0.053f;
//
//    reported = 0;
//
//    TskTop::SetLeds(0x2);
//
//    GetWallInfo(&cur_wall);
//
//    bool isLeft = ((u32)act & (u32)Action::MaskDir) == (u32)Action::DirLeft;
//    bool isIn = ((u32)act & (u32)Action::MaskIo) == (u32)Action::IoIn;
//
//    TskMotor::VoAdj.Omega = 0.0f;
//
//    // calc turn seq
//    float requ;
//    osLen = MotionCalcTurn(pp.RushTurnSpeed,
//            isLeft ? pp.Pi * .25f : -pp.Pi * .25f,
//            pp.Mu, os, &requ);
//    // calc pre, post and total distance
//    float x = pp.GridSize * 0.5f - 0.414214f * requ;
//    configASSERT(x > 0.f);
//    float y = pp.GridSize * 0.707107f - 0.414214f * requ;
//    configASSERT(y > 0.f);
//    float straightPre  = isIn ? x + (isLeft ? ap.Xbcp.TurnAdj.L45iPre  : ap.Xbcp.TurnAdj.R45iPre)
//                              : y + (isLeft ? ap.Xbcp.TurnAdj.L45oPre  : ap.Xbcp.TurnAdj.R45oPre);
//    float straightPost = isIn ? y + (isLeft ? ap.Xbcp.TurnAdj.L45iPost : ap.Xbcp.TurnAdj.R45iPost)
//                              : x + (isLeft ? ap.Xbcp.TurnAdj.L45oPost : ap.Xbcp.TurnAdj.R45oPost);
//    float total = straightPre + osLen * (pp.RushTurnSpeed * pp.Ts) + straightPost;
//
//    // calc pre seq
//    vsLen = MotionCalcFwd(SP.RushSpeed, SP.RushSpeed, straightPre, vs);
//
//    WaitQEnd();
//    //update the grid info
//    distZero = TskMotor::DistanceAcc;
//
//    // write pre seq
//    for(i = 0; i < oslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
//    // calc post seq
//    vsLen = MotionCalcFwd(pp.RushTurnSpeed, pp.RushTurnSpeed, straightPost, vs);
//
//    // instantiate turn time corr
//    IrDistCh ch = isIn ? (isLeft ? IrDistCh::FL : IrDistCh::FR)
//                       : (isLeft ? IrDistCh::SR : IrDistCh::SL);
//    float irdist = isIn ? pp.CenterToWall + pp.GridSize - straightPre
//                        : irSideDistAt45oBegin - 0.5f * straightPre;
//    float adj = act == Action::L45i ? ap.Xscp.TurnTimeAdj.L45i :
//                act == Action::R45i ? ap.Xscp.TurnTimeAdj.R45i :
//                act == Action::L45o ? ap.Xscp.TurnTimeAdj.L45o :
//                act == Action::R45o ? ap.Xscp.TurnTimeAdj.R45o : 0.0f;
//    CorrTurnTime corrTurn(ch, irdist - adj, pp.RushTurnSpeed, os, osLen, vs, vsLen);
//
//    if(corr && (isIn ? wall.fwd : isLeft ? wall.right : wall.left)) // wait turn time
//    {
//        for(i = 0; i < osLen + vsLen; i++) MotionSeqWrite(pp.RushTurnSpeed, 0.0f);
//    }
//    else
//    {
//        for(i = 0; i < osLen; i++) MotionSeqWrite(pp.RushTurnSpeed, os[i]);
//        for(i = 0; i < vsLen; i++) MotionSeqWrite(vs[i], 0.0f);
//    }
//
//    while(true)
//    {
//        xSemaphoreTake(TskMotor::SemActTick, portMAX_DELAY);
//        float pos = TskMotor::DistAcc() - pos0;
//        if(corr)
//        {
//            corrTurn.Tick();
//        }
//        report.Tick(pos, nextWall);
//        if(MotionSeqLen() <= 4)
//        {
//            break;
//        }
//    }
//}
//
//void actLR45io(Action act, bool corr = false)
//{
//    const float irSideDistAt45oBegin = 0.053f;
//    using namespace TskIr;
//    WallStatus wall, nextWall;
//    getWall(wall);
//
//    bool isLeft = ((u32)act & (u32)Action::MaskDir) == (u32)Action::DirLeft;
//    bool isIn = ((u32)act & (u32)Action::MaskIo) == (u32)Action::IoIn;
//
//    TskMotor::VoAdj.Omega = 0.0f;
//
//    // calc turn seq
//    float requ;
//    osLen = MotionCalcTurn(pp.RushTurnSpeed,
//            isLeft ? pp.Pi * .25f : -pp.Pi * .25f,
//            pp.Mu, os, &requ);
//    // calc pre, post and total distance
//    float x = pp.GridSize * 0.5f - 0.414214f * requ;
//    configASSERT(x > 0.f);
//    float y = pp.GridSize * 0.707107f - 0.414214f * requ;
//    configASSERT(y > 0.f);
//    float straightPre  = isIn ? x + (isLeft ? ap.Xbcp.TurnAdj.L45iPre  : ap.Xbcp.TurnAdj.R45iPre)
//                              : y + (isLeft ? ap.Xbcp.TurnAdj.L45oPre  : ap.Xbcp.TurnAdj.R45oPre);
//    float straightPost = isIn ? y + (isLeft ? ap.Xbcp.TurnAdj.L45iPost : ap.Xbcp.TurnAdj.R45iPost)
//                              : x + (isLeft ? ap.Xbcp.TurnAdj.L45oPost : ap.Xbcp.TurnAdj.R45oPost);
//    float total = straightPre + osLen * (pp.RushTurnSpeed * pp.Ts) + straightPost;
//
//    Reporter report(total - pp.ReportAhead);
//
//    // calc pre seq
//    vsLen = MotionCalcFwd(pp.RushTurnSpeed, pp.RushTurnSpeed, straightPre, vs);
//    WaitSeqEmpty();
//    float pos0 = TskMotor::DistAcc();
//    // write pre seq
//    for(int i = 0; i < vsLen; i++) MotionSeqWrite(vs[i], 0.0f);
//    // calc post seq
//    vsLen = MotionCalcFwd(pp.RushTurnSpeed, pp.RushTurnSpeed, straightPost, vs);
//
//    // instantiate turn time corr
//    IrDistCh ch = isIn ? (isLeft ? IrDistCh::FL : IrDistCh::FR)
//                       : (isLeft ? IrDistCh::SR : IrDistCh::SL);
//    float irdist = isIn ? pp.CenterToWall + pp.GridSize - straightPre
//                        : irSideDistAt45oBegin - 0.5f * straightPre;
//    float adj = act == Action::L45i ? ap.Xscp.TurnTimeAdj.L45i :
//                act == Action::R45i ? ap.Xscp.TurnTimeAdj.R45i :
//                act == Action::L45o ? ap.Xscp.TurnTimeAdj.L45o :
//                act == Action::R45o ? ap.Xscp.TurnTimeAdj.R45o : 0.0f;
//    CorrTurnTime corrTurn(ch, irdist - adj, pp.RushTurnSpeed, os, osLen, vs, vsLen);
//
//    if(corr && (isIn ? wall.fwd : isLeft ? wall.right : wall.left)) // wait turn time
//    {
//        for(int i = 0; i < osLen + vsLen; i++) MotionSeqWrite(pp.RushTurnSpeed, 0.0f);
//    }
//    else
//    {
//        for(int i = 0; i < osLen; i++) MotionSeqWrite(pp.RushTurnSpeed, os[i]);
//        for(int i = 0; i < vsLen; i++) MotionSeqWrite(vs[i], 0.0f);
//    }
//
//    while(true)
//    {
//        xSemaphoreTake(TskMotor::SemActTick, portMAX_DELAY);
//        float pos = TskMotor::DistAcc() - pos0;
//        if(corr)
//        {
//            corrTurn.Tick();
//        }
//        report.Tick(pos, nextWall);
//        if(MotionSeqLen() <= 4)
//        {
//            break;
//        }
//    }
//}
//void actL135i(void){
//    float requ;
//    int i, vslen, oslen;
//    bool rtn;
//    TskMotor::OmgAdj = 0.0f; //actHDirPid->Reset();
//    distZero = TskMotor::DistanceAcc;
//    oslen = MotionCalcTurn(SP.RushSpeed, (float)PP::PI_2 * 3 / 2.f, PP::Mu, o_s, &requ);
//
//	TskTop::SetLeds(0x0);
//    float straightPre = (PP::GridSize - requ);
//    float straightPost = (PP::GridSize - requ + SP.TURNLI135_POST_ADJ);
//
//    vslen = MotionCalcFwd(SP.RushSpeed, SP.RushSpeed, straightPre, v_s);
//
//    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
//
//    //use the pillar info to correct
//    while(TskMotor::DistanceAcc - distZero < straightPre)
//    {
//        rtn = xSemaphorePend(SemActTick, 2);
//        configASSERT(rtn == pdPASS);
//		IRint = TskIr::IrDists.LS;
//		if(IMin < 0.001f) IMin = IRint;
//		if(fabs(IRint - SP.TURNLI135_MAX_DIST) < SP.ERRDist && IMin < SP.TURNLI135_MIN_DIST)
//		{
//			TskTop::SetLeds(0xf);
//			TskMotor::QMotor->Clear();
//		    vslen = MotionCalcFwd(SP.RushSpeed, SP.RushSpeed, SP.TURNLI135_PRE_ADJ, v_s);
//		    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
//			break;
//		}
//		if(IMin > IRint) IMin = IRint;
//    }
//    IMin = 0.f;
//    WaitQEnd();
//    for(i = 0; i < oslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(SP.RushSpeed,o_s[i]));
//    WaitQEnd();
//    vslen = MotionCalcFwd(SP.RushSpeed, SP.RushSpeed, straightPost, v_s);
//    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
//    WaitQEnd();
//}

void actL90r(void){
    float requ;
    int i, vslen, oslen;
    bool rtn;
    distZero = TskMotor::DistanceAcc;
    oslen = MotionCalcTurn(SP.RushSpeed, (float)PP::PI_2, PP::Mu, o_s, &requ);

	TskTop::SetLeds(0x0);
    float straightPre = (PP::GridSize - requ);

    float straightPost = (PP::GridSize - requ + SP.TURNL90R_POST_ADJ);

    vslen = MotionCalcFwd(SP.RushSpeed, SP.RushSpeed, straightPre, v_s);

    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    //use the pillar info to correct
    while(TskMotor::DistanceAcc - distZero < straightPre)
    {
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);
		IRint = TskIr::IrDists.LS;
		if(IMin < 0.001f) IMin = IRint;
		if(fabs(IRint - SP.TURNL90R_MAX_DIST) < SP.ERRDist && IMin < SP.TURNL90R_MIN_DIST)
		{
			TskMotor::QMotor->Clear();
			TskTop::SetLeds(0xf);
		    vslen = MotionCalcFwd(SP.RushSpeed, SP.RushSpeed, SP.TURNL90R_PRE_ADJ, v_s);
		    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
			break;
		}
		if(IMin > IRint) IMin = IRint;
    }
    IMin = 0.f;
	WaitQEnd();
    for(i = 0; i < oslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(SP.RushSpeed,o_s[i]));
    WaitQEnd();
    vslen = MotionCalcFwd(SP.RushSpeed, SP.RushSpeed, straightPost, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    WaitQEnd();
}

void actL180(void){
    float requ;
    int i, vslen, oslen;
    bool rtn;
    distZero = TskMotor::DistanceAcc;

    oslen = MotionCalcTurn(SP.T180Speed, (float)PP::PI, SP.TL180Mu, o_s, &requ);

	TskTop::SetLeds(0x0);
    float straightPre = (PP::GridSize - requ);

    float straightPost = (PP::GridSize - requ +SP.TURNL180_POST_ADJ);

    vslen = MotionCalcFwd(SP.RushSpeed, SP.T180Speed, 0.01f, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    vslen = MotionCalcFwd(SP.T180Speed, SP.T180Speed, straightPre-0.01f, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    //use the pillar info to correct
    while(TskMotor::DistanceAcc - distZero < straightPre)
    {
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);
		IRint = TskIr::IrDists.LS;
		if(IMin < 0.001f) IMin = IRint;
		if(fabs(IRint - SP.TURNL180_MAX_DIST) < SP.ERRDist && IMin <SP.TURNL180_MIN_DIST)
		{
			TskMotor::QMotor->Clear();
			TskTop::SetLeds(0xf);
		    vslen = MotionCalcFwd(SP.T180Speed, SP.T180Speed, SP.TURNL180_PRE_ADJ, v_s);
		    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
			break;
		}
		if(IMin > IRint) IMin = IRint;
    }
	IMin = 0.f;
    WaitQEnd();
    for(i = 0; i < oslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(SP.T180Speed,o_s[i]));
    WaitQEnd();

    vslen = MotionCalcFwd(SP.T180Speed, SP.T180Speed, straightPost-0.01f, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    vslen = MotionCalcFwd(SP.T180Speed, SP.T180Speed, 0.01f, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    WaitQEnd();
}

void actL135o(void){
    float requ;
    int i, vslen, oslen;
    bool rtn;
    distZero = TskMotor::DistanceAcc;
	TskTop::SetLeds(0x0);
    oslen = MotionCalcTurn(SP.RushSpeed, (float)PP::PI_2 * 3.f/ 2.f, PP::Mu, o_s, &requ);

    float straightPre = (PP::GridSize - requ);
    float straightPost = (PP::GridSize - requ + SP.TURNLO135_POST_ADJ);

    vslen = MotionCalcFwd(SP.RushSpeed, SP.RushSpeed, straightPre, v_s);

    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    //use the pillar info to correct
    while(TskMotor::DistanceAcc - distZero < straightPre)
    {
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);
		IRint = TskIr::IrDists.LS;
		if(IMin < 0.001f) IMin = IRint;
		if(fabs(IRint - SP.TURNLO135_MAX_DIST) < SP.ERRDist  && IMin < SP.TURNLO135_MIN_DIST)
		{
			TskTop::SetLeds(0xf);
			TskMotor::QMotor->Clear();
			vslen = MotionCalcFwd(SP.RushSpeed, SP.RushSpeed, SP.TURNLO135_PRE_ADJ, v_s);
		    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
			break;
		}
		if(IMin > IRint) IMin = IRint;
    }
    IMin = 0.f;
    WaitQEnd();
    for(i = 0; i < oslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(SP.RushSpeed,o_s[i]));
    WaitQEnd();

    vslen = MotionCalcFwd(SP.RushSpeed, SP.RushSpeed, straightPost, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    WaitQEnd();
}

void actL90t(void){
    float requ;
    int i, vslen, oslen;
    bool rtn;
    distZero = TskMotor::DistanceAcc;
    oslen = MotionCalcTurn(SP.RushSpeed, (float)PP::PI_2, PP::Mu, o_s, &requ);

	TskTop::SetLeds(0x0);
    float straightPre = (PP::GridSize - requ);

    float straightPost = (PP::GridSize - requ + SP.TURNL90T_POST_ADJ);

    vslen = MotionCalcFwd(SP.RushSpeed, SP.RushSpeed, straightPre, v_s);

    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    //use the pillar info to correct
    while(TskMotor::DistanceAcc - distZero < straightPre)
    {
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);
		IRint =TskIr::IrDists.LS;
		if(IMin < 0.001f) IMin = IRint;
		if(fabs(IRint - SP.TURNL90T_MAX_DIST) < SP.ERRDist && IMin < SP.TURNL90T_MIN_DIST)
		{
			TskMotor::QMotor->Clear();
			TskTop::SetLeds(0xf);
		    vslen = MotionCalcFwd(SP.RushSpeed, SP.RushSpeed, SP.TURNL90T_PRE_ADJ, v_s);
		    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
			break;
		}
		if(IMin > IRint) IMin = IRint;
    }
    IMin = 0.f;
	WaitQEnd();
    for(i = 0; i < oslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(SP.RushSpeed,o_s[i]));
    WaitQEnd();
    vslen = MotionCalcFwd(SP.RushSpeed, SP.RushSpeed, straightPost, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    WaitQEnd();
}

void actStart()
{
    int i, len;
    bool rtn;
    TskMotor::OmgAdj = 0.0f;
    reported = 0;

    TskTop::SetLeds(0x2);

    GetWallInfo(&cur_wall);

    len = MotionCalcFwd(0.0f, PP::SearchSpeed, PP::StartAcclDist, v_s);

    WaitQEnd();
    //update the grid info
    distZero = TskMotor::DistanceAcc;

    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    len = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed,
    		PP::StartTotalDist - PP::StartAcclDist, v_s);
    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    TskTop::SetLeds(0x0);
    while(1){
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);

        report(PP::StartTotalDist - PP::GetWallDist, distZero);

        if(MotionSeqLen() <= 4)
        {
            break;
        }
    }
}

unsigned int  actStop(bool corr)
{
    int i, len;
    bool rtn;
    float stopDist;
    TskMotor::OmgAdj = 0.0f;
    reported = 0;

    TskTop::SetLeds(0x2);
    GetWallInfo(&cur_wall);

    stopDist = PP::StopTotalDist-PP::StopAccDist;
    len = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed, stopDist, v_s);

    WaitQEnd();
    //update the grid info
    distZero = TskMotor::DistanceAcc;

    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    len = MotionCalcFwd(PP::SearchSpeed, 0.0f, PP::StopAccDist, v_s);
    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    TskTop::SetLeds(0x0);
    while(1){
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);

        //stop action don't report wall info
        // report(PP::StopTotalDist - PP::GetWallDist);

        if(MotionSeqLen() <= 4)
        {
            break;
        }
    }

    if(corr){
        TskTop::SetLeds(0x0);
        if(cur_wall.fwd)
        {
            i = 0;
            while(fabsf(CP.FWDDISADJ - (PP::CenterToWall - 0.5f * (TskIr::IrDists.FLns + TskIr::IrDists.FRns))) > 0.002f)
            {
                rtn = xSemaphorePend(SemActTick, 2);
                configASSERT(rtn == pdPASS);
                TskMotor::LvAdj = actFwdDistCorrByFwdIr(&cur_wall);
                if(i > 1000) break;
                else i++;
            }
            TskMotor::LvAdj = 0.0f;
            actHDirPid->Reset();
        }
    }

    vTaskDelay(750);
    return cur_wall.msk;
}

void actBack(bool corr, WallStatus *wall)
{
	int i, len;
	bool rtn;
	TskMotor::OmgAdj = 0.0f;
    reported = 0;

	float tht = wall->left? PP::PI_2 : -PP::PI_2;

	TskTop::SetLeds(0x2);

	len = MotionCalcRotate(tht, 0.5f * PP::Mu, o_s);

    WaitQEnd();
    //update the grid info
    distZero = TskMotor::DistanceAcc;

	for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(0.f,o_s[i]));

    if(corr){
        TskTop::SetLeds(0x0);
        if(wall->left || wall->right)
        {
            i = 0;
            while(fabsf(CP.FLRYAWERROR - TskIr::IrYaw.byFLR) > 0.5 * PP::PI / 180.f
                    || fabsf(CP.FWDDISADJ - (PP::CenterToWall - 0.5f * (TskIr::IrDists.FLns + TskIr::IrDists.FRns))) > 0.002f)
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
	for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(0.f,o_s[i]));

    TskTop::SetLeds(0x0);
    while(1){
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);
        // report(PP::StartTotalDist - PP::GetWallDist, distZero);

        if(MotionSeqLen() <= 4)
        {
            break;
        }
    }
}

void actRestart()
{
    int i, len;
    bool rtn;
    TskMotor::OmgAdj = 0.0f; //actHDirPid->Reset();
    reported = 0;

    TskTop::SetLeds(0x2);
    len = MotionCalcFwd(0.0f, PP::SearchSpeed, PP::RestartDist, v_s);

    WaitQEnd();
    //update the grid info
    distZero = TskMotor::DistanceAcc;

    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    len = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed, CP.RESTART_DIST_ADJ, v_s);
    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    TskTop::SetLeds(0x0);    
    while(1){
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);

        report(PP::RestartDist - PP::GetWallDist, distZero);

        if(MotionSeqLen() <= 4)
        {
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

void actLR90(bool corr, Act::ActType act)
{
    float requ;
    bool rtn;
    int i, vslen, oslen;
    
    reported = 0;

    TskTop::SetLeds(0x2);
    GetWallInfo(&cur_wall);

    oslen = MotionCalcTurn(PP::SearchSpeed, act == Act::L90 ?
            (float)PP::PI_2 : -(float)PP::PI_2, PP::Mu, o_s, &requ);

    float straightPre = (PP::GridSize / 2.f - requ)
            + (act == Act::L90 ? CP.TURNL90_PRE_ADJ : CP.TURNR90_PRE_ADJ);

    float straightPost = (PP::GridSize / 2.f - requ)
            + (act == Act::L90 ?  CP.TURNL90_POST_ADJ : CP.TURNR90_POST_ADJ);

    vslen = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed, straightPre, v_s);
    
    WaitQEnd();
    //update the grid info
    distZero = TskMotor::DistanceAcc;

    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    if(corr)
    {
        TskTop::SetLeds(0x0);
        actLRDistCorrByFwdIr(act, requ + PP::CenterToWall - (act == Act::L90 ? CP.TURNLWAIT_DIST_ADJ : CP.TURNRWAIT_DIST_ADJ));
    }
    for(i = 0; i < oslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(PP::SearchSpeed,o_s[i]));

    vslen = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed, straightPost, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
   
    TskTop::SetLeds(0x0);    
    while(1){
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);

        report(PP::GridSize - PP::GetWallDist, distZero);

        if(MotionSeqLen() <= 4)
        {
            break;
        }
    }
}

void actFwd(bool corr)
{
    int i, len;
    bool rtn;
    TskMotor::OmgAdj = 0.0f;
    reported = 0;

    float pos = PP::GridSize;

    TskTop::SetLeds(0x2);
    //update the grid info
    GetWallInfo(&cur_wall);

    len = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed, PP::GridSize, v_s);

    WaitQEnd();
    //update the grid info
    distZero = TskMotor::DistanceAcc;

    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    if(corr)
    {
        TskTop::SetLeds(0x0);
        if(cur_wall.left || cur_wall.right)
        {
            TskTop::SetLeds(0x0);
            while(TskMotor::DistanceAcc - distZero < CP.HEADING_BY_SIRSIDE_START_DIST)
            {
                rtn = xSemaphorePend(SemActTick, 2);
                configASSERT(rtn == pdPASS);
                TskMotor::OmgAdj = actHeadingDirCorrBySideIrSide(&cur_wall);
            }
            TskMotor::OmgAdj = 0.0f;
            actHDirPid->Reset();

            while(TskMotor::QMotor->Len() > 1)
            {
                TskTop::SetLeds(0x0);
                rtn = xSemaphorePend(SemActTick, 2);
                configASSERT(rtn == pdPASS);
                if(actFwdEndCorrBySideWallDisappear(&cur_wall,TskMotor::CurrentV,PP::SearchSpeed, &pos))
                {
                    TskTop::SetLeds(0x0);

                    break;
                }

            }

        }

        //TODO
        else    // centipede
        {
            TskTop::SetLeds(0x0);
            float lDis = TskIr::IrDists.LS;
            float rDis = TskIr::IrDists.RS;
            float dlMin = 1e38f, drMin = 1e38f;
            float lPos = -1.0f, rPos = -1.0f;
            while(TskMotor::DistanceAcc - distZero < CP.HEADING_BY_SIRFWD_BGNSTAT_POS)
            {
                rtn = xSemaphorePend(SemActTick, 2);
                configASSERT(rtn == pdPASS);
            }
            while(TskMotor::DistanceAcc - distZero < CP.HEADING_BY_SIRFWD_BEGIN_POS)
            {
                TskTop::SetLeds(0x2);
                rtn = xSemaphorePend(SemActTick, 2);
                configASSERT(rtn == pdPASS);
                if(lDis < dlMin) dlMin = lDis;
                if(rDis < drMin) drMin = rDis;
                if(lPos < 0.f)  // lpos not updated
                {
                    if(dlMin < PP::GridSize && lDis > dlMin + 0.008f)
                    {
                        lPos = TskMotor::DistanceAcc - distZero;
                        # if 0
                            sprintf(dbgStr, "dlMin:%6.3f, lDis:%6.3f\r\n", dlMin, lDis);
                            rtn = xQueuePost(TskPrint::MbCmd, dbgStr, (TickType_t)0);
                            configASSERT(rtn == pdPASS);
                        #endif
                    }

                }
                if(rPos < 0.f)  // rpos not updated
                {
                    if(drMin < PP::GridSize && rDis > drMin + 0.005f)
                    {
                        rPos = TskMotor::DistanceAcc - distZero;
                        # if 0
                            sprintf(dbgStr, "drMin:%6.3f rDis:%6.3f\r\n", drMin, rDis);
                            rtn = xQueuePost(TskPrint::MbCmd, dbgStr, (TickType_t)0);
                            configASSERT(rtn == pdPASS);
                        #endif
                    }

                }
                lDis = TskIr::IrDists.LS;
                rDis = TskIr::IrDists.RS;
            }
            float angErr;
            if(lPos > 0.0f && rPos > 0.0f)
            {
                TskTop::SetLeds(0x4);
                angErr = (lPos - rPos) * (PP::IrSizeAngle
                        / (CP.HEADING_BY_SIRFWD_END_POS - CP.HEADING_BY_SIRFWD_BEGIN_POS));
            }
            else
            {
                angErr = 0.0f;
            }
            float omgMax = angErr * (CP.CENTIPEDE_CORR_GAIN * 2.0f
                    / ((CP.HEADING_BY_SIRFWD_END_POS - CP.HEADING_BY_SIRFWD_BEGIN_POS) / PP::SearchSpeed));
            # if 1
                    sprintf(dbgStr, "omgMax:%6.3f lPos:%6.3f rPos:%6.3f\r\n", omgMax, lPos, rPos);
                    rtn = xQueuePost(TskPrint::MbCmd, dbgStr, (TickType_t)0);
                    configASSERT(rtn == pdPASS);
            #endif
            while(TskMotor::DistanceAcc - distZero < CP.HEADING_BY_SIRFWD_END_POS)
            {
                rtn = xSemaphorePend(SemActTick, 2);
                configASSERT(rtn == pdPASS);
                TskMotor::OmgAdj = -omgMax;
            }

            TskMotor::OmgAdj = 0.0f;
    //        actHDirPid->Reset();

            while(TskMotor::DistanceAcc - distZero < PP::GridSize || TskMotor::QMotor->Len() > 2)
            {
                rtn = xSemaphorePend(SemActTick, 2);
                configASSERT(rtn == pdPASS);
                TskMotor::OmgAdj = omgMax;

                report(PP::GridSize - PP::GetWallDist, distZero);
            }

            TskMotor::OmgAdj = 0.0f;
            actHDirPid->Reset();
        }
    }

    TskTop::SetLeds(0x0);
    while(1){
        rtn = xSemaphorePend(SemActTick, 2);
        configASSERT(rtn == pdPASS);

        report(PP::GridSize - PP::GetWallDist, distZero);

        if(MotionSeqLen() <= 4)
        {
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
        	actCurrAct = (Act::ActType)((uint32_t)act & 0xFF000000);
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
            // case Act::TBack:
            // 	if(flag)
            // 		actCorrStopBack();
            // 	else actTurnBack();
            // 	break;
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
//            case Act::CRush:
//            	actCRush();
//                break;
//            case Act::TRush:
//            	actTRush();
//                break;
//            case Act::RushIn:
//            	actRushIn();
//                break;
//            case Act::RushOut:
//            	actRushOut();
//                break;
//            case Act::L45i:
//            	actL45i();
//                break;
//            case Act::L45o:
//            	actL45o();
//                break;
//            case Act::R45i:
//            	actR45i();
//                break;
//            case Act::R45o:
//            	actR45o();
//                break;
//            case Act::L90r:
//            	actL90r();
//                break;
//            case Act::R90r:
//            	actR90r();
//                break;
//            case Act::L90t:
//            	actL90t();
//                break;
//            case Act::R90t:
//            	actR90t();
//                break;
//            case Act::L135i:
//            	actL135i();
//                break;
//            case Act::L135o:
//            	actL135o();
//                break;
//            case Act::R135i:
//            	actR135i();
//                break;
//            case Act::R135o:
//            	actR135o();
//                break;
//            case Act::L180:
//            	actL180();
//                break;
//            case Act::R180:
//            	actR180();
//                break;
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
