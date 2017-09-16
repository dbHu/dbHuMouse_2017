/*
 * motion.cpp
 *
 *  Created on: Aug 5, 2014
 *      Author: loywong
 */

#include <includes.h>

#define BACK_CORR_PID_P     100.0f
#define CENTIPEDE_CORR_GAIN 0.05f

namespace TskAction
{
float v_s[PP::SeqArrayLen],o_s[PP::SeqArrayLen];

const int tskPrio = 8;
const int tskStkSize = 1024;
Task_Handle tsk;
//Error_Block eb;

ActMsg::MsgType end_msg;

Info info;
volatile float Info_LV[512];
float IRint,IMin;
Mailbox_Handle MbCmd;

#define SIDEIR_CORR_PID_P   3.f
#define SIDEIR_CORR_PID_I   0.f
#define SIDEIR_CORR_PID_D   0.f
#define SIDEIR_CORR_PID_N   0.f

#define CORR_FWDEND_DIST_NW2W (cur_wall.right? CP::LFWDEND_DIST_NW2W : CP::RFWDEND_DIST_NW2W)
#define CORR_BACKCENTER_ADJ (wall->left? CP::LBACKCENTER_ADJ : CP::RBACKCENTER_ADJ)
#define CORR_BACKANGLE_ADJ (wall->left? CP::LBACKANGLE_LRDIFF : CP::RBACKANGLE_LRDIFF)
Pid *actHDirPid;

WallStatus cur_wall;
WallStatus nextWall;
float distZero;
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

void WaitQEnd(void)
{
    while(TskMotor::QMotor->Len() > 0)
    {
        if(!Semaphore_pend(SemActTick, 2))
            System_abort("pend SemActTick failed!\n");
//        if(i < 512)
//        Info_LV[i++] = TskIr::IrDists.FRns;
    }
}

int MotionCalcFwd(float v0, float v1, float s, float *vs)
{
    s *= 2.0f;
    float t = s / (v0 + v1);
    float a = (v1*v1-v0*v0)/s;
    int i, imax = t / PP::Ts;
    int t_a;
    t_a = Timestamp_get32();

    for(i = 1; i <= imax; i++)
    {
        vs[i - 1] = v0 + i * PP::Ts * a;
    }
    vs[i - 1] = v1;
    t_a = Timestamp_get32() - t_a;
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
//    static float lastV = 0.0f, lastAng = 0.0f, lastMu = 0.0f;
//    static float lastOmgs[256] = {0.0f}, lastRequ = 0.0f, lastCnt;

    if(v < 0.001f && v > -0.001f)
        return MotionCalcRotate(ang, mu, omgs);

//    if(lastV == v && lastAng == ang && lastMu == mu)
//    {
//        *requ = lastRequ;
//        memcpy(omgs, lastOmgs, lastCnt * sizeof(float));
//        return lastCnt;
//    }
//    else
//    {
//        lastV = v; lastAng = ang; lastMu = mu;

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
        /*lastRequ =*/ *requ = x / tanf(tht) + y;
        //lastCnt = 2 * n;
        //memcpy(lastOmgs, omgs, lastCnt * sizeof(float));
        return 2 * n;
//    }
}

float actHeadingDirCorrBySideIrSide(WallStatus *wall)
{
#if ENABLE_CORRECTION == 0
    return 0.0f;
#endif

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
//    if(yaw > 45 * PP::PI / 180.f)
//    	yaw = 45 * PP::PI / 180.f;
//    if(yaw < -45 * PP::PI / 180.f)
//    	yaw = -45 * PP::PI / 180.f;

    return actHDirPid->Tick(-yaw);
}

float actHeadingDirCorrByFwdIr(WallStatus *wall)
{
#if ENABLE_CORRECTION == 0
    return 0.0f;
#endif

    float yaw;
    yaw = TskIr::IrYaw.byFLR;
//
//    if(yaw > 10 * PP::PI / 180.f)
//    	yaw = 10 * PP::PI / 180.f;
//    if(yaw < -10 * PP::PI / 180.f)
//    	yaw = -10 * PP::PI / 180.f;

    return actHDirPid->Tick(CP::FLRYAWERROR-yaw);
}

float actFwdDisCorrByFwdIr(WallStatus *wall)
{
#if ENABLE_CORRECTION == 0
    return 0.0f;
#endif

    float dist;
//    dist = 0.5f * (TskIr::IrDists.FLns + TskIr::IrDists.FRns) - PP::CenterToWall - CP::FWDDISERROR;
    dist = CP::FWDDISADJ - (PP::CenterToWall - 0.5f * (TskIr::IrDists.FLns + TskIr::IrDists.FRns));

    if(dist > 0.005f)
    	dist = 0.005f;
    else if(dist < -0.005f)
    	dist = -0.005f;
    return (dist * CORR_BACKCENTER_ADJ);
}

// detect whether wall disappeared, if yes, flush
int actFwdEndCorrBySideWallDisappear(WallStatus *wall, float v0, float v1,  float d)
{
#if ENABLE_CORRECTION == 0
    return 0;
#endif

    int len, i;
    float s;
    if((wall->left && TskIr::IrBins.LS == 0) || (wall->right && TskIr::IrBins.RS == 0))
    {

    	s = (TskIr::IrBins.LS?CP::LFWDEND_DIST_W2NW : CP::RFWDEND_DIST_W2NW);
        // clear motion seq
        TskMotor::QMotor->Clear();
        // add
        len = MotionCalcFwd(v0, v1, s, v_s);
        for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

#if DBG_PRINT_ACT_INFO > 0
        sprintf(actionDbgString, "\tFwdEnd@: %dmm\n", (int)(d * 1000.0f));
        DbgUartPutLine(actionDbgString);
#endif

        return 1;
    }
    return 0;
}

void actStart()
{
	int i, len;
    len = MotionCalcFwd(0.0f, PP::SearchSpeed, PP::StartAcclDist, v_s);
    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    len = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed,
    		PP::StartTotalDist - PP::StartAcclDist, v_s);
    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    TskTop::SetLeds(0x00);
    WaitQEnd();
    TskTop::SetLeds(0x01);
}

unsigned int actStop()
{
	int i, len;
	float stopDist;

    TskTop::SetLeds(0x02);
    GetWallInfo(&cur_wall);
    stopDist = PP::StopTotalDist-PP::StopAccDist;
    len = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed, stopDist, v_s);
    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    stopDist = PP::StopAccDist;
    len = MotionCalcFwd(PP::SearchSpeed, 0.0f, stopDist, v_s);
    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    WaitQEnd();
    TskTop::SetLeds(0x03);
    Task_sleep(750);
    return cur_wall.msk;
}

void actBack()
{
    int i, len;
    //TskMotor::OmgAdj = 0.0f; //actHDirPid->Reset();

    float tht =  PP::PI / 2.f;

    len = MotionCalcRotate(tht, 0.5f * PP::Mu, o_s);
    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(0.f,o_s[i]));
    WaitQEnd();
    len = MotionCalcRotate(tht, 0.5 * PP::Mu, o_s);
    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(0.f,o_s[i]));
    WaitQEnd();
   // Task_sleep(1000);
}

void actRestart()
{
    int i, len;
    //TskMotor::OmgAdj = 0.0f; //actHDirPid->Reset();

    len = MotionCalcFwd(0.0f, PP::SearchSpeed, PP::RestartDist + CP::RESTART_DIST_ADJ, v_s);
    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    WaitQEnd();
}

void actTurnBack()
{
	actStop();
	actBack();
	actRestart();
}

void actFwd()
{
    int i, len;
    len = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed, PP::GridSize, v_s);
    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    WaitQEnd();
}

void actLR90(Act::ActType act)
{
    float requ;
    int i, vslen, oslen;
    TskMotor::OmgAdj = 0.0f; //actHDirPid->Reset();

    oslen = MotionCalcTurn(PP::SearchSpeed, act == Act::L90 ?
    		(float)PP::PI_2 : -(float)PP::PI_2, PP::Mu, o_s, &requ);

    float straightPre = (PP::GridSize / 2.f - requ)
            + (act == Act::L90 ? CP::TURNL90_PRE_ADJ : CP::TURNR90_PRE_ADJ);

    float straightPost = (PP::GridSize / 2.f - requ)
            + (act == Act::L90 ?  CP::TURNL90_POST_ADJ : CP::TURNR90_POST_ADJ);

    vslen = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed, straightPre, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    for(i = 0; i < oslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(PP::SearchSpeed,o_s[i]));

    vslen = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed, straightPost, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    WaitQEnd();
}

void actCRush(void)
{
    int i, len;
    len = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, PP::GridSize, v_s);
    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    WaitQEnd();
}
//TODO
void actTRush(void){
    int i, len;
    bool flag;
	TskTop::SetLeds(0x0);
    distZero = TskMotor::DistanceAcc;
    len = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, SP::TRushDist, v_s);

    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    if(TskIr::IrDists.LS < TskIr::IrDists.RS)	flag = 1;
    else flag = 0;
    while(TskMotor::DistanceAcc - distZero < SP::TRushDist)
    {
		if(!Semaphore_pend(SemActTick, 2))
			System_abort("pend SemActTick failed!\n");
		IRint = (flag?TskIr::IrDists.LS : TskIr::IrDists.RS);
		if(IMin < 0.001f) IMin = IRint;
//		IRint = TskIr::IrDists.LS;
		if(fabs(IRint - (flag?SP::TRushL_MAX_DIST : SP::TRushR_MAX_DIST)) < SP::ERRDist
					&& IMin < (flag?SP::TRushL_MIN_DIST : SP::TRushR_MIN_DIST))
//		if(IRint > SP::TRushL_MAX_DIST)
		{
			TskMotor::QMotor->Clear();
			TskTop::SetLeds(0xf);
			len = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, SP::TRushComDist, v_s);
			for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
			break;
		}
		if(IMin > IRint) IMin = IRint;
    }
    IMin = 0.f;
    WaitQEnd();
}
//start
void actRushIn(void){
	int i, len;
    len = MotionCalcFwd(0.0f, SP::RushSpeed, SP::RushInAcclDist, v_s);
    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    WaitQEnd();
}
//stop
void actRushOut(void){
	int i, len;

	len = MotionCalcFwd(SP::RushSpeed, 0.0f, SP::RushOutAcclDist, v_s);
    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    WaitQEnd();
    Task_sleep(750);
}

void actL45i(void){
    float requ;
    int i, vslen, oslen;
    distZero = TskMotor::DistanceAcc;
    oslen = MotionCalcTurn(SP::RushSpeed, (float)PP::PI_2 / 2.f, PP::Mu, o_s, &requ);

    float straightPre = (PP::GridSize - requ - SP::TURNLI45_PRE_ADJ);
    float straightPost = (PP::GridSize - requ + SP::TURNLI45_POST_ADJ);

	TskTop::SetLeds(0x0);
    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, straightPre, v_s);

    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    //use the pillar info to correct
    while(TskMotor::DistanceAcc - distZero < straightPre)
    {
        if(!Semaphore_pend(SemActTick, 2))
            System_abort("pend SemActTick failed!\n");
		IRint = TskIr::IrDists.LS;
		if(IMin < 0.001f) IMin = IRint;
		if(fabs(IRint - SP::TURNLI45_MAX_DIST) < SP::ERRDist && IMin < SP::TURNLI45_MIN_DIST)
		{
			TskMotor::QMotor->Clear();
			TskTop::SetLeds(0xf);
			vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, SP::TURNLI45_PRE_ADJ, v_s);
		    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
			break;
		}
	   if(IMin > IRint) IMin = IRint;
    }
    IMin = 0.f;
    WaitQEnd();
    for(i = 0; i < oslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(SP::RushSpeed,o_s[i]));
    WaitQEnd();

    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, straightPost, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    WaitQEnd();
}

void actR45i(void){
    float requ;
    int i, vslen, oslen;
    TskMotor::OmgAdj = 0.0f; //actHDirPid->Reset();
    distZero = TskMotor::DistanceAcc;
    oslen = MotionCalcTurn(SP::RushSpeed, (float)-PP::PI_2 / 2.f, PP::Mu, o_s, &requ);

	TskTop::SetLeds(0x0);
    float straightPre = (PP::GridSize - requ - 1.8 * SP::TURNRI45_PRE_ADJ);
    float straightPost = (PP::GridSize - requ + SP::TURNRI45_POST_ADJ);

    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, straightPre, v_s);

    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    //use the pillar info to correct
    while(TskMotor::DistanceAcc - distZero < straightPre)
    {
        if(!Semaphore_pend(SemActTick, 2))
            System_abort("pend SemActTick failed!\n");
		IRint = TskIr::IrDists.RS;
		if(IMin < 0.001f) IMin = IRint;
		if(fabs(IRint - SP::TURNRI45_MAX_DIST) < SP::ERRDist && IMin < SP::TURNRI45_MIN_DIST)
		//if(!IRint && IRmax > SP::TURNRI45_MAX_IR)
		{
			TskMotor::QMotor->Clear();
			TskTop::SetLeds(0xf);
		    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, SP::TURNRI45_PRE_ADJ, v_s);
		    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
			break;
		}
		if(IMin > IRint) IMin = IRint;
    }
    IMin = 0.f;
    for(i = 0; i < oslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(SP::RushSpeed,o_s[i]));
    WaitQEnd();
    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, straightPost, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    WaitQEnd();
}

void actL135i(void){
    float requ;
    int i, vslen, oslen;
    TskMotor::OmgAdj = 0.0f; //actHDirPid->Reset();
    distZero = TskMotor::DistanceAcc;
    oslen = MotionCalcTurn(SP::RushSpeed, (float)PP::PI_2 * 3 / 2.f, PP::Mu, o_s, &requ);

	TskTop::SetLeds(0x0);
    float straightPre = (PP::GridSize - requ);
    float straightPost = (PP::GridSize - requ + SP::TURNLI135_POST_ADJ);

    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, straightPre, v_s);

    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    //use the pillar info to correct
    while(TskMotor::DistanceAcc - distZero < straightPre)
    {
        if(!Semaphore_pend(SemActTick, 2))
            System_abort("pend SemActTick failed!\n");
		IRint = TskIr::IrDists.LS;
		if(IMin < 0.001f) IMin = IRint;
		if(fabs(IRint - SP::TURNLI135_MAX_DIST) < SP::ERRDist && IMin < SP::TURNLI135_MIN_DIST)
		{
			TskTop::SetLeds(0xf);
			TskMotor::QMotor->Clear();
		    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, SP::TURNLI135_PRE_ADJ, v_s);
		    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
			break;
		}
		if(IMin > IRint) IMin = IRint;
    }
    IMin = 0.f;
    WaitQEnd();
    for(i = 0; i < oslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(SP::RushSpeed,o_s[i]));
    WaitQEnd();
    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, straightPost, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    WaitQEnd();
}

void actR135i(void){
    float requ;
    int i, vslen, oslen;
    TskMotor::OmgAdj = 0.0f; //actHDirPid->Reset();
    distZero = TskMotor::DistanceAcc;
    oslen = MotionCalcTurn(SP::RushSpeed, (float)-PP::PI_2 * 3 / 2.f, PP::Mu, o_s, &requ);

	TskTop::SetLeds(0x0);
    float straightPre = (PP::GridSize - requ);
    float straightPost = (PP::GridSize - requ + SP::TURNRI135_POST_ADJ);

    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, straightPre, v_s);

    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    //use the pillar info to correct
    while(TskMotor::DistanceAcc - distZero < straightPre)
    {
        if(!Semaphore_pend(SemActTick, 2))
            System_abort("pend SemActTick failed!\n");
		IRint = TskIr::IrDists.RS;
		if(IMin < 0.001f) IMin = IRint;
		if(fabs(IRint - SP::TURNRI135_MAX_DIST) < SP::ERRDist && IMin < SP::TURNRI135_MIN_DIST)
		//if(IRint && IRmax > SP::TURNRI135_MAX_IR)
		{
			TskMotor::QMotor->Clear();
			TskTop::SetLeds(0xf);
		    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, SP::TURNRI135_PRE_ADJ, v_s);
		    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
			break;
		}
		if(IMin > IRint) IMin = IRint;
    }
    IMin = 0.f;
    for(i = 0; i < oslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(SP::RushSpeed,o_s[i]));
    WaitQEnd();
    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, straightPost, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    WaitQEnd();
}

void actL90r(void){
    float requ;
    int i, vslen, oslen;
    distZero = TskMotor::DistanceAcc;
    oslen = MotionCalcTurn(SP::RushSpeed, (float)PP::PI_2, PP::Mu, o_s, &requ);

	TskTop::SetLeds(0x0);
    float straightPre = (PP::GridSize - requ);

    float straightPost = (PP::GridSize - requ + SP::TURNL90R_POST_ADJ);

    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, straightPre, v_s);

    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    //use the pillar info to correct
    while(TskMotor::DistanceAcc - distZero < straightPre)
    {
        if(!Semaphore_pend(SemActTick, 2))
            System_abort("pend SemActTick failed!\n");
		IRint = TskIr::IrDists.LS;
		if(IMin < 0.001f) IMin = IRint;
		if(fabs(IRint - SP::TURNL90R_MAX_DIST) < SP::ERRDist && IMin < SP::TURNL90R_MIN_DIST)
		{
			TskMotor::QMotor->Clear();
			TskTop::SetLeds(0xf);
		    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, SP::TURNL90R_PRE_ADJ, v_s);
		    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
			break;
		}
		if(IMin > IRint) IMin = IRint;
    }
    IMin = 0.f;
	WaitQEnd();
    for(i = 0; i < oslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(SP::RushSpeed,o_s[i]));
    WaitQEnd();
    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, straightPost, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    WaitQEnd();
}

void actR90r(void){
    float requ;
    int i, vslen, oslen;
    distZero = TskMotor::DistanceAcc;
    oslen = MotionCalcTurn(SP::RushSpeed, -(float)PP::PI_2, PP::Mu, o_s, &requ);

	TskTop::SetLeds(0x0);
    float straightPre = (PP::GridSize - requ);

    float straightPost = (PP::GridSize - requ +SP::TURNR90R_POST_ADJ);

    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, straightPre, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    //use the pillar info to correct
    while(TskMotor::DistanceAcc - distZero < straightPre)
    {
        if(!Semaphore_pend(SemActTick, 2))
            System_abort("pend SemActTick failed!\n");
		IRint = TskIr::IrDists.RS;
		if(IMin < 0.001f) IMin = IRint;
		if(fabs(IRint - SP::TURNR90R_MAX_DIST) < 0.005f && IMin < SP::TURNR90R_MIN_DIST)
		{
			TskMotor::QMotor->Clear();
			TskTop::SetLeds(0xf);
		    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, SP::TURNR90R_PRE_ADJ, v_s);
		    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
			break;
		}
		if(IMin > IRint) IMin = IRint;
    }
    IMin = 0.f;
    WaitQEnd();
    for(i = 0; i < oslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(SP::RushSpeed,o_s[i]));
    WaitQEnd();

    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, straightPost, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    WaitQEnd();
}

void actL180(void){
    float requ;
    int i, vslen, oslen;
    distZero = TskMotor::DistanceAcc;

    oslen = MotionCalcTurn(SP::T180Speed, (float)PP::PI, SP::TL180Mu, o_s, &requ);

	TskTop::SetLeds(0x0);
    float straightPre = (PP::GridSize - requ);

    float straightPost = (PP::GridSize - requ +SP::TURNL180_POST_ADJ);

    vslen = MotionCalcFwd(SP::RushSpeed, SP::T180Speed, 0.01f, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    vslen = MotionCalcFwd(SP::T180Speed, SP::T180Speed, straightPre-0.01f, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    //use the pillar info to correct
    while(TskMotor::DistanceAcc - distZero < straightPre)
    {
        if(!Semaphore_pend(SemActTick, 2))
            System_abort("pend SemActTick failed!\n");
		IRint = TskIr::IrDists.LS;
		if(IMin < 0.001f) IMin = IRint;
		if(fabs(IRint - SP::TURNL180_MAX_DIST) < SP::ERRDist && IMin <SP::TURNL180_MIN_DIST)
		{
			TskMotor::QMotor->Clear();
			TskTop::SetLeds(0xf);
		    vslen = MotionCalcFwd(SP::T180Speed, SP::T180Speed, SP::TURNL180_PRE_ADJ, v_s);
		    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
			break;
		}
		if(IMin > IRint) IMin = IRint;
    }
	IMin = 0.f;
    WaitQEnd();
    for(i = 0; i < oslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(SP::T180Speed,o_s[i]));
    WaitQEnd();

    vslen = MotionCalcFwd(SP::T180Speed, SP::T180Speed, straightPost-0.01f, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    vslen = MotionCalcFwd(SP::T180Speed, SP::T180Speed, 0.01f, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    WaitQEnd();
}

void actR180(void){
    float requ;
    int i, vslen, oslen;
    distZero = TskMotor::DistanceAcc;

    oslen = MotionCalcTurn(SP::T180Speed, -(float)PP::PI, SP::TR180Mu, o_s, &requ);

    float straightPre = (PP::GridSize - requ);

    float straightPost = (PP::GridSize - requ +SP::TURNR180_POST_ADJ);

    vslen = MotionCalcFwd(SP::RushSpeed, SP::T180Speed, 0.01f, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    vslen = MotionCalcFwd(SP::T180Speed, SP::T180Speed, straightPre-0.01f, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    //use the pillar info to correct
    while(TskMotor::DistanceAcc - distZero < straightPre)
    {
        if(!Semaphore_pend(SemActTick, 2))
            System_abort("pend SemActTick failed!\n");
		IRint = TskIr::IrDists.RS;
		if(IMin < 0.001f) IMin = IRint;
		if(fabs(IRint - SP::TURNR180_MAX_DIST) < SP::ERRDist && IMin < SP::TURNR180_MIN_DIST)
		{
			TskMotor::QMotor->Clear();
			TskTop::SetLeds(0xf);
		    vslen = MotionCalcFwd(SP::T180Speed, SP::T180Speed, SP::TURNR180_PRE_ADJ, v_s);
		    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
			break;
		}
		if(IMin > IRint) IMin = IRint;
    }
	IMin = 0.f;
    WaitQEnd();
    for(i = 0; i < oslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(SP::T180Speed,o_s[i]));
    WaitQEnd();

    vslen = MotionCalcFwd(SP::T180Speed, SP::T180Speed, straightPost-0.01f, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    vslen = MotionCalcFwd(SP::T180Speed, SP::T180Speed, 0.01f, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    WaitQEnd();
}

//TODO about turn out
void actL45o(void){
    float requ;
    int i, vslen, oslen;
    distZero = TskMotor::DistanceAcc;

	TskTop::SetLeds(0x0);
    oslen = MotionCalcTurn(SP::RushSpeed, (float)PP::PI_2 / 2.f, PP::Mu, o_s, &requ);

    float straightPre = (PP::GridSize - requ);
    float straightPost = (PP::GridSize - requ + SP::TURNLO45_POST_ADJ);

    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, straightPre, v_s);

    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    //use the pillar info to correct
    while(TskMotor::DistanceAcc - distZero < straightPre)
    {
        if(!Semaphore_pend(SemActTick, 2))
            System_abort("pend SemActTick failed!\n");
		IRint = TskIr::IrDists.LS;
		if(IMin < 0.001f) IMin = IRint;
		if(fabs(IRint - SP::TURNLO45_MAX_DIST) < SP::ERRDist && IMin < SP::TURNLO45_MIN_DIST)
		{
			TskMotor::QMotor->Clear();
			TskTop::SetLeds(0xf);
			vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, SP::TURNLO45_PRE_ADJ, v_s);
		    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
			break;
		}
		if(IMin > IRint) IMin = IRint;
    }
	IMin = 0.f;
    WaitQEnd();
    for(i = 0; i < oslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(SP::RushSpeed,o_s[i]));
    WaitQEnd();

    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, straightPost, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    WaitQEnd();
}

void actR45o(void){
    float requ;
    int i, vslen, oslen;
    distZero = TskMotor::DistanceAcc;
	TskTop::SetLeds(0x0);
    oslen = MotionCalcTurn(SP::RushSpeed, (float)-PP::PI_2 / 2.f, PP::Mu, o_s, &requ);

    float straightPre = (PP::GridSize - requ);
    float straightPost = (PP::GridSize - requ + SP::TURNRO45_POST_ADJ);

    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, straightPre, v_s);

    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    //use the pillar info to correct
    while(TskMotor::DistanceAcc - distZero < straightPre)
    {
        if(!Semaphore_pend(SemActTick, 2))
            System_abort("pend SemActTick failed!\n");
		IRint = TskIr::IrDists.RS;
		if(IMin < 0.001f) IMin = IRint;
		if(fabs(IRint - SP::TURNRO45_MAX_DIST) < SP::ERRDist && IMin < SP::TURNRO45_MIN_DIST)
		{
			TskMotor::QMotor->Clear();
			TskTop::SetLeds(0xf);
			vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, SP::TURNRO45_PRE_ADJ, v_s);
		    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
			break;
		}
		if(IMin > IRint) IMin = IRint;
    }
    IMin = 0.f;
    WaitQEnd();
    for(i = 0; i < oslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(SP::RushSpeed,o_s[i]));
    WaitQEnd();

    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, straightPost, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    WaitQEnd();
}

void actL135o(void){
    float requ;
    int i, vslen, oslen;
    distZero = TskMotor::DistanceAcc;
	TskTop::SetLeds(0x0);
    oslen = MotionCalcTurn(SP::RushSpeed, (float)PP::PI_2 * 3.f/ 2.f, PP::Mu, o_s, &requ);

    float straightPre = (PP::GridSize - requ);
    float straightPost = (PP::GridSize - requ + SP::TURNLO135_POST_ADJ);

    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, straightPre, v_s);

    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    //use the pillar info to correct
    while(TskMotor::DistanceAcc - distZero < straightPre)
    {
        if(!Semaphore_pend(SemActTick, 2))
            System_abort("pend SemActTick failed!\n");
		IRint = TskIr::IrDists.LS;
		if(IMin < 0.001f) IMin = IRint;
		if(fabs(IRint - SP::TURNLO135_MAX_DIST) < SP::ERRDist  && IMin < SP::TURNLO135_MIN_DIST)
		{
			TskTop::SetLeds(0xf);
			TskMotor::QMotor->Clear();
			vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, SP::TURNLO135_PRE_ADJ, v_s);
		    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
			break;
		}
		if(IMin > IRint) IMin = IRint;
    }
    IMin = 0.f;
    WaitQEnd();
    for(i = 0; i < oslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(SP::RushSpeed,o_s[i]));
    WaitQEnd();

    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, straightPost, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    WaitQEnd();
}

void actR135o(void){
    float requ;
    int i, vslen, oslen;
    distZero = TskMotor::DistanceAcc;
	TskTop::SetLeds(0x0);
    oslen = MotionCalcTurn(SP::RushSpeed, (float)-PP::PI_2 * 3.f / 2.f, PP::Mu, o_s, &requ);

    float straightPre = (PP::GridSize - requ);
    float straightPost = (PP::GridSize - requ + SP::TURNRO135_POST_ADJ);

    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, straightPre, v_s);

    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    //use the pillar info to correct
    while(TskMotor::DistanceAcc - distZero < straightPre)
    {
        if(!Semaphore_pend(SemActTick, 2))
            System_abort("pend SemActTick failed!\n");
		IRint = TskIr::IrDists.RS;
		if(IMin < 0.001f) IMin = IRint;
		if(fabs(IRint - SP::TURNRO135_MAX_DIST) < SP::ERRDist && IMin < SP::TURNRO135_MIN_DIST)
		{
			TskMotor::QMotor->Clear();
			TskTop::SetLeds(0xf);
			vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, SP::TURNRO135_PRE_ADJ, v_s);
		    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
			break;
		}
		if(IMin > IRint) IMin = IRint;
    }
    IMin = 0.f;
    WaitQEnd();
    for(i = 0; i < oslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(SP::RushSpeed,o_s[i]));
    WaitQEnd();

    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, straightPost, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    WaitQEnd();
}

void actL90t(void){
    float requ;
    int i, vslen, oslen;
    distZero = TskMotor::DistanceAcc;
    oslen = MotionCalcTurn(SP::RushSpeed, (float)PP::PI_2, PP::Mu, o_s, &requ);

	TskTop::SetLeds(0x0);
    float straightPre = (PP::GridSize - requ);

    float straightPost = (PP::GridSize - requ + SP::TURNL90T_POST_ADJ);

    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, straightPre, v_s);

    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    //use the pillar info to correct
    while(TskMotor::DistanceAcc - distZero < straightPre)
    {
        if(!Semaphore_pend(SemActTick, 2))
            System_abort("pend SemActTick failed!\n");
		IRint =TskIr::IrDists.LS;
		if(IMin < 0.001f) IMin = IRint;
		if(fabs(IRint - SP::TURNL90T_MAX_DIST) < SP::ERRDist && IMin < SP::TURNL90T_MIN_DIST)
		{
			TskMotor::QMotor->Clear();
			TskTop::SetLeds(0xf);
		    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, SP::TURNL90T_PRE_ADJ, v_s);
		    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
			break;
		}
		if(IMin > IRint) IMin = IRint;
    }
    IMin = 0.f;
	WaitQEnd();
    for(i = 0; i < oslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(SP::RushSpeed,o_s[i]));
    WaitQEnd();
    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, straightPost, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    WaitQEnd();
}

void actR90t(void){
    float requ;
    int i, vslen, oslen;
    distZero = TskMotor::DistanceAcc;
    oslen = MotionCalcTurn(SP::RushSpeed, -(float)PP::PI_2, PP::Mu, o_s, &requ);

	TskTop::SetLeds(0x0);
    float straightPre = (PP::GridSize - requ);

    float straightPost = (PP::GridSize - requ +SP::TURNR90T_POST_ADJ);

    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, straightPre, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    //use the pillar info to correct
    while(TskMotor::DistanceAcc - distZero < straightPre)
    {
        if(!Semaphore_pend(SemActTick, 2))
            System_abort("pend SemActTick failed!\n");
		IRint = TskIr::IrDists.RS;
		if(IMin < 0.001f) IMin = IRint;
		if(fabs(IRint - SP::TURNR90T_MAX_DIST) < SP::ERRDist && IMin < SP::TURNR90T_MIN_DIST)
		{
			TskMotor::QMotor->Clear();
			TskTop::SetLeds(0xf);
		    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, SP::TURNR90T_PRE_ADJ, v_s);
		    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
			break;
		}
		if(IMin > IRint) IMin = IRint;
    }
    IMin = 0.f;
    WaitQEnd();
    for(i = 0; i < oslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(SP::RushSpeed,o_s[i]));
    WaitQEnd();

    vslen = MotionCalcFwd(SP::RushSpeed, SP::RushSpeed, straightPost, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    WaitQEnd();
}

void actCorrStart()
{
    int i, len;
    TskMotor::OmgAdj = 0.0f;

    //update the grid info
    distZero = TskMotor::DistanceAcc;
    GetWallInfo(&cur_wall);

    len = MotionCalcFwd(0.0f, PP::SearchSpeed, PP::StartAcclDist, v_s);
    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    len = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed,
    		PP::StartTotalDist - PP::StartAcclDist, v_s);
    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

	//Get wall info
    while(TskMotor::DistanceAcc - distZero < PP::StartTotalDist - PP::GetWallDist)
    {
    	 if(!Semaphore_pend(SemActTick, 2))
    	            System_abort("pend SemActTick failed!\n");
    }
    GetWallInfo(&nextWall);
    Mailbox_post(solve::MbAct, &nextWall, BIOS_NO_WAIT);
    WaitQEnd();
}

unsigned int  actCorrStop()
{
    int i, len;
    float stopDist;
    TskMotor::OmgAdj = 0.0f;

    //update the grid info
    distZero = TskMotor::DistanceAcc;
    GetWallInfo(&cur_wall);

    stopDist = PP::StopTotalDist-PP::StopAccDist;
    len = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed, stopDist, v_s);
    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    if(cur_wall.fwd)
    {
#if ENABLE_CORRECTION > 0
        while(fabsf(0.5f * (TskIr::IrDists.FLns + TskIr::IrDists.FRns))
        		> PP::CenterToWall + PP::StopAccDist - CP::STOPEND_DIST_ADJ)
        {
        	 if(!Semaphore_pend(SemActTick, 2))
        	            System_abort("pend SemActTick failed!\n");

        }
        TskMotor::QMotor->Clear();
#endif
    	TskTop::SetLeds(0x0f);
        len = MotionCalcFwd(PP::SearchSpeed, 0.0f, PP::StopAccDist, v_s);
        for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

        WaitQEnd();

        while(fabsf(CP::FLRYAWERROR - TskIr::IrYaw.byFLR) > PP::PI / 180.f
        		|| fabsf(CP::FWDDISADJ - (PP::CenterToWall - 0.5f * (TskIr::IrDists.FLns + TskIr::IrDists.FRns))) > 0.0025f)
        {
        	if(!Semaphore_pend(SemActTick, 2))
        		System_abort("pend SemActTick failed!\n");
            TskMotor::OmgAdj = actHeadingDirCorrByFwdIr(&cur_wall);
			TskMotor::LvAdj = actFwdDisCorrByFwdIr(&cur_wall);
        }
        TskMotor::OmgAdj = 0.0f;
        TskMotor::LvAdj = 0.0f;
        actHDirPid->Reset();
    }
    else
    {
        len = MotionCalcFwd(PP::SearchSpeed, 0.0f, PP::StopAccDist, v_s);
        for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
    }

    WaitQEnd();

    Task_sleep(750);
    return cur_wall.msk;
}

void actCorrBack(WallStatus *wall)
{
	int i, len;
	TskMotor::OmgAdj = 0.0f;

	float tht = wall->left? PP::PI_2 : -PP::PI_2;

	TskTop::SetLeds(0x00);

	len = MotionCalcRotate(tht, 0.5f * PP::Mu, o_s);

	TskTop::SetLeds(0x0f);
	for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(0.f,o_s[i]));

	WaitQEnd();
    if(wall->left || wall->right)
    {
        while(fabsf(CP::FLRYAWERROR - TskIr::IrYaw.byFLR) > PP::PI / 180.f
        		|| fabsf(CP::FWDDISADJ - (PP::CenterToWall - 0.5f * (TskIr::IrDists.FLns + TskIr::IrDists.FRns))) > 0.0025f)
        {
        	if(!Semaphore_pend(SemActTick, 2))
        		System_abort("pend SemActTick failed!\n");
            TskMotor::OmgAdj = actHeadingDirCorrByFwdIr(wall);
			TskMotor::LvAdj = actFwdDisCorrByFwdIr(wall);
        }
        TskMotor::OmgAdj = 0.0f;
        TskMotor::LvAdj = 0.0f;
        actHDirPid->Reset();
    }

	len = MotionCalcRotate(tht + (wall->left?CP::LRBACKANGLE_ADJ:-CP::LRBACKANGLE_ADJ), 0.5f * PP::Mu, o_s);
	for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(0.f,o_s[i]));

	WaitQEnd();
}

void actCorrRestart()
{
    int i, len;
    TskMotor::OmgAdj = 0.0f; //actHDirPid->Reset();

    len = MotionCalcFwd(0.0f, PP::SearchSpeed, PP::RestartDist, v_s);
    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    len = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed, CP::RESTART_DIST_ADJ, v_s);
    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
	//Get wall info
    while(TskMotor::DistanceAcc - distZero < PP::RestartDist - PP::GetWallDist)
    {
    	 if(!Semaphore_pend(SemActTick, 2))
    	            System_abort("pend SemActTick failed!\n");
    }
    GetWallInfo(&nextWall);
    Mailbox_post(solve::MbAct, &nextWall, BIOS_NO_WAIT);

    WaitQEnd();
}

void actCorrStopBackRestart()
{
    WallStatus stopWall;
    stopWall.msk = actCorrStop();
    //actCorrBack(&stopWall);
    actCorrBack(&stopWall);
    actCorrRestart();
}

void actCorrStopBack()
{
    WallStatus stopWall;
    stopWall.msk = actCorrStop();
    actCorrBack(&stopWall);
}

void actCorrFwd()
{
    int i, len;
    TskMotor::OmgAdj = 0.0f;
    bool posted = 0;
    //update the grid info
    distZero = TskMotor::DistanceAcc;
    GetWallInfo(&cur_wall);

    len = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed, PP::GridSize, v_s);
    for(i = 0; i < len; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    if(cur_wall.left || cur_wall.right)
    {
    	while(TskMotor::DistanceAcc - distZero < CP::HEADING_BY_SIRSIDE_START_DIST)
        {
    		TskTop::SetLeds(0x8);
            if(!Semaphore_pend(SemActTick, 2))
                System_abort("pend SemActTick failed!\n");
            TskMotor::OmgAdj = actHeadingDirCorrBySideIrSide(&cur_wall);
        }
        TskMotor::OmgAdj = 0.0f;
        actHDirPid->Reset();

        while(TskMotor::QMotor->Len() > 1)
        {
            if(!Semaphore_pend(SemActTick, 2))
                System_abort("pend SemActTick failed!\n");
            if(actFwdEndCorrBySideWallDisappear(&cur_wall,TskMotor::CurrentV,PP::SearchSpeed, TskMotor::DistanceAcc - distZero))
            {
            	TskTop::SetLeds(0x01);
            	break;
            }

        }
    	//Get wall info
        while(TskMotor::DistanceAcc - distZero < PP::GridSize - PP::GetWallDist)
        {
        	 if(!Semaphore_pend(SemActTick, 2))
        	            System_abort("pend SemActTick failed!\n");
        }
        GetWallInfo(&nextWall);
        Mailbox_post(solve::MbAct, &nextWall, BIOS_NO_WAIT);
    }

    //TODO
    else    // centipede
    {
        int lfInt = TskIr::IrInts.sl, rfInt = TskIr::IrInts.sr;
        int lfIntLast = 0, rfIntLast = 0, lFallCnt = 0, rFallCnt = 0;
        float lfMaxDist = -1.0f, rfMaxDist = -1.0f;
        while(TskMotor::DistanceAcc - distZero < CP::HEADING_BY_SIRFWD_BGNSTAT_POS)
        {
        	if(!Semaphore_pend(SemActTick, 2))
        		System_abort("pend SemActTick failed!\n");
        }
        while(TskMotor::DistanceAcc - distZero < CP::HEADING_BY_SIRFWD_BEGIN_POS)
        {
        	if(!Semaphore_pend(SemActTick, 2))
        		System_abort("pend SemActTick failed!\n");
            lfInt = TskIr::IrInts.sl;
            rfInt = TskIr::IrInts.sr;
            if(lfMaxDist < 0.0f)
            {
                if(lfInt < lfIntLast - 2)
                {
                    if(++lFallCnt == 2)
                        lfMaxDist = TskMotor::DistanceAcc - distZero;
                }
                else lFallCnt = 0;
            }
            if(rfMaxDist < 0.0f)
            {
                if(rfInt < rfIntLast - 2)
                {
                    if(++rFallCnt == 2)
                        rfMaxDist = TskMotor::DistanceAcc - distZero;
                }
                else rFallCnt = 0;
            }
            lfIntLast = lfInt;
            rfIntLast = rfInt;
        }
        float angErr;
        if(lfMaxDist > 0.0f && rfMaxDist > 0.0f)
        {
            angErr = (lfMaxDist - rfMaxDist) * (PP::IrSizeAngle
                    / (CP::HEADING_BY_SIRFWD_END_POS - CP::HEADING_BY_SIRFWD_BEGIN_POS));
        }
        else
        {
            angErr = 0.0f;
        }
        float omgMax = angErr * (CENTIPEDE_CORR_GAIN * 2.0f
                / ((CP::HEADING_BY_SIRFWD_END_POS - CP::HEADING_BY_SIRFWD_BEGIN_POS) / PP::SearchSpeed));

        while(TskMotor::DistanceAcc - distZero < CP::HEADING_BY_SIRFWD_END_POS)
        {
        	if(!Semaphore_pend(SemActTick, 2))
        		System_abort("pend SemActTick failed!\n");
        	TskMotor::OmgAdj = -omgMax;
        }
        TskMotor::OmgAdj = 0.0f;
//        actHDirPid->Reset();

        while(TskMotor::DistanceAcc - distZero < PP::GridSize || TskMotor::QMotor->Len() > 0)
        {
        	if(!Semaphore_pend(SemActTick, 2))
        		System_abort("pend SemActTick failed!\n");
        	TskMotor::OmgAdj = omgMax;

        	//Get wall info
        	if(TskMotor::DistanceAcc - distZero >= PP::GridSize - PP::GetWallDist)
        	{
        		if(!posted)
        		{
					GetWallInfo(&nextWall);
					Mailbox_post(solve::MbAct, &nextWall, BIOS_NO_WAIT);
					posted = 1;
        		}
        	}
        }

        TskMotor::OmgAdj = 0.0f;
        actHDirPid->Reset();
    }

    WaitQEnd();
}

void actCorrLR90(Act::ActType act)
{
    float requ;
    int i, vslen, oslen;

    //update the grid info
    distZero = TskMotor::DistanceAcc;
    GetWallInfo(&cur_wall);
    Info_LV[0] = distZero;
    oslen = MotionCalcTurn(PP::SearchSpeed, act == Act::L90 ?
    		(float)PP::PI_2 : -(float)PP::PI_2, PP::Mu, o_s, &requ);

    float straightPre = (PP::GridSize / 2.f - requ)
            + (act == Act::L90 ? CP::TURNL90_PRE_ADJ : CP::TURNR90_PRE_ADJ);

    float straightPost = (PP::GridSize / 2.f - requ)
            + (act == Act::L90 ?  CP::TURNL90_POST_ADJ : CP::TURNR90_POST_ADJ);

	TskTop::SetLeds(0x00);
    vslen = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed, straightPre, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));

    if(cur_wall.fwd)
    {
    	// TODO:...
        while((act == Act::L90 ?TskIr::IrDists.FLns : TskIr::IrDists.FRns) >
        		requ + PP::CenterToWall - (act == Act::L90 ? CP::TURNLWAIT_DIST_ADJ : CP::TURNRWAIT_DIST_ADJ))
        {
        	 if(!Semaphore_pend(SemActTick, 2))
        		 System_abort("pend SemActTick failed!\n");

         	TskTop::SetLeds(0x0f);
        }

        TskMotor::QMotor->Clear();
    }

    for(i = 0; i < oslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(PP::SearchSpeed,o_s[i]));

    vslen = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed, straightPost, v_s);
    for(i = 0; i < vslen; i++) TskMotor::QMotor->En(TskMotor::VelOmega(v_s[i],0.f));
	//Get wall info
    while(TskMotor::DistanceAcc - distZero < PP::StartTotalDist - PP::GetWallDist)
    {
    	 if(!Semaphore_pend(SemActTick, 2))
    	            System_abort("pend SemActTick failed!\n");
    }
    GetWallInfo(&nextWall);
    Mailbox_post(solve::MbAct, &nextWall, BIOS_NO_WAIT);

    WaitQEnd();
}

Act::ActType actCurrAct;

void task(UArg arg0, UArg arg1)
{
    WallStatus stopWall;

    actCurrAct = Act::Null;
    Act::ActType act;
    bool flag;
//    actCorrsInfo.fwdEnd = 0;
//    actCorrsInfo.irSideDist = 0.0f;
//    actCorrsInfo.turnWait = 0;
//    actCurrWall.msk = 0x6;

    actHDirPid = new Pid(
            SIDEIR_CORR_PID_P, SIDEIR_CORR_PID_I, SIDEIR_CORR_PID_D,
            SIDEIR_CORR_PID_N, PP::Ts, -3142.0f, 3142.0f);

    while(true)
    {
    	Mailbox_pend(MbCmd, &act, BIOS_WAIT_FOREVER);
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
            	if(flag)
            		actCorrStart();
            	else actStart();
                break;
            case Act::Stop:
            	if(flag)
            		stopWall.msk = actCorrStop();
            	else stopWall.msk = actStop();
                break;
            case Act::Back:
            	if(flag)
            		actCorrBack(&stopWall);
            	else actBack();
                break;
            case Act::TBackR:
            	if(flag)
            		actCorrStopBackRestart();
            	else actTurnBack();
            	break;
            case Act::TBack:
            	if(flag)
            		actCorrStopBack();
            	else actTurnBack();
            	break;
            case Act::Restart:
            	if(flag)
            		actCorrRestart();
            	else actRestart();
                break;
            case Act::Fwd:
            	if(flag)
            		actCorrFwd();
            	else actFwd();
                break;
            case Act::L90:
            case Act::R90:
            	if(flag)
            		actCorrLR90(actCurrAct);
            	else actLR90(actCurrAct);
                break;
            case Act::CRush:
            	actCRush();
                break;
            case Act::TRush:
            	actTRush();
                break;
            case Act::RushIn:
            	actRushIn();
                break;
            case Act::RushOut:
            	actRushOut();
                break;
            case Act::L45i:
            	actL45i();
                break;
            case Act::L45o:
            	actL45o();
                break;
            case Act::R45i:
            	actR45i();
                break;
            case Act::R45o:
            	actR45o();
                break;
            case Act::L90r:
            	actL90r();
                break;
            case Act::R90r:
            	actR90r();
                break;
            case Act::L90t:
            	actL90t();
                break;
            case Act::R90t:
            	actR90t();
                break;
            case Act::L135i:
            	actL135i();
                break;
            case Act::L135o:
            	actL135o();
                break;
            case Act::R135i:
            	actR135i();
                break;
            case Act::R135o:
            	actR135o();
                break;
            case Act::L180:
            	actL180();
                break;
            case Act::R180:
            	actR180();
                break;
            default:
                break;
            }

        }
		end_msg = ActMsg::Action_ed;
		Mailbox_post(TskTop::MbCmd, &end_msg, BIOS_NO_WAIT);
    }
}

void Init()
{
    Task_Params tskParams;

    MbCmd = Mailbox_create(4, 4, NULL, NULL);
    if(MbCmd == NULL)
        System_abort("create TskAction::MbCmd failed.\n");

//    Error_init(&eb);
    Task_Params_init(&tskParams);
    tskParams.priority = tskPrio;
    tskParams.stackSize = tskStkSize;
    tsk = Task_create(task, &tskParams, NULL);

    if(tsk == NULL)
        System_abort("TskMotor failed.");
}
}
