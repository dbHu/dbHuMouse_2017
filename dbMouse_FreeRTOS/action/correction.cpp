/*
 * CorrYawBySide.cc
 *
 *  Created on: Oct 8, 2017
 *      Author: loywong
 */
#include <stdio.h>
#include <math.h>

#include "Correction.h"
#include "../TskIr/TskIr.h"
#include "../TskMotor/TskMotor.h"
#include "action/action.h"
#include "action/motion.h"
#include "Physparams.h"
#include "TskTop/DbgUart.h"
#include "TskTop/TskTop.h"
#include "PinConfig/pinout.h"

#define Sat(a,b)        ((a) > (b))?(b) : ((a) < -(b))?-(b) : (a)
#define Cor(a,b)        ((a) > (b))?((a) - (b)) : ((a) < -(b))?((a) + (b)) : 0
#define InRng(x,min,max)    ((x) >= (min) && (x) <= (max))

namespace TskAction
{

//extern char *dbgStr;

#define PRINT_CORRINFO true

#if(PRINT_CORRINFO)
bool rtn;
char dbgStr[128];
#endif

// ==== report ====
WallStatus Reporter::report()
{
    bool rtn;
    WallStatus w;
    w.fwd = TskIr::IrBins.Fwd;
    w.left = TskIr::IrBins.LS;
    w.right = TskIr::IrBins.RS;
    unsigned int msg = w.msk & 0x7;

    rtn = xQueuePost(solve::MbAct, &msg, (TickType_t)0);
    configASSERT(rtn == pdPASS || rtn == errQUEUE_FULL);
//    DbgPutWall(w);
    return w;
}
Reporter::Reporter()
{
    reportAt = 1.0e38f;
    reported = false;
}
Reporter::Reporter(float reportDist)
{
    reportAt = reportDist;
    reported = false;
}
Reporter::~Reporter()
{
    if(!reported)
        report();
}
bool Reporter::Tick(float pos, WallStatus &w)
{
    if(!reported && pos >= reportAt)
    {
        w = report();
        reported = true;
        return true;
    }
    return false;
}

// ==== Yaw by Side ====
//CorrYawBySide::CorrYawBySide(WallStatus wall, float start, float rollback, float end, float rbRatio)
//    : start(start), rollback(rollback), end(end), rbRatio(rbRatio)
//{
//    this->wall = wall;
//    this->getWallAtStart = false;
//}
CorrYawBySide::CorrYawBySide(float start, float rollback, float end, float rbRatio)
    : start(start), rollback(rollback), end(end), rbRatio(rbRatio)
{
    this->wall.msk = 0;
    this->state = idle;
    this->nxtSts = idle;
    this->angAcc = 0.0f;
    this->rbOmg = 0.0f;
    this->p = 12.0f;
    this->tol = 0.0f * 0.01745f;
    this->sat = 15.0f * 0.01745f;
//    this->getWallAtStart = true;
}
CorrYawBySide::~CorrYawBySide()
{
    TskMotor::OmgAdj = 0.0f;
}
void CorrYawBySide::UpdateWall(WallStatus wall)
{
    this->wall = wall;
}
float CorrYawBySide::Tick(float pos)
{
    float err = 0.f, omg;

    switch(state)
    {
    case idle:
        if(pos >= start)
        {
//            if(getWallAtStart)
                getWall(this->wall);
            if(!wall.fwd && (wall.left || wall.right))
                nxtSts = correcting;
            else
                nxtSts = finished;
        }
        break;
    case correcting:
        if(pos >= rollback)
            nxtSts = rollbacking;
        break;
    case rollbacking:
        if(pos >= end)
            nxtSts = finished;
        break;
    case finished:
    default:
        nxtSts = finished;
        break;
    }

    switch(state)
    {
    case idle:
        break;
    case correcting:
        err = 0.f;
        err -= wall.left ? TskIr::IrYaw.byLS : 0.f;
        err -= wall.right ? TskIr::IrYaw.byRS : 0.f;
        err *= (wall.left && wall.right) ? 0.5f : 1.0f;
//        omg = p * Sat(Cor(err, tol), sat);
        omg = p * PP::SearchSpeed / TskMotor::CurrentV * Sat(Cor(err, tol), sat);
        TskMotor::OmgAdj = omg;
        angAcc += omg * PP::Ts;
        if(nxtSts == rollbacking)
        {
            float rbTime = (end - rollback) / TskMotor::desire.Velocity;
            rbOmg = (rbTime >= 0.001f) ? (-angAcc * rbRatio / rbTime) : 0.0f;
#if(PRINT_CORRINFO)
            sprintf(dbgStr, "Y:%6.3f", angAcc);
            rtn = xQueuePost(TskPrint::MbCmd, dbgStr, (TickType_t)0);
            configASSERT(rtn == pdPASS);
#endif
        }
        break;
    case rollbacking:
        TskMotor::OmgAdj = rbOmg;
        if(nxtSts == finished)
            TskMotor::OmgAdj = 0.0f;
        break;
    case finished:
    default:
        break;
    }

    state = nxtSts;

    return TskMotor::OmgAdj;
}

// ==== Yaw by Fwd wall (when turning)====
CorrYawByFwd::CorrYawByFwd(float start, float end)
    : start(start), end(end) {
    this->state = idle;
    this->nxtSts = idle;
    this->omg = 0.0f;
    this->ratio = 0.3f;
    tol = 15.f * 0.01745f;
    sat = 40.0f * 0.01745f;

}
CorrYawByFwd::~CorrYawByFwd()
{
    TskMotor::OmgAdj = 0.f;
}
void CorrYawByFwd::Tick(float pos)
{
    switch(state)
    {
    case idle:
        if(pos >= start)
        {
            getWall(this->wall);
            if(wall.fwd && !(wall.left || wall.right))
                nxtSts = correcting;
            else
                nxtSts = finished;
        }
        break;
    case correcting:
        if(pos >= end)
            nxtSts = finished;
        break;
    case finished:
    default:
        break;
    }

    switch(state)
    {
    case idle:
        if(nxtSts == correcting)
        {
            float yawErr = Sat(Cor(-TskIr::IrYaw.byFLR, tol), sat);
//            float yawErr = -TskIr::IrYaw.byFLR;
            float corTime = (end - start) / TskMotor::desire.Velocity;
            omg = (corTime >= 0.001f) ? (yawErr * ratio / corTime) : 0.0f;
#if(PRINT_CORRINFO)
            sprintf(dbgStr, "YBF:%6.3f %6.3f\r\n", omg, yawErr);
            rtn = xQueuePost(TskPrint::MbCmd, dbgStr, (TickType_t)0);
            configASSERT(rtn == pdPASS);
#endif
//            TskHif::Puts(dbgStr);
            TskTop::SetLeds(0x2);
        }
        else if(nxtSts == finished)
            omg = 0.f;
        break;
    case correcting:
        if(nxtSts == finished)
            TskMotor::OmgAdj = 0.f;
        else
            TskMotor::OmgAdj = omg;
        break;
    case finished:
    default:
        break;
    }

    state = nxtSts;
}

// ==== End by Side ====
//CorrEndBySide::CorrEndBySide(WallStatus wall, float start, float lApp, float rApp, float velEnd)
//    : start(start), lApp(lApp), rApp(rApp), velEnd(velEnd)
//{
//    this->wall = wall;
//    vs = new float[PP::TicksInOneAction];
//    this->getWallAtStart = false;
//}
CorrEndBySide::CorrEndBySide(float start, float end, float lApp, float rApp, float velEnd)
    : start(start), end(end), lApp(lApp), rApp(rApp), velEnd(velEnd)
{
    this->wall.msk = 0;
    vs = new float[PP::TicksInOneAction];
    state = idle;

    pstamp = 0.f;
    perr =0.f;
}
CorrEndBySide::~CorrEndBySide()
{
    delete vs;
}

bool CorrEndBySide::TickORush(float pos)
{
    bool distIncreasing = false;
    int len;
    float accl;
    float dist;

    if(state == waiting)
    {
        if(TskIr::SideLWallDisPos > 0.001f || TskIr::SideRWallDisPos > 0.001f)
        pstamp = TskMotor::DistanceAcc - ((TskIr::SideLWallDisPos > TskIr::SideRWallDisPos)?
             TskIr::SideLWallDisPos : TskIr::SideRWallDisPos);
        perr = (pstamp > 1e-6f && pstamp < 0.03f)? pstamp : 0.f;

        if(perr > 1e-6f)
        {
            distIncreasing = true;
            dist = (TskIr::SideLWallDisPos > TskIr::SideRWallDisPos)?lApp - perr : rApp - perr;
            if(dist < 1e-6f) dist = 0.f;
            TskTop::SetLeds(0x2);
        }
    }
    switch(state)
    {
    case idle:
        state = waiting;
        break;
    case waiting:
        if(distIncreasing || pos >= end)
        {
            len = MotionCalcFwd(TskMotor::desire.Velocity, velEnd, dist, vs, &accl);
            MotionSeqFlush();
            for(int i = 0; i < len; i++) MotionSeqWrite(vs[i], 0.0f);
        }
        state = finished;
        break;
    case finished:
    default:
        state = finished;
        break;
    }
    return distIncreasing;

}
bool CorrEndBySide::Tick(float pos)
{
    bool wallDisappearing = false;
    int len;
    float accl;
    if(state == waiting)
    {
        if(wall.left && TskIr::IrBins.LS == 0)
        {
            len = MotionCalcFwd(TskMotor::desire.Velocity, velEnd, lApp, vs, &accl);
            MotionSeqFlush();
            for(int i = 0; i < len; i++) MotionSeqWrite(vs[i], 0.0f);
            TskTop::SetLeds(0x2);
            wallDisappearing = true;
        }
        else if(wall.right && TskIr::IrBins.RS == 0)
        {
            len = MotionCalcFwd(TskMotor::desire.Velocity, velEnd, rApp, vs, &accl);
            MotionSeqFlush();
            for(int i = 0; i < len; i++) MotionSeqWrite(vs[i], 0.0f);
            TskTop::SetLeds(0x1);
            wallDisappearing = true;
        }
    }

    switch(state)
    {
    case idle:
        if(pos >= start)
        {
//            if(getWallAtStart)
            getWall(this->wall);
            state = waiting;
        }
        break;
    case waiting:
        if(wallDisappearing || pos >= end)
            state = finished;
        break;
    case finished:
    default:
        state = finished;
        break;
    }
    return wallDisappearing;
}

// ==== End by fwd wall ====
//CorrEndByFwd::CorrEndByFwd(WallStatus wall, float start, float dist, float app)
//    :start(start), dist(dist), app(app)
//{
//    this->wall = wall;
//    vs = new float[PP::TicksInOneAction];
//    this->getWallAtStart = false;
//}
CorrEndByFwd::CorrEndByFwd(float start, float dist, float app)
    :start(start), dist(dist), app(app)
{
    this->wall.msk = 0;
    vs = new float[PP::TicksInOneAction];
    state = idle;
//    this->getWallAtStart = true;
}
CorrEndByFwd::~CorrEndByFwd()
{
    delete vs;
}
bool CorrEndByFwd::Tick(float pos)
{
    bool wallComming = false;
    int len;
    float accl;
    if(state == waiting)
    {
        if(wall.fwd &&
                (TskIr::irDistFwd() < dist))//PP::CenterToWall + PP::StopAcclDist - scp.ActEnd.Stop))
        {
            len = MotionCalcFwd(TskMotor::desire.Velocity, 0.0f, app, vs, &accl);
            MotionSeqFlush();
            for(int i = 0; i < len; i++) MotionSeqWrite(vs[i], 0.0f);
            TskTop::SetLeds(0x2);
            wallComming = true;
        }
    }

    switch(state)
    {
    case idle:
        if(pos >= start)
        {
//            if(this->getWallAtStart)
                getWall(this->wall);
            state = waiting;
        }
        break;
    case waiting:
        if(wallComming)
            state = finished;
        break;
    case finished:
    default:
        state = finished;
        break;
    }
    return wallComming;
}

// ==== Turn time ====
//CorrTurnTime::CorrTurnTime(WallStatus wall, Action act, float requ, float *osTurn, int osLen, float *vsPost, int vsLen)
//    : requ(requ), osTurn(osTurn), osLen(osLen), vsPost(vsPost), vsLen(vsLen)
//{
//    this->wall = wall;
//    this->act = act;
//    this->adj = (act == Action::L90 ? scp.TurnTimeAdj.L : scp.TurnTimeAdj.R);
//}

CorrTurnTime::CorrTurnTime(unsigned char irCh, float dist, float turnSpd, float *osTurn, int osLen, float *vsPost, int vsLen)
    : dist(dist), turnSpeed(turnSpd), osTurn(osTurn), osLen(osLen), vsPost(vsPost), vsLen(vsLen)/*, qvo(NULL)*/
{
    this->irCh = irCh;
    getWall(this->wall);
    this->state = waiting;
    pstamp = 0.f;
    perr = 0.f;
}

CorrTurnTime::~CorrTurnTime() {}

bool CorrTurnTime::WallDis(float pos, float adj)
{
    using namespace TskIr;
    bool isTime = false;
    float len, v_s[150];
    float accl;
    float corrdist = 0.f;

    if(state == waiting)
    {
        if(pos < dist){

            if(TskIr::SideLWallDisPos > 0.001f || TskIr::SideRWallDisPos > 0.001f)
            pstamp = TskMotor::DistanceAcc - ((TskIr::SideLWallDisPos > TskIr::SideRWallDisPos)?
                 TskIr::SideLWallDisPos : TskIr::SideRWallDisPos);
            perr = (pstamp > 1e-6f && pstamp < 0.05f)? pstamp : 0.f;

            //from wall to no wall
            if(perr > 1e-6f)
            {
                corrdist = (adj > perr)?adj - perr : 0.f;
                TskTop::SetLeds(0x2);
                isTime = true;
            }
        }
        else {
            isTime = true;
        }
        MotionSeqWrite(SP.RushTurnSpeed, 0);
    }

    switch(state)
    {
    case waiting:
        if(isTime)
        {
#if(PRINT_CORRINFO)
            sprintf(dbgStr, "pstamp:%6.3f,perr:%6.3f,corr:%6.3f\r\n", pstamp, perr,corrdist);
            rtn = xQueuePost(TskPrint::MbCmd, dbgStr, (TickType_t)0);
            configASSERT(rtn == pdPASS);
#endif
            MotionSeqFlush();
            len = MotionCalcFwd(SP.RushTurnSpeed, SP.RushTurnSpeed, corrdist, v_s, &accl);
            for(int i = 0; i < len; i++) MotionSeqWrite(v_s[i], 0.f);
            for(int i = 0; i < osLen; i++) MotionSeqWrite(turnSpeed, osTurn[i]);
            for(int i = 0; i < vsLen; i++) MotionSeqWrite(vsPost[i], 0.0f);
            state = finished;
        }
        break;
    case finished:
        break;
    default:
        state = finished;
        break;
    }

    return isTime;
}

bool CorrTurnTime::Tick(float adj)
{
    using namespace TskIr;
    bool isTime = false;
    if(     (((1<<irCh) & IrCh::FL || (1<<irCh) & IrCh::FR) && wall.fwd) ||
            ((1<<irCh) & IrCh::SL && wall.left) ||
            ((1<<irCh) & IrCh::SR && wall.right))
    {
        if(state == waiting)
        {
            float irDist = TskIr::IrDists.ch[(int)irCh];
            if(irDist <= adj)
            {
//                if(qvo == NULL)
//                {
                    MotionSeqFlush();
                    for(int i = 0; i < osLen; i++) MotionSeqWrite(turnSpeed, osTurn[i]);
                    for(int i = 0; i < vsLen; i++) MotionSeqWrite(vsPost[i], 0.0f);
//                }
//                else
//                {
//                    TskMotor::VelOmega vo;
//                    while(qvo->De(vo))
//                    {
//                        MotionSeqWrite(vo);
//                    }
//                }
                    TskTop::SetLeds(0x2);
                isTime = true;
            }

            MotionSeqWrite(SP.RushTurnSpeed, 0);
        }

        switch(state)
        {
        case waiting:
            if(isTime)
                state = finished;
            break;
        case finished:
        default:
            state = finished;
            break;
        }
    }
    return isTime;
}

// ==== Centipede ====
CorrCentipede::CorrCentipede(float detect, float start, float rollback, float end, float accl)
    : detect(detect), start(start), rollback(rollback), end(end), accl(accl)
{
    corrRatio = 0.6f; // correct angle / angle error detected
    rbRatio = CP.CENTIPEDE_CORR_GAIN;   // rollback angle / correct angle
    tol = 0.0f * 0.01745f;
    sat = 10.0f * 0.01745f;
    state = idle, nxtSts = idle;
    lCnt = 0, rCnt = 0;
    lPos = -1.f, rPos = -1.f;
    crOmg = 0.0f, rbOmg = 0.0f;
    wallExist = false;
}

CorrCentipede::~CorrCentipede()
{
    TskMotor::OmgAdj = 0.0f;
}
float CorrCentipede::Tick(float pos)
{

    float lDis = TskIr::IrDists.LS;
    float rDis = TskIr::IrDists.RS;

    switch(state)
    {
    case idle:
        if(pos >= detect)
        {
            WallStatus w;
            getWall(w);
            if(w.left || w.right)
                nxtSts = finished;
            else
                nxtSts = detecting;
        }
        break;
    case detecting:
        if(pos >= start)
            nxtSts = correcting;
        break;
    case correcting:
        if(pos >= rollback)
            nxtSts = rollbacking;
        break;
    case rollbacking:
        if(pos >= end)
            nxtSts = finished;
        break;
    case finished:
    default:
        nxtSts = finished;
        break;
    }

    switch(state)
    {
    case detecting:
        if(lPos < 0.f)  // lpos not updated
        {
            //TODO
            if(TskMotor::DistanceAcc - TskIr::SideLWallDisPos > 0.f
               && TskMotor::DistanceAcc - TskIr::SideLWallDisPos < 0.03f
               && TskIr::SideLWallDisPos > 1e-6f)
            {
                lPos = TskIr::SideLWallDisPos;
            }
        }
        if(rPos < 0.f)  // rpos not updated
        {
            if(TskMotor::DistanceAcc - TskIr::SideRWallDisPos > 0.f
               && TskMotor::DistanceAcc - TskIr::SideRWallDisPos < 0.03f
               && TskIr::SideRWallDisPos > 1e-6f)
            {
                rPos = TskIr::SideRWallDisPos;
            }
        }
        if(nxtSts == correcting)
        {
            float ang;
            float crTime, rbTime;
            if(lPos > 0.f && rPos > 0.f)
            {
                ang = (lPos - rPos) / PP::GridSize;
            }
            else
                ang = 0.f;
            ang = Sat(Cor(ang, tol), sat);
            if(accl < 1e-6f)
            {
                crTime = (rollback - start) / TskMotor::desire.Velocity;
                rbTime = (end - rollback) / TskMotor::desire.Velocity;
            }
            else
            {
                float vsq1 = TskMotor::desire.Velocity * TskMotor::desire.Velocity + 2.f * accl * (rollback - start);
                configASSERT(vsq1);
                float vsq2 = TskMotor::desire.Velocity * TskMotor::desire.Velocity + 2.f * accl * (end - rollback);
                configASSERT(vsq2);
                crTime = (sqrt(vsq1) - TskMotor::desire.Velocity) / accl;
                rbTime = (sqrt(vsq2) - TskMotor::desire.Velocity) / accl;
            }
            crOmg = crTime >= 0.001f ? (-corrRatio * ang / crTime) : 0.0f;
            rbOmg = rbTime >= 0.001f ? (+rbRatio * corrRatio * ang / rbTime) : 0.0f;
            TskTop::SetLeds(0x2);
#if(PRINT_CORRINFO)
            sprintf(dbgStr, "CP%6.3f,%6.3f", lPos, rPos);
            rtn = xQueuePost(TskPrint::MbCmd, dbgStr, (TickType_t)0);
            configASSERT(rtn == pdPASS);
#endif
        }
        break;
    case correcting:
        TskMotor::OmgAdj = crOmg;
        break;
    case rollbacking:
        TskMotor::OmgAdj = rbOmg;
        if(nxtSts == finished)
            TskMotor::OmgAdj = 0.0f;
        break;
    case idle:
    case finished:
    default:
        break;
    }

    state = nxtSts;

    return TskMotor::OmgAdj;
}

// ========
CorrRushYaw::CorrRushYaw(float start, float rback, float end, float irDistMin, float rbRatio)
    : start(start), rollback(rback), end(end), irDistMin(irDistMin), rbRatio(rbRatio) {
    pYaw = 314.f; // @ rush turn speed
    distErrSat = 0.005f;
    state = idle, nxtSts = idle;
    angAcc = 0.f;
    omg = 0.f;

}
CorrRushYaw::~CorrRushYaw()
{
    TskMotor::OmgAdj = 0.f;
}
float CorrRushYaw::Tick(float pos)
{
    float dl, dr, p;

    switch(state)
    {
    case idle:
        if(pos >= start)
            nxtSts = correcting;
        break;
    case correcting:
        if(pos > rollback)
            nxtSts = rollbacking;
        break;
    case rollbacking:
        if(pos > end)
            nxtSts = finished;
        break;
    default:
        break;
    }

    switch(state)
    {
    case idle:
        break;
    case correcting:
        dl = TskIr::IrDists.LS;
        dr = TskIr::IrDists.RS;
        p = pYaw / SP.RushTurnSpeed * TskMotor::desire.Velocity;
        if(dl < irDistMin)
        {
            omg = -p * Sat(irDistMin - dl, distErrSat);
            angAcc += omg * PP::Ts;
            TskMotor::OmgAdj = omg;
        }
        else if(dr < irDistMin)
        {
            omg = p * Sat(irDistMin - dr, distErrSat);
            angAcc += omg * PP::Ts;
            TskMotor::OmgAdj = omg;
        }
        else
        {
            TskMotor::OmgAdj = 0.f;
        }
        if(nxtSts == rollbacking)
        {
            float rbTime = (end - rollback) / TskMotor::desire.Velocity;
            omg = (rbTime >= 0.001f) ? (-rbRatio * angAcc / rbTime) : 0.0f;
        }
        break;
    case rollbacking:
        if(nxtSts == finished)
        {
            omg = 0.f;
            TskMotor::OmgAdj = omg;
        }
        else
        {
            TskMotor::OmgAdj = omg;
        }
        break;
    case finished:
        break;
    default:
        break;
    }

    state = nxtSts;
    return omg;
}

// ========
CorrDiagRushEnd::CorrDiagRushEnd(float start, float lApp, float rApp):
        start(start), lapp(lApp), rapp(rApp)
{
    vs = new float[PP::TicksInOneAction];
    dlMin = 1.0e38f;
    drMin = 1.0e38f;
    state = idle;
}
CorrDiagRushEnd::~CorrDiagRushEnd()
{
    delete vs;
}
bool CorrDiagRushEnd::Tick(float pos)
{
    const float distMinLim = 0.036f;
    float dl = TskIr::IrDists.LS;
    float dr = TskIr::IrDists.RS;
    float accl;
    bool distIncreasing = false;
    if(state == waiting)
    {
        if(dl < dlMin) dlMin = dl;
        if(dr < drMin) drMin = dr;
        if(dlMin < distMinLim && dl > dlMin + 0.000f)
        {
            float v = TskMotor::desire.Velocity;
            int len = MotionCalcFwd(v, v, lapp, vs, &accl);
            MotionSeqFlush();
            for(int i = 0; i < len; i++) MotionSeqWrite(vs[i], 0.0f);
            distIncreasing = true;
            TskTop::SetLeds(0x2);
        }
        else if(drMin < distMinLim && dr > drMin + 0.000f)
        {
            float v = TskMotor::desire.Velocity;
            int len = MotionCalcFwd(v, v, rapp, vs, &accl);
            MotionSeqFlush();
            for(int i = 0; i < len; i++) MotionSeqWrite(vs[i], 0.0f);
            distIncreasing = true;
            TskTop::SetLeds(0x1);
        }
    }

    switch(state)
    {
    case idle:
        if(pos >= start)
            state = waiting;
        break;
    case waiting:
        if(distIncreasing)
            state = finished;
        break;
    case finished:
    default:
        state = finished;
        break;
    }
    return distIncreasing;
}

// ====
//CorrRushTurnTime::CorrRushTurnTime(Action act, float requ, float *osTurn, int osLen, float *vsPost, int vsLen)
//    : requ(requ), osTurn(osTurn), osLen(osLen), vsPost(vsPost), vsLen(vsLen)
//{
//    getWall(this->wall);
//    this->act = act;
////    this->adj = (act == Action::L90 ? scp.TurnTimeAdj.L : scp.TurnTimeAdj.R);
//}
//CorrRushTurnTime::~CorrRushTurnTime() {}
//bool CorrRushTurnTime::Tick(float pos)
//{
//    bool isTime = false;
//    if(wall.fwd)
//    {
//        if(state == waiting)
//        {
//            float irDist = (act == Action::L45i ? TskIr::IrDists.FL : TskIr::IrDists.FR);
//            if(irDist <= PP::GridSize + PP::CenterToWall - )
//            {
//                MotionSeqFlush();
//                for(int i = 0; i < osLen; i++) MotionSeqWrite(PP::SearchSpeed, osTurn[i]);
//                for(int i = 0; i < vsLen; i++) MotionSeqWrite(vsPost[i], 0.0f);
//                TskHif::Led(0x080, 0x080, 1);
//                isTime = true;
//            }
//        }
//
//        switch(state)
//        {
//        case waiting:
//            if(isTime)
//                state = finished;
//            break;
//        case finished:
//        default:
//            state = finished;
//            break;
//        }
//    }
//    return isTime;
//}



} /* namespace TskAct */
