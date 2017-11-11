/*
 * CorrYawBySide.h
 *
 *  Created on: Oct 8, 2017
 *      Author: loywong
 */

#ifndef SRC_TSKACTION_CORRECTION_H_
#define SRC_TSKACTION_CORRECTION_H_

#include "Physparams.h"
#include "action/action.h"
#include "../TskIr/TskIr.h"
#include "Queue/Queue.h"
#include "../TskMotor/TskMotor.h"
#include "solve/solve.h"

namespace TskAction
{

class Reporter
{
private:
    bool reported;
    float reportAt;
    WallStatus report();
public:
    Reporter();
    Reporter(float reportDist);
    virtual ~Reporter();
    bool Tick(float pos, WallStatus &w);
};

// correct yaw and side displace if side wall exist & fwd wall not exist
// DO NOT use when speed is zero
class CorrYawBySide
{
private:
    float p;
    float tol;   // tolerance
    float sat;  // saturate large err to

    float start;
    float rollback;
    float end;
    WallStatus wall;
    float rbRatio;

    enum {idle, correcting, rollbacking, finished} state, nxtSts;
    float angAcc;
//    float rbTime;
    float rbOmg;
//    bool getWallAtStart = false;
public:
    // use given wall status
//    CorrYawBySide(WallStatus wall, float start, float rollback, float end, float rbRatio = 0.0f);
    // wall status detected at start pos
    CorrYawBySide(float start, float rollback, float end, float rbRatio = 0.0f);
    virtual ~CorrYawBySide();
    float Tick(float pos);
    void UpdateWall(WallStatus wall);
};

// ==== Yaw by Fwd wall (when turning)====
class CorrYawByFwd
{
private:
    float ratio;
    float start;
    float end;
    float tol;
    float sat;
    WallStatus wall;
    enum {idle, correcting, finished} state, nxtSts;
    float omg;
public:
    CorrYawByFwd(float start, float end);
    virtual ~CorrYawByFwd();
    void Tick(float pos);
};

// correct distance by side wall disappeared,
// if no side wall exists it will do noting
class CorrEndBySide
{
private:
    float start, end;
    float lApp, rApp;
    float velEnd;
    WallStatus wall;

    enum {idle, waiting, finished} state;
    float *vs;
    float pstamp, perr;
//    bool getWallAtStart = false;
//    int vsLen;
public:
    // lDist, when left wall disappeared, flush motion seq and append lApp
    // rDist, ...  right ...
//    CorrEndBySide(WallStatus wall, float start, float lApp, float rApp, float velEnd);
    // wall status detected at start pos
    CorrEndBySide(float start, float end, float lApp, float rApp, float velEnd);
    virtual ~CorrEndBySide();
    bool TickORush(float pos);
    bool Tick(float pos);
};

// correct distance by fwd wall in a stop action
// if no fwd wall, it will do nothing
class CorrEndByFwd
{
private:
    float start;
    float dist;
    float app;
    WallStatus wall;

    enum {idle, waiting, finished} state;
    float *vs;
//    bool getWallAtStart = false;
public:
    // dist, when distance between gravity center and fwd wall is dist
    // flush motion seq and append app
//    CorrEndByFwd(WallStatus wall, float start, float dist, float app);
    // wall status detected at start pos
    CorrEndByFwd(float start, float dist, float app);
    virtual ~CorrEndByFwd();
    bool Tick(float pos);
};

// corr turn time by fwd wall if exist
class CorrTurnTime
{
private:
    WallStatus wall;
//    Action act;
    unsigned char irCh;
    float dist;
    float turnSpeed;
    float *osTurn;
    int osLen;
    float *vsPost;
    int vsLen;
    float pstamp;
    float perr;
//    Queue<TskMotor::VelOmega> *qvo;

    enum {waiting, finished} state;
//    float adj;
public:
//    CorrTurnTime(WallStatus wall, Action act, float requ, float *osTurn, int osLen, float *vsPost, int vsLen);
    // wall status will be detected in construction func
    CorrTurnTime(unsigned char irCh, float dist, float turnSpd, float *osTurn, int osLen, float *vsPost, int vsLen);
//    CorrTurnTime(Action act, TskIr::IrDistCh irCh, float dist, Queue<TskMotor::VelOmega> *qvo);
    virtual ~CorrTurnTime();
    bool WallDis(float pos, float adj);
    bool Tick(float adj);
};

class CorrCentipede
{
private:
    float corrRatio; // correct angle / angle error detected
    float rbRatio;   // rollback angle / correct angle
    float tol;
    float sat;

    float detect;       // pos to start detect
    float start;        // pos to stop detect and start correct
    float rollback;     // pos to stop correct and start rollback
    float end;
    float accl;

    enum {idle, detecting, correcting, rollbacking, finished}state, nxtSts;
//    float crTime;
//    float rbTime;

    int lCnt, rCnt;
    float lPos, rPos;
    float crOmg, rbOmg;
//    float lDisLast = 1.f, rDisLast = 1.f;
    float lpstamp, rpstamp;
    bool wallExist;
public:
    CorrCentipede(float detect, float start, float rollback, float end, float accl);
    virtual ~CorrCentipede();
    float Tick(float pos);
};

class CorrRushYaw
{
private:
//    const float irDistMin = 0.030f;
    float pYaw; // @ rush turn speed
    float distErrSat;
    float start;
    float rollback;
    float end;
    float irDistMin;
    float rbRatio;

    enum {idle, correcting, rollbacking, finished} state, nxtSts;
    float angAcc;
    float omg;
public:
    CorrRushYaw(float start, float rback, float end, float irDistMin, float rbRatio = 0.5f);
    virtual ~CorrRushYaw();
    float Tick(float pos);
};

class CorrDiagRushEnd
{
private:
    float start;
    float lapp, rapp;

    enum {idle, waiting, finished} state;
    float dlMin;
    float drMin;
    float *vs;
public:
    // when side wall dist increse rapidly
    // flush motion seq and append app
    CorrDiagRushEnd(float start, float lApp, float rApp);
    virtual ~CorrDiagRushEnd();
    bool Tick(float pos);
};

} /* namespace TskAct */

#endif /* SRC_TSKACTION_CORRECTION_H_ */
