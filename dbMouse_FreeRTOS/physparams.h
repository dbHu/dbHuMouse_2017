/*
 * phys_params.h
 *
 *  Created on: Aug 7, 2014
 *Author: loywong
 */

#ifndef PHYSPARAMS_H_
#define PHYSPARAMS_H_

#define VERSION  "V_1"

#define ENABLE_CORRECTION	1

#define RAD2DEG(x) 	((x) * (180.0f / 3.1415926f))
#define ABSF(x) 		((x) < 0.0f ? -(x) : (x))
#define ABSI(x) 		((x) < 0 ? -(x) : (x))
#define MAX(x, y) 	((x) > (y) ? (x) : (y))

namespace PP
{
//================  physics parameters
const float PI= 	3.1415926536f;
const float PI_2=   (PI / 2.f);

// friction coefficient
const float Mu =0.5f; // 0.72f comment @2015-09-16 17:17

// gravity constant
const float g =9.80665f;

// gravity center height
const float H =0.005f;

// half of distance between 2 tires
const float W =0.0175f;

// wheel
const float RWheel = 0.0063f;

// system sampling period, DO NOT change this independently
// if Ts changed, consider:
//  * Encoder sampling period
//  * PID controller
//  * Imu auto adj zero
//  * Motion seq calc
const float Ts =0.001f;
//================ end of physics

//================ geometry informations
// gravity center to the center of 4 wheels
const float GvCenterFwd =   0.f;

// all geometry of mouse related to GRAVITY CENTER
const float BodyLength =   0.041f;
const float BodyWidth =    0.035f;
const float HeadFwd = (0.0228f - GvCenterFwd);
// geometry center forward
const float GeoFwd =  (0.000f - GvCenterFwd);
// !!! straight segment must be no less then 2x GeoFwd !!!
// tail
const float TailBack =(0.0182f + GvCenterFwd + 0.001f);

// ir position related to gravity center
// forward ir forward dist to gravity center, approx
const float IrFFwd =  (0.0135f - GvCenterFwd);
// side ir forward dist to gravity center, approx
const float IrSFwd =  (0.0145f - GvCenterFwd);

// forward ir side dist to gravity center, approx
const float IrFSide =  0.0129f;
// side ir side dist to gravity center, approx
const float IrSSide =  -0.0058f;

const float IrFwdLRDist =  0.0258f;

const float IrSizeAngle =  (57.f / 180.0f * 3.1415926536f);//side ir angle

// grids
const float GridSize =   0.090f;
const float WallThick =  0.006f;
const float CenterToWall =((GridSize - WallThick) / 2.0f);
const float StartTotalDist =   (GridSize - WallThick / 2.0f - TailBack);
const float StartAcclDist =    0.015f;
const float StopTotalDist =    (GridSize / 2.0f);
const float StopAccDist =0.020f;
const float RestartDist =(GridSize / 2.0f);

//================ end geometry infos

//================ the dist to get wall info
const float GetWallDist =  0.01f;
//================
//================ action speeds & etc.
const int SeqArrayLen    = 512;
const float SearchSpeed    = 0.3f;	//0.36f comment @20150916 17:18
const int TicksInOneAction = 300;   //9cm @ 0.3m/s 
//================

//distance for rush action
const float RushInAcclDist = (PP::GridSize / 2.f - PP::WallThick / 2.0f - PP::TailBack - 0.001f);
const float RushStopAcclDist =  0.017f;
const float RushStopBackDist =  -0.017f;
const float RushDiagDist = PP::GridSize * 0.707f;
};


struct PidParam
{

    // when P=100,I=200 can't follow
    // when P=1000,I=1500,1800 static error
    // when I=0,P=1000,900 underdamp P=800 overdamp
    // when P=800,I=1000,2000,4000,5000 overdamp
    // when P=850,I=5000,8000,9000,overdamp I=10000 underdamp
     float lvPidP;
     float lvPidI;
     float lvPidD;
     float lvPidN;

    //when P=20,I=300 overshoot
    //I=0,P=20,100,50,15,12 overshoot  P=8,10 static error
    //P=10,I=200,300 overdamp I=340,380,400 underdamp
     float avPidP;
     float avPidI;
     float avPidD;
     float avPidN;

     float posCoff;
     float velCoff;

     PidParam()
     {
         lvPidP = 1.f;
         lvPidI = 0.f;
         lvPidD = 0.f;
         lvPidN = 0.f;

         avPidP = 10.f;
         avPidI = 340.f;
         avPidD = 0.f;
         avPidN = 0.f;

         posCoff = 120.f;
         velCoff = 6.f;
     };
};

struct RushParam
{
    float RushTurnSpeed;
    float RushDiagSpeed;
    float BackSpeed;
    float RushMu;
    float TR180Mu ;
    float TL180Mu ;

    //distance for rush action;
    float RushInAcclDist;
    float RushStopAcclDist;
    float RushBackAcclDist;

    float RushDiagDist;

    //fwd distance correction;
    float ORushEndLDist;
    float ORushEndRDist;
    float DRushEndLDist;
    float DRushEndRDist;

    // 45 degree left & right turn in straight segment;
    float TURNLI45_PRE_ADJ ;
    float TURNRI45_PRE_ADJ ;
    float TURNLI45_POST_ADJ;
    float TURNRI45_POST_ADJ;

    float TURNLO45_PRE_ADJ;
    float TURNRO45_PRE_ADJ;
    float TURNLO45_POST_ADJ;
    float TURNRO45_POST_ADJ;

    //the value about deciding the turn;
    float TURNLI45TT_ADJ;
    float TURNRI45TT_ADJ;
    float TURNLO45TT_ADJ;
    float TURNRO45TT_ADJ;

    // 90 degree left & right turn in straight segment(90r);
    float TURNLO90_PRE_ADJ;
    float TURNRO90_PRE_ADJ;
    float TURNLO90_POST_ADJ;
    float TURNRO90_POST_ADJ;

    // 90 degree left & right turn in straight segment(90t);
    float TURNLD90_PRE_ADJ;
    float TURNRD90_PRE_ADJ;
    float TURNLD90_POST_ADJ;
    float TURNRD90_POST_ADJ;

    //the value about deciding the turn;
    float TURNLO90TT_ADJ;
    float TURNRO90TT_ADJ;
    float TURNLD90TT_ADJ;
    float TURNRD90TT_ADJ;

    // 135 degree left & right turn in straight segment;
    float TURNLI135_PRE_ADJ;
    float TURNRI135_PRE_ADJ;
    float TURNLI135_POST_ADJ;
    float TURNRI135_POST_ADJ;

    //135 degree left & right turn out straight segment;
    float TURNLO135_PRE_ADJ;
    float TURNRO135_PRE_ADJ;
    float TURNLO135_POST_ADJ;
    float TURNRO135_POST_ADJ;

     //the value about deciding the turn;
    float TURNLI135TT_ADJ;
    float TURNRI135TT_ADJ;
    float TURNLO135TT_ADJ;
    float TURNRO135TT_ADJ;

   // 180 degree left & right turn in straight segment;
    float TURNL180_PRE_ADJ;
    float TURNR180_PRE_ADJ;
    float TURNL180_POST_ADJ;
    float TURNR180_POST_ADJ;

    //the value about deciding the turn;
    float TURNL180TT_ADJ;
    float TURNR180TT_ADJ;

    RushParam()
    {
        RushTurnSpeed = 0.42f;
        BackSpeed = -0.1f;
        RushDiagSpeed = 1.f;
        RushMu  = 0.6f;
        TR180Mu = 0.45f;
        TL180Mu = 0.46f;

        ORushEndLDist = 0.f;
        ORushEndRDist = 0.f;
        DRushEndLDist = 0.f;
        DRushEndRDist = 0.f;

       // 45 degree left & right turn in or out straight segment
        TURNLI45_PRE_ADJ  = 0.f;
        TURNRI45_PRE_ADJ  = 0.001f;
        TURNLI45_POST_ADJ = -0.001f;
        TURNRI45_POST_ADJ = -0.002f;
        TURNLO45_PRE_ADJ  = 0.002f;
        TURNRO45_PRE_ADJ  = 0.f;
        TURNLO45_POST_ADJ = 0.f;
        TURNRO45_POST_ADJ = 0.f;

        //the value about deciding the turn;
        TURNLI45TT_ADJ  = 0.01f;
        TURNRI45TT_ADJ  = 0.012f;
        TURNLO45TT_ADJ  = 0.035f;
        TURNRO45TT_ADJ  = 0.035f;

       // 90 degree left & right turn in straight segment(O90)
        TURNLO90_PRE_ADJ  = -0.001f;
        TURNRO90_PRE_ADJ  = 0.001f;
        TURNLO90_POST_ADJ  = 0.001f;
        TURNRO90_POST_ADJ  = 0.001f;

       // 90 degree left & right turn in straight segment(D90)
        TURNLD90_PRE_ADJ  = 0.f;
        TURNRD90_PRE_ADJ  = 0.002f;
        TURNLD90_POST_ADJ  = 0.f;
        TURNRD90_POST_ADJ  = 0.f;

        //the value about deciding the turn;
        TURNLO90TT_ADJ  = 0.024f;
        TURNRO90TT_ADJ  = 0.037f;
        TURNLD90TT_ADJ  = 0.015f;
        TURNRD90TT_ADJ  = 0.015f;

       // 135 degree left & right turn in straight segment
        TURNLI135_PRE_ADJ  = -0.001f;
        TURNRI135_PRE_ADJ  = 0.001f;
        TURNLI135_POST_ADJ = 0.002f;
        TURNRI135_POST_ADJ = 0.004f;

       //135 degree left & right turn out straight segment
        TURNLO135_PRE_ADJ  = 0.004f;
        TURNRO135_PRE_ADJ  = 0.004f;
        TURNLO135_POST_ADJ = 0.002f;
        TURNRO135_POST_ADJ = 0.002f;

        //the value about deciding the turn;
        TURNLI135TT_ADJ  = 0.092f;
        TURNRI135TT_ADJ  = 0.09f;
        TURNLO135TT_ADJ  = 0.037f;
        TURNRO135TT_ADJ  = 0.037f;

       // 180 degree left & right turn in straight segment
        TURNL180_PRE_ADJ  = -0.004f;
        TURNR180_PRE_ADJ  = -0.004f;
        TURNL180_POST_ADJ = -0.004f;
        TURNR180_POST_ADJ = -0.004f;

        TURNL180TT_ADJ  = 0.f;
        TURNR180TT_ADJ  = 0.f;
    };
};

#define CORR_BACKBYIR_EN		1

struct SeachParam
{
    //================ basic correction coefficients
     float EncoderUnitCompensation;  // reduce to run farther 1.01 -> 1.15
     float GyroUnitCompensation;  // reduce to turn/rotate more
     float AcclUnitCompensation;

    //================ end basic correction coefficients

    //================ action correction parameters

    // 3 params defines how far will the side ir corr for heading dir effects during diff act
    // after these fwd end corr starts
    float HEADING_BY_SIRSIDE_START_DIST;
    float HEADING_BY_SIRSIDE_STOP_DIST;
   // 2 params defines segs where side ir corr for heading dir effects during centipede;
    float HEADING_BY_SIRFWD_BGNSTAT_POS;
    float HEADING_BY_SIRFWD_BEGIN_POS  ;
    float HEADING_BY_SIRFWD_END_POS;   
   // params defines distances added at fwd end corr occur;
    float LFWDEND_DIST_W2NW;
    float RFWDEND_DIST_W2NW;
    float CENTIPEDE_CORR_GAIN;

   //#define HEADING_BY_SIRFWD_B2EHALF_DIST ((HEADING_BY_SIRFWD_END_POS - HEADING_BY_SIRFWD_BEGIN_POS) * 0.5f);
   // left & right turn straight segment when no fwd wall for turnwait;
   // abs of these must be less than GeoFwd;
    float TURNL90_PRE_ADJ;
    float TURNR90_PRE_ADJ;
    float TURNL90_POST_ADJ;
    float TURNR90_POST_ADJ;   
    // turn wait dist adjustment;
    float TURNLWAIT_DIST_ADJ;
    float TURNRWAIT_DIST_ADJ;


    float RESTART_DIST_ADJ;

    float STOPEND_DIST_ADJ;
   // during Act Back;
    float FWDDISADJ;
    float LRBACKANGLE_ADJ;
    float FLRYAWERROR;
    float LBACKCENTER_ADJ;
    float RBACKCENTER_ADJ;
    //================ end action correction parameters

    SeachParam()
    {
        //================ basic correction coefficients
        EncoderUnitCompensation = 1.016f;  // reduce to run farther 1.01 -> 1.15
        GyroUnitCompensation    = 0.998f;  // reduce to turn/rotate more
        AcclUnitCompensation    = 1.015f;

        //================ end basic correction coefficients

        //================ action correction parameters

        // params defines how far will the side ir corr for heading dir effects during diff act
        // after these fwd end corr starts
         HEADING_BY_SIRSIDE_START_DIST = 0.005f;
         HEADING_BY_SIRSIDE_STOP_DIST = 0.04f;

        // 2 params defines segs where side ir corr for heading dir effects during centipede
         HEADING_BY_SIRFWD_BGNSTAT_POS = 0.015f;
         HEADING_BY_SIRFWD_BEGIN_POS   = 0.060f;
         HEADING_BY_SIRFWD_END_POS = (PP::GridSize * 0.667 +  HEADING_BY_SIRFWD_BEGIN_POS * 0.333);
        // params defines distances added at fwd end corr occur
         LFWDEND_DIST_W2NW= 0.030f;
         RFWDEND_DIST_W2NW= 0.038f;
         CENTIPEDE_CORR_GAIN = 0.1f;

        // left & right turn straight segment when no fwd wall for turnwait
        // abs of these must be less than GeoFwd
         TURNL90_PRE_ADJ  = 0.003f;
         TURNR90_PRE_ADJ  = 0.007f;
         TURNL90_POST_ADJ = -0.001f;
         TURNR90_POST_ADJ = -0.004f;

        // turn wait dist adjustment
         TURNLWAIT_DIST_ADJ    = 0.04f;    // more positive to turn later
         TURNRWAIT_DIST_ADJ    = 0.043f;    // more positive to turn later

         RESTART_DIST_ADJ = 0.002f;

         STOPEND_DIST_ADJ = 0.001f;

         FWDDISADJ = 0.004f;
         LRBACKANGLE_ADJ  = -.5f * PP::PI / 180.f; //
         FLRYAWERROR = 0.02f * PP::PI / 180.f;
         LBACKCENTER_ADJ  = 10.f;   //
         RBACKCENTER_ADJ  = 10.f;   //
    };
};

#endif /* PHYS_PARAMS_H_ */
