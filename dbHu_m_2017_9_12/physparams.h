/*
 * phys_params.h
 *
 *  Created on: Aug 7, 2014
 *Author: loywong
 */

#ifndef PHYSPARAMS_H_
#define PHYSPARAMS_H_

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
const float W =0.0185f;

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
const float GvCenterFwd =   -0.002f;

// all geometry of mouse related to GRAVITY CENTER
const float BodyLength =   0.041f;
const float BodyWidth =    0.037f;
const float HeadFwd = (0.0228f - GvCenterFwd);
// geometry center forward
const float GeoFwd =  (0.000f - GvCenterFwd);
// !!! straight segment must be no less then 2x GeoFwd !!!
// tail
const float TailBack =(0.0182f + GvCenterFwd + 0.001f);

// ir position related to gravity center
// forward ir forward dist to gravity center, approx
const float IrFFwd =  (0.0157f - GvCenterFwd);
// side ir forward dist to gravity center, approx
const float IrSFwd =  (0.0148f - GvCenterFwd);

// forward ir side dist to gravity center, approx
const float IrFSide =  0.0145f;
// side ir side dist to gravity center, approx
const float IrSSide =  0.0083f;

const float IrFwdLRDist =  0.029f;

const float IrSizeAngle =  (60.f / 180.0f * 3.1415926536f);//side ir angle

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
const float GetWallDist =  0.005f;
//================
//================ action speeds & etc.
const int SeqArrayLen    = 512;
const float SearchSpeed    = 0.3f;	//0.36f comment @20150916 17:18
//================

//================ basic correction coefficients
 const float EncoderUnitCompensation = 1.010f;  // reduce to run farther
 const float GyroUnitCompensation    = 0.996f;  // reduce to turn/rotate more
 const float AcclUnitCompensation    = 1.015f;
}


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

         posCoff = 170.f;
         velCoff = 3.7f;
     };
};

struct RushParam
{
    float RushSpeed;
    float T180Speed;
    float TR180Mu ;
    float TL180Mu ;
    //distance for rush action;
    float RushInAcclDistDbg;
    float RushInAcclDist;
    float RushOutAcclDistDbg;
    float RushOutAcclDist;
    //the wait distance limit;
    float ERRDist;
    //TRush;
    float TRushDist;
    float TRushComDist ;
    //6.2cm;
    float TRushL_MAX_DIST;
    float TRushL_MIN_DIST;
    //6cm;
    float TRushR_MAX_DIST  ;
    float TRushR_MIN_DIST  ;
    // 45 degree left & right turn in straight segment;
    float TURNLI45_PRE_ADJ ;
    float TURNRI45_PRE_ADJ ;
    float TURNLI45_POST_ADJ;
    float TURNRI45_POST_ADJ;
    //the value about deciding the turn;
    //leave the grid top 2cm;
    float TURNLI45_MAX_DIST  ;
    float TURNLI45_MIN_DIST  ;
    //leave the grid top 2cm;
    float TURNRI45_MAX_DIST  ;
    float TURNRI45_MIN_DIST  ;

    //45 degree left & right turn out straight segment
     float TURNLO45_PRE_ADJ;
     float TURNRO45_PRE_ADJ;
     float TURNLO45_POST_ADJ;
     float TURNRO45_POST_ADJ;
    //6.2cm
     float TURNLO45_MAX_DIST;
     float TURNLO45_MIN_DIST;
    //6cm
     float TURNRO45_MAX_DIST;
     float TURNRO45_MIN_DIST;

    // 90 degree left & right turn in straight segment(90r);
    float TURNL90R_PRE_ADJ;
    float TURNR90R_PRE_ADJ;
    float TURNL90R_POST_ADJ;
    float TURNR90R_POST_ADJ;

    //the value about deciding the turn;
    //leave the grid top 2cm;
    float TURNL90R_MAX_DIST;
    float TURNL90R_MIN_DIST;
   //leave the grid top 2cm;
    float TURNR90R_MAX_DIST;
    float TURNR90R_MIN_DIST;

   // 90 degree left & right turn in straight segment(90t);
    float TURNL90T_PRE_ADJ;
    float TURNR90T_PRE_ADJ;
    float TURNL90T_POST_ADJ;
    float TURNR90T_POST_ADJ;

   //TODO;
   //the value about deciding the turn;
   //leave the grid top 2cm;
    float TURNL90T_MAX_DIST;
    float TURNL90T_MIN_DIST;
   //leave the grid top 2cm;
    float TURNR90T_MAX_DIST;
    float TURNR90T_MIN_DIST;

   // 135 degree left & right turn in straight segment;
    float TURNLI135_PRE_ADJ;
    float TURNRI135_PRE_ADJ;
    float TURNLI135_POST_ADJ;
    float TURNRI135_POST_ADJ;
   //the value about deciding the turn;
   //leave the grid top 2cm;
    float TURNLI135_MAX_DIST;
    float TURNLI135_MIN_DIST;
   //leave the grid top 2cm;
    float TURNRI135_MAX_DIST;
    float TURNRI135_MIN_DIST;

   //135 degree left & right turn out straight segment;
    float TURNLO135_PRE_ADJ;
    float TURNRO135_PRE_ADJ;
    float TURNLO135_POST_ADJ;
    float TURNRO135_POST_ADJ;

   //the value about deciding the turn;
   //6.2cm;
    float TURNLO135_MAX_DIST;
    float TURNLO135_MIN_DIST;
   //6cm;
    float TURNRO135_MAX_DIST;
    float TURNRO135_MIN_DIST;

   // 180 degree left & right turn in straight segment;
    float TURNL180_PRE_ADJ;
    float TURNR180_PRE_ADJ;
    float TURNL180_POST_ADJ;
    float TURNR180_POST_ADJ;
   //the value about deciding the turn;
   //leave the grid top 2cm;
    float TURNL180_MAX_DIST;
    float TURNL180_MIN_DIST;
   //leave the grid top 2cm;
    float TURNR180_MAX_DIST;
    float TURNR180_MIN_DIST;

    RushParam()
    {
        RushSpeed    = 0.6f;
        T180Speed    = 0.7f;
        TR180Mu  = 0.53f;
        TL180Mu  = 0.54f;

       //distance for rush action
        RushInAcclDistDbg =  PP::GridSize * 0.707f;
        RushInAcclDist = (PP::GridSize / 2.f - PP::WallThick / 2.0f - PP::TailBack);
        RushOutAcclDistDbg =  PP::GridSize * 0.707f;
        RushOutAcclDist = 0.05f;

       //the wait distance limit
        ERRDist = 0.01f;
       //TRush
        TRushDist        = PP::GridSize * 0.707f;
        TRushComDist         = PP::GridSize * 0.707f - 0.018f;

       //6.2cm
        TRushL_MAX_DIST      = 0.14f;
        TRushL_MIN_DIST      = 0.115f;
       //6cm
        TRushR_MAX_DIST   = 0.155f;
        TRushR_MIN_DIST   = 0.115f;

       // 45 degree left & right turn in straight segment
        TURNLI45_PRE_ADJ  = 0.03f;
        TURNRI45_PRE_ADJ  = 0.025f;
        TURNLI45_POST_ADJ = 0.02f;
        TURNRI45_POST_ADJ = 0.02f;
       //the value about deciding the turn

       //leave the grid top 2cm
        TURNLI45_MAX_DIST   = 0.11f;
        TURNLI45_MIN_DIST   = 0.115f;
       //leave the grid top 2cm
        TURNRI45_MAX_DIST   = 0.14f;
        TURNRI45_MIN_DIST   = 0.115f;

       //45 degree left & right turn out straight segment
        TURNLO45_PRE_ADJ  = 0.06f;
        TURNRO45_PRE_ADJ  = 0.065f;
        TURNLO45_POST_ADJ = -0.025f;
        TURNRO45_POST_ADJ = -0.03f;
       //6.2cm
        TURNLO45_MAX_DIST = 0.14f;
        TURNLO45_MIN_DIST = 0.14f;
       //6cm
        TURNRO45_MAX_DIST = 0.155f;
        TURNRO45_MIN_DIST = 0.14f;


       // 90 degree left & right turn in straight segment(90r)
        TURNL90R_PRE_ADJ  = 0.06f;
        TURNR90R_PRE_ADJ  = 0.079f;
        TURNL90R_POST_ADJ = 0.016f;
        TURNR90R_POST_ADJ = 0.017f;

       //the value about deciding the turn
       //leave the grid top 2cm
        TURNL90R_MAX_DIST = 0.11f;
        TURNL90R_MIN_DIST = 0.09f;
       //leave the grid top 2cm
        TURNR90R_MAX_DIST = 0.14f;
        TURNR90R_MIN_DIST = 0.115f;

       // 90 degree left & right turn in straight segment(90t)
        TURNL90T_PRE_ADJ  = 0.032f;
        TURNR90T_PRE_ADJ  = 0.045f;
        TURNL90T_POST_ADJ = -0.028f;
        TURNR90T_POST_ADJ = -0.025f;

       //TODO
       //the value about deciding the turn
       //leave the grid top 2cm
        TURNL90T_MAX_DIST = 0.11f;
        TURNL90T_MIN_DIST = 0.09f;
       //leave the grid top 2cm
        TURNR90T_MAX_DIST = 0.14f;
        TURNR90T_MIN_DIST = 0.115f;

       // 135 degree left & right turn in straight segment
        TURNLI135_PRE_ADJ  = 0.10f;
        TURNRI135_PRE_ADJ  = 0.10f;
        TURNLI135_POST_ADJ = 0.005f;
        TURNRI135_POST_ADJ = 0.03f;
       //the value about deciding the turn
       //leave the grid top 2cm
        TURNLI135_MAX_DIST = 0.11f;
        TURNLI135_MIN_DIST = 0.11f;
       //leave the grid top 2cm
        TURNRI135_MAX_DIST = 0.14f;
        TURNRI135_MIN_DIST = 0.115f;

       //135 degree left & right turn out straight segment
        TURNLO135_PRE_ADJ  = 0.08f;
        TURNRO135_PRE_ADJ  = 0.11f;
        TURNLO135_POST_ADJ = 0.01f;
        TURNRO135_POST_ADJ = 0.048f;

       //the value about deciding the turn
       //6.2cm
        TURNLO135_MAX_DIST = 0.14f;
        TURNLO135_MIN_DIST = 0.10f;
       //6cm
        TURNRO135_MAX_DIST = 0.155f;
        TURNRO135_MIN_DIST = 0.10f;

       // 180 degree left & right turn in straight segment
        TURNL180_PRE_ADJ  = 0.045f;
        TURNR180_PRE_ADJ  = 0.06f;
        TURNL180_POST_ADJ = 0.02f;
        TURNR180_POST_ADJ = 0.022f;
       //the value about deciding the turn
       //leave the grid top 2cm
        TURNL180_MAX_DIST = 0.11f;
        TURNL180_MIN_DIST = 0.09f;
       //leave the grid top 2cm
        TURNR180_MAX_DIST = 0.14f;
        TURNR180_MIN_DIST = 0.115f;
    };
};

#define CORR_BACKBYIR_EN		1

struct SeachParam
{
    // static omega
    float StaticOmegaCoef ;

    //================ end basic correction coefficients

    //================ action correction parameters

    // 3 params defines how far will the side ir corr for heading dir effects during diff act
    // after these fwd end corr starts
    float HEADING_BY_SIRSIDE_START_DIST;
    float HEADING_BY_SIRSIDE_STOP_DIST ;
    float HEADING_BY_SIRSIDE_FWD_DIST  ;
    //  HEADING_BY_SIRSIDE_FWD_DIST   = 0.060f;

   // float HEADING_BY_SIRSIDE_FWD_DIST  ;
   // 2 params defines segs where side ir corr for heading dir effects during centipede;
    float HEADING_BY_SIRFWD_BGNSTAT_POS;
    float HEADING_BY_SIRFWD_BEGIN_POS  ;
    float HEADING_BY_SIRFWD_END_POS;
   //#define HEADING_BY_SIRFWD_B2EHALF_DIST ((HEADING_BY_SIRFWD_END_POS - HEADING_BY_SIRFWD_BEGIN_POS) * 0.5f);
   // left & right turn straight segment when no fwd wall for turnwait;
   // abs of these must be less than GeoFwd;
    float TURNL90_PRE_ADJ;
    float TURNR90_PRE_ADJ;
    float TURNL90_POST_ADJ;
    float TURNR90_POST_ADJ;
   // params defines distances added at fwd end corr occur;
    float LFWDEND_DIST_W2NW;
    float RFWDEND_DIST_W2NW;
   //TODO;
   // params defines distances added at fwd end corr occur;
    float LFWDEND_DIST_NW2W;
    float RFWDEND_DIST_NW2W;
   //#define FWDEND_NW2W_ENABLE    0;
   //#define FWDEND_DIST_NW2W0.045f;
    float RESTART_DIST_ADJ;
    float STOPEND_DIST_ADJ;
   // turn wait dist adjustment;
    float TURNLWAIT_DIST_ADJ;
    float TURNRWAIT_DIST_ADJ;
   // during Act Back;
    float LBACKANGLE_LRDIFF;
    float RBACKANGLE_LRDIFF;
    float LRBACKANGLE_ADJ;
   // float RBACKANGLE_ADJ ;
    float FWDDISADJ;
    float FLRYAWERROR;
    float LBACKCENTER_ADJ;
    float RBACKCENTER_ADJ;
    //================ end action correction parameters

    SeachParam()
    {
        // static omega
         StaticOmegaCoef  = 0.0f;

        //================ end basic correction coefficients

        //================ action correction parameters

        // 3 params defines how far will the side ir corr for heading dir effects during diff act
        // after these fwd end corr starts
         HEADING_BY_SIRSIDE_START_DIST = 0.030f;
         HEADING_BY_SIRSIDE_STOP_DIST  = 0.020f;
         HEADING_BY_SIRSIDE_FWD_DIST   = 0.060f;
        //  HEADING_BY_SIRSIDE_FWD_DIST   = 0.060f;

        // 2 params defines segs where side ir corr for heading dir effects during centipede
         HEADING_BY_SIRFWD_BGNSTAT_POS = 0.010f;
         HEADING_BY_SIRFWD_BEGIN_POS   = 0.120f;
         HEADING_BY_SIRFWD_END_POS= (PP::GridSize * 0.667 +  HEADING_BY_SIRFWD_BEGIN_POS * 0.333);
        //#define HEADING_BY_SIRFWD_B2EHALF_DIST ((HEADING_BY_SIRFWD_END_POS - HEADING_BY_SIRFWD_BEGIN_POS) * 0.5f)

        // left & right turn straight segment when no fwd wall for turnwait
        // abs of these must be less than GeoFwd
         TURNL90_PRE_ADJ  = -0.005f;
         TURNR90_PRE_ADJ  = -0.009f;
         TURNL90_POST_ADJ = -0.006f;
         TURNR90_POST_ADJ = -0.010f;
        // params defines distances added at fwd end corr occur
         LFWDEND_DIST_W2NW= 0.06f;
         RFWDEND_DIST_W2NW= 0.039f;
        //TODO
        // params defines distances added at fwd end corr occur
         LFWDEND_DIST_NW2W= 0.083f;
         RFWDEND_DIST_NW2W= 0.083f;
        //#define FWDEND_NW2W_ENABLE    0
        //#define FWDEND_DIST_NW2W 0.045f
         RESTART_DIST_ADJ = 0.013f;
         STOPEND_DIST_ADJ = -0.005f;
        // turn wait dist adjustment
         TURNLWAIT_DIST_ADJ    = -0.012f;    // more positive to turn later
         TURNRWAIT_DIST_ADJ    = -0.014f;    // more positive to turn later
        // during Act Back
         LBACKANGLE_LRDIFF= 1.02f;    // more positive to skew left more
         RBACKANGLE_LRDIFF= 1.f;    // more positive to skew left more
         LRBACKANGLE_ADJ  = -2.f * PP::PI / 180.f; //
        //  RBACKANGLE_ADJ   = 4.f * PP::PI / 180.f;
         FWDDISADJ   = -0.002f;
         FLRYAWERROR = 2.f * PP::PI / 180.f;
         LBACKCENTER_ADJ  = 10.f;   //
         RBACKCENTER_ADJ  = 10.f;   //
    };
};

#endif /* PHYS_PARAMS_H_ */
