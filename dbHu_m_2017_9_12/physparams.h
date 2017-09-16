/*
 * phys_params.h
 *
 *  Created on: Aug 7, 2014
 *      Author: loywong
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
const float Mu =     0.70f; // 0.72f comment @2015-09-16 17:17

// gravity constant
const float g =      9.80665f;

// gravity center height
const float H =      0.002f;

// half of distance between 2 tires
const float W =      0.032f;

// wheel
const float RWheel = 0.01175f;

// system sampling period, DO NOT change this independently
// if Ts changed, consider:
//  * Encoder sampling period
//  * PID controller
//  * Imu auto adj zero
//  * Motion seq calc
const float Ts =     0.001f;
//================ end of physics

//================ geometry informations
// gravity center to the center of 4 wheels
const float GvCenterFwd =   0.000f;

// all geometry of mouse related to GRAVITY CENTER
const float BodyLength =   0.087f;
const float BodyWidth =    0.054f;
const float HeadFwd =       (0.064f - GvCenterFwd);
// geometry center forward
const float GeoFwd =        (0.000f - GvCenterFwd);
// !!! straight segment must be no less then 2x GeoFwd !!!
// tail
const float TailBack =      (0.033f + GvCenterFwd);

// ir position related to gravity center
// forward ir forward dist to gravity center, approx
const float IrFFwd =        (0.0355f - GvCenterFwd);
// side ir forward dist to gravity center, approx
const float IrSFwd =        (0.0405f - GvCenterFwd);

// forward ir side dist to gravity center, approx
const float IrFSide =        0.0275f;
// side ir side dist to gravity center, approx
const float IrSSide =        0.012f;

const float IrFwdLRDist =  0.055f;

const float IrSizeAngle =  (60.f / 180.0f * 3.1415926536f);     //side ir angle

// grids
const float GridSize =         0.180f;
const float WallThick =        0.012f;
const float CenterToWall =     ((GridSize - WallThick) / 2.0f);
const float StartTotalDist =   (GridSize - WallThick / 2.0f - TailBack);
const float StartAcclDist =    0.030f;
const float StopTotalDist =    (GridSize / 2.0f);
const float StopAccDist =      0.040f;
const float RestartDist =      (GridSize / 2.0f);

//================ end geometry infos

//================ the dist to get wall info
const float GetWallDist =  0.01f;
//================
//================ action speeds & etc.
const int SeqArrayLen    = 512;
const float SearchSpeed    = 0.6f;	//0.36f comment @20150916 17:18
//================

}

namespace SP
{
	const float RushSpeed    = 0.6f;
	const float T180Speed    = 0.7f;
	const float TR180Mu       = 0.53f;
	const float TL180Mu       = 0.54f;

	//distance for rush action
	const float RushInAcclDistDbg =  PP::GridSize * 0.707f;
	const float RushInAcclDist = (PP::GridSize / 2.f - PP::WallThick / 2.0f - PP::TailBack);
	const float RushOutAcclDistDbg =  PP::GridSize * 0.707f;
	const float RushOutAcclDist = 0.05f;

	//the wait distance limit
	const float ERRDist                = 0.01f;
	//TRush
	const float TRushDist     		   = PP::GridSize * 0.707f;
	const float TRushComDist 		   = PP::GridSize * 0.707f - 0.018f;

	//6.2cm
	const float TRushL_MAX_DIST  	   = 0.14f;
	const float TRushL_MIN_DIST  	   = 0.115f;
	//6cm
	const float TRushR_MAX_DIST        = 0.155f;
	const float TRushR_MIN_DIST        = 0.115f;

	// 45 degree left & right turn in straight segment
	const float TURNLI45_PRE_ADJ       = 0.03f;
	const float TURNRI45_PRE_ADJ       = 0.025f;
	const float TURNLI45_POST_ADJ      = 0.02f;
	const float TURNRI45_POST_ADJ      = 0.02f;
	//the value about deciding the turn

	//leave the grid top 2cm
	const float TURNLI45_MAX_DIST        = 0.11f;
	const float TURNLI45_MIN_DIST        = 0.115f;
	//leave the grid top 2cm
	const float TURNRI45_MAX_DIST        = 0.14f;
	const float TURNRI45_MIN_DIST        = 0.115f;

	//45 degree left & right turn out straight segment
	const float TURNLO45_PRE_ADJ       = 0.06f;
	const float TURNRO45_PRE_ADJ       = 0.065f;
	const float TURNLO45_POST_ADJ      = -0.025f;
	const float TURNRO45_POST_ADJ      = -0.03f;
	//6.2cm
	const float TURNLO45_MAX_DIST      = 0.14f;
	const float TURNLO45_MIN_DIST      = 0.14f;
	//6cm
	const float TURNRO45_MAX_DIST      = 0.155f;
	const float TURNRO45_MIN_DIST      = 0.14f;


	// 90 degree left & right turn in straight segment(90r)
	const float TURNL90R_PRE_ADJ       = 0.06f;
	const float TURNR90R_PRE_ADJ       = 0.079f;
	const float TURNL90R_POST_ADJ      = 0.016f;
	const float TURNR90R_POST_ADJ      = 0.017f;

	//the value about deciding the turn
	//leave the grid top 2cm
	const float TURNL90R_MAX_DIST      = 0.11f;
	const float TURNL90R_MIN_DIST      = 0.09f;
	//leave the grid top 2cm
	const float TURNR90R_MAX_DIST      = 0.14f;
	const float TURNR90R_MIN_DIST      = 0.115f;

	// 90 degree left & right turn in straight segment(90t)
	const float TURNL90T_PRE_ADJ       = 0.032f;
	const float TURNR90T_PRE_ADJ       = 0.045f;
	const float TURNL90T_POST_ADJ      = -0.028f;
	const float TURNR90T_POST_ADJ      = -0.025f;

	//TODO
	//the value about deciding the turn
	//leave the grid top 2cm
	const float TURNL90T_MAX_DIST      = 0.11f;
	const float TURNL90T_MIN_DIST      = 0.09f;
	//leave the grid top 2cm
	const float TURNR90T_MAX_DIST      = 0.14f;
	const float TURNR90T_MIN_DIST      = 0.115f;

	// 135 degree left & right turn in straight segment
	const float TURNLI135_PRE_ADJ       = 0.10f;
	const float TURNRI135_PRE_ADJ       = 0.10f;
	const float TURNLI135_POST_ADJ      = 0.005f;
	const float TURNRI135_POST_ADJ      = 0.03f;
	//the value about deciding the turn
	//leave the grid top 2cm
	const float TURNLI135_MAX_DIST      = 0.11f;
	const float TURNLI135_MIN_DIST      = 0.11f;
	//leave the grid top 2cm
	const float TURNRI135_MAX_DIST      = 0.14f;
	const float TURNRI135_MIN_DIST      = 0.115f;

	//135 degree left & right turn out straight segment
	const float TURNLO135_PRE_ADJ       = 0.08f;
	const float TURNRO135_PRE_ADJ       = 0.11f;
	const float TURNLO135_POST_ADJ      = 0.01f;
	const float TURNRO135_POST_ADJ      = 0.048f;

	//the value about deciding the turn
	//6.2cm
	const float TURNLO135_MAX_DIST     = 0.14f;
	const float TURNLO135_MIN_DIST     = 0.10f;
	//6cm
	const float TURNRO135_MAX_DIST     = 0.155f;
	const float TURNRO135_MIN_DIST     = 0.10f;

	// 180 degree left & right turn in straight segment
	const float TURNL180_PRE_ADJ       = 0.045f;
	const float TURNR180_PRE_ADJ       = 0.06f;
	const float TURNL180_POST_ADJ      = 0.02f;
	const float TURNR180_POST_ADJ      = 0.022f;
	//the value about deciding the turn
	//leave the grid top 2cm
	const float TURNL180_MAX_DIST      = 0.11f;
	const float TURNL180_MIN_DIST      = 0.09f;
	//leave the grid top 2cm
	const float TURNR180_MAX_DIST      = 0.14f;
	const float TURNR180_MIN_DIST      = 0.115f;
}
#define CORR_BACKBYIR_EN		1

namespace CP
{
//================ basic correction coefficients
const float EncoderUnitCompensation = 1.010f;		// reduce to run farther
const float GyroUnitCompensation    = 1.000f;		// reduce to turn/rotate more
const float AcclUnitCompensation    = 1.012f;

// static omega
const float StaticOmegaCoef       = 0.0f;

//================ end basic correction coefficients

//================ action correction parameters

// 3 params defines how far will the side ir corr for heading dir effects during diff act
// after these fwd end corr starts
const float HEADING_BY_SIRSIDE_START_DIST = 0.030f;
const float HEADING_BY_SIRSIDE_STOP_DIST  = 0.020f;
const float HEADING_BY_SIRSIDE_FWD_DIST   = 0.060f;
//const float HEADING_BY_SIRSIDE_FWD_DIST   = 0.060f;
// 2 params defines segs where side ir corr for heading dir effects during centipede
const float HEADING_BY_SIRFWD_BGNSTAT_POS = 0.010f;
const float HEADING_BY_SIRFWD_BEGIN_POS   = 0.120f;
const float HEADING_BY_SIRFWD_END_POS     = (PP::GridSize * 0.667 +  HEADING_BY_SIRFWD_BEGIN_POS * 0.333);
//#define HEADING_BY_SIRFWD_B2EHALF_DIST ((HEADING_BY_SIRFWD_END_POS - HEADING_BY_SIRFWD_BEGIN_POS) * 0.5f)

// left & right turn straight segment when no fwd wall for turnwait
// abs of these must be less than GeoFwd
const float TURNL90_PRE_ADJ       = -0.0093f;
const float TURNR90_PRE_ADJ       = -0.008f;
const float TURNL90_POST_ADJ      = 0.022f;
const float TURNR90_POST_ADJ      = 0.018f;
// params defines distances added at fwd end corr occur
const float LFWDEND_DIST_W2NW     = 0.06f;
const float RFWDEND_DIST_W2NW     = 0.039f;
//TODO
// params defines distances added at fwd end corr occur
const float LFWDEND_DIST_NW2W     = 0.083f;
const float RFWDEND_DIST_NW2W     = 0.083f;
//#define FWDEND_NW2W_ENABLE    0
//#define FWDEND_DIST_NW2W      0.045f
const float RESTART_DIST_ADJ      = 0.013f;
const float STOPEND_DIST_ADJ      = -0.005f;
// turn wait dist adjustment
const float TURNLWAIT_DIST_ADJ    = -0.012f;    // more positive to turn later
const float TURNRWAIT_DIST_ADJ    = -0.014f;    // more positive to turn later
// during Act Back
const float LBACKANGLE_LRDIFF     = 1.02f;    // more positive to skew left more
const float RBACKANGLE_LRDIFF     = 1.f;    // more positive to skew left more
const float LRBACKANGLE_ADJ       = -2.f * PP::PI / 180.f; //
//const float RBACKANGLE_ADJ        = 4.f * PP::PI / 180.f;
const float FWDDISADJ	          = -0.002f;
const float FLRYAWERROR           = 2.f * PP::PI / 180.f;
const float LBACKCENTER_ADJ       = 10.f;   //
const float RBACKCENTER_ADJ       = 10.f;   //
//================ end action correction parameters
}
#endif /* PHYS_PARAMS_H_ */
