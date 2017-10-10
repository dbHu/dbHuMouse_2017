/*
 * motion.h
 *
 *  Created on: Aug 5, 2014
 *      Author: loywong
 */

#ifndef MOTION_H_
#define MOTION_H_

#include "../physparams.h"
#include <ti/sysbios/knl/Mailbox.h>

//#ifdef __cplusplus
//extern "C"
//{
//#endif /* __cplusplus */


namespace TskAction{

struct CorrsInfo
{
    unsigned int fwdEnd;
    float irSideDist;
    unsigned int turnWait;
    void Reset()
    {
        this->fwdEnd = 0;
        this->irSideDist = 0.0f;
        this->turnWait = 0;
    }
    CorrsInfo()
    {
        this->Reset();
    }

};

union WallStatus
{
    unsigned int msk;
    struct
    {
        unsigned int fwd    :1;   // 1: wall exist; 0: no wall
        unsigned int left   :1;
        unsigned int right  :1;
    };
};

class ActMsg
{
public:
    enum MsgType
    {
        Action_ed     = 0x00010000,
        Action_ing    = 0x00020000
//        DoCorrection    = 0x00030000
    };
};

class Act
{
public: enum ActType
    {
		Corr     = 0x00000001,
        Null     = 0x01000000,
        Start    = 0x02000000,
        Stop     = 0x03000000,
        Back     = 0x04000000,
        Restart  = 0x05000000,
        Fwd      = 0x06000000,
        L90      = 0x07000000,
        R90      = 0x08000000,
        SRush    = 0x10000000,
        CRush    = 0x11000000,
        TRush    = 0x12000000,
        RushIn   = 0x13000000,
        RushOut  = 0x14000000,
        L45i     = 0x15000000,
        L45o     = 0x16000000,
        R45i     = 0x17000000,
        R45o     = 0x18000000,
        L90r     = 0x19000000,
        R90r     = 0x1A000000,
        L90t     = 0x1B000000,
        R90t     = 0x1C000000,
        L135i    = 0x1D000000,
        L135o    = 0x1E000000,
        R135i    = 0x1F000000,
        R135o    = 0x20000000,
        L180     = 0x21000000,
        R180     = 0x22000000,
		TBackR   = 0x23000000,
		TBack    = 0x24000000,
		PRush    = 0x25000000,
    };
};

extern Mailbox_Handle MbCmd;

extern float v_s[512],o_s[512];
extern volatile float Info[512],Desire[512];

int MotionCalcFwd(float v0, float v1, float s, float *vs);
int MotionCalcTurn(float v, float ang, float mu, float *omgs, float *requ);

void Init();
}

extern RushParam   SP;
extern SeachParam  CP;

//#ifdef __cplusplus
//}
//#endif /* __cplusplus */

#endif /* MOTION_H_ */
