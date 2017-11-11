/*
 * motion.h
 *
 *  Created on: Aug 5, 2014
 *      Author: loywong
 */

#ifndef ACTION_H_
#define ACTION_H_

#include "../physparams.h"
#include "FreeRTOS.h"
#include "queue.h"

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
        AcclPos  = 0x00100000,
        AcclNeg  = 0x00200000,
        MaskAccl = 0x00300000,
        DirLeft  = 0x00400000,
        IoIn     = 0x00800000,
        MaskDir  = 0x00400000,
        MaskIo   = 0x00800000,       
        Null     = 0x01000000,
        Start    = 0x02000000,
        Stop     = 0x03000000,
        Back     = 0x04000000,
        Restart  = 0x05000000,
        Fwd      = 0x06000000,
        L90      = 0x07000000,
        R90      = 0x08000000,
        ORush    = 0x11000000,
        ORushAcc = 0x11100000,
        ORushDea = 0x11200000,
        DRush    = 0x12000000,
        DRushAcc = 0x12100000,
        DRushDea = 0x12200000,
        RushStart= 0x13000000,
        RushStop = 0x14000000,
        L45i     = 0x15C00000,
        L45o     = 0x16400000,
        R45i     = 0x17800000,
        R45o     = 0x18000000,
        L90o     = 0x19000000,
        R90o     = 0x1A000000,
        L90d     = 0x1B000000,
        R90d     = 0x1C000000,
        L135i    = 0x1DC00000,
        L135o    = 0x1E400000,
        R135i    = 0x1F800000,
        R135o    = 0x20000000,
        L180     = 0x21000000,
        R180     = 0x22000000,
		TBackR   = 0x23000000
    };
};

extern QueueHandle_t MbCmd;

extern float v_s[512],o_s[512];
extern volatile float Info[640],Desire[640];

void WaitSeqEmpty(void);
void getWall(WallStatus &w);
void Init();
}

extern RushParam   SP;
extern SeachParam  CP;

//#ifdef __cplusplus
//}
//#endif /* __cplusplus */

#endif /* ACTION_H_ */
