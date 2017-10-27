/*
 * TskIr.h
 *
 *  Created on: Aug 13, 2016
 *      Author: loywong
 */

#ifndef TSKIR_TSKIR_H_
#define TSKIR_TSKIR_H_

#include "FreeRTOS.h"
#include "queue.h"

#define IrApproxOrder 2

namespace TskIr
{

class IrMsg
{
public:
    enum MsgType
    {
        EnableEmitt     = 0x00010000,
        DisableEmitt    = 0x00020000
//        DoCorrection    = 0x00030000
    };
};

class IrCh
{
public:
    enum Type
    {
        FL = 0x1,
        FR = 0x2,
        SL = 0x4,
        SR = 0x8
    };
};

union IrDist
{
    float ch[5];    // unit: m
    struct
    {
        float FLns;
//        float FLws;
        float FRns;
//        float FRws;
        float LS;
//        float LF;
        float RS;
//        float RF;
    };
};

union IrIntensity
{
    short ch[4];
    struct
    {
        short fl;
        short fr;
        short sl;
        short sr;
    };
};

const int IrLookupTableLen = 32;
#pragma pack(push)
#pragma pack(1)
struct IrLookupTable
{
    unsigned short Dists[IrLookupTableLen];   // distance unit: 0.1mm, from ir diodes
    union
    {
        unsigned short Ints[4][IrLookupTableLen];
        struct
        {
            unsigned short FLnsInts[IrLookupTableLen];
//            unsigned short FLwsInts[IrLookupTableLen];
            unsigned short FRnsInts[IrLookupTableLen];
//            unsigned short FRwsInts[IrLookupTableLen];
            unsigned short LSInts[IrLookupTableLen];
//            unsigned short LFInts[IrLookupTableLen];
            unsigned short RSInts[IrLookupTableLen];
//            unsigned short RFInts[IrLookupTableLen];
        };
    };
};

struct IrApproxCoef
{
    union
    {
        float k[4][3];
        struct
        {
            float FLns[3];
            float FRns[3];
            float LS[3];
            float RS[3];
        };
    };
};

union IrDistBinTh
{
    unsigned int Th;
    struct
    {
        unsigned short ThLo;    // unit: mm
        unsigned short ThHi;
    };
};
union IrDistBinThs
{
    IrDistBinTh ch[5];
    struct
    {
        IrDistBinTh FLns;
//        IrDistBinTh FLws;
        IrDistBinTh FRns;
//        IrDistBinTh FRws;
        IrDistBinTh LS;
//        IrDistBinTh LF;
        IrDistBinTh RS;
//        IrDistBinTh RF;
        IrDistBinTh Fwd;
    };
};
#pragma pack(pop)

union IrDistBins
{
    bool ch[5];
    struct
    {
        bool FLns;   // 1: wall near; 0: wall far
//        bool FLws;
        bool FRns;
//        bool FRws;
        bool LS;
//        bool LF;
        bool RS;
//        bool RF;
        bool Fwd;
    };
};

//union IrApproxCoef
//{
//    float Coef[4][2];
//    struct
//    {
//        float FLnsCoef[2];
////        float FLwsCoef[2];
//        float FRnsCoef[2];
////        float FRwsCoef[2];
//        float LSCoef[2];
////        float LFCoef[2];
//        float RSCoef[2];
////        float RFCoef[2];
//    };
//};

union IrHeadingYaw
{
    float ch[3];
    struct
    {
        float byLS;
        float byRS;
        float byFLR;
    };
};

extern volatile IrIntensity IrInts;
extern volatile IrDist IrDists;
extern volatile IrDistBinThs IrBinThs;
extern volatile IrDistBins IrBins;
extern volatile IrHeadingYaw IrYaw;

extern QueueHandle_t MbCmd;

extern const char *IrChNames[]; // = {"FLns", "FLws", "FRns", "FRws", "LS  ", "LF  ", "RS  ", "RF  "};

extern IrLookupTable IrLTs;
extern IrApproxCoef IrACs;

void Init();

// this func test ir touch frist,
// if touched, it will wait until untouch,
// otherwise, it return immediately.
// chMask: 0x1-fl, 0x2-fr, 0x4-sl, 0x8-sr
// hTh, lTh: th for touch and untouch, unit in IrIntensity
bool TestIrTouch(unsigned char chMask, int hTh, int lTh);
// this func will block!
void WaitIrTouch(unsigned char chMask, int hTh, int lTh);

}

#endif /* TSKIR_TSKIR_H_ */
