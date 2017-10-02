/*
 * IrCorr.cc
 *
 *  Created on: Aug 15, 2016
 *      Author: loywong
 */

#include <ti/sysbios/knl/Task.h>
#include "TskIr.h"
#include "../TskMotor/TskMotor.h"
//#include "../TskMotor/WheelEnc.h"
#include "../dbmouse_chassis.h"
#include "../physparams.h"
#include "../TskTop/DbgUart.h"
#include "../TskTop/TskTop.h"

#include <inc/hw_memmap.h>
#include <inc/hw_flash.h>
#include <inc/hw_types.h>

#include <math.h>
#include <stdio.h>
#include "TskIr.h"
#include "IrCorr.h"

namespace TskIr
{

#define test_irApproSide 0
// blkIdx: 0~255
void eraseFlashBlock(int blkIdx)
{
    unsigned int key;
    key = (HWREG(FLASH_BOOTCFG) & FLASH_BOOTCFG_KEY) ? 0xA4420000 : 0x71D50000;
    HWREG(FLASH_FMA) = blkIdx * 0x400;
    HWREG(FLASH_FMC) = key | FLASH_FMC_ERASE;
    while(HWREG(FLASH_FMC) & FLASH_FMC_ERASE)
        Task_sleep(1);
}

void programFlashWord(unsigned int addr, unsigned int data)
{
    unsigned int key;
    key = (HWREG(FLASH_BOOTCFG) & FLASH_BOOTCFG_KEY) ? 0xA4420000 : 0x71D50000;
    HWREG(FLASH_FMD) = data;
    HWREG(FLASH_FMA) = addr & 0xFFFFFFFC;
    HWREG(FLASH_FMC) = key | FLASH_FMC_WRITE;
    while(HWREG(FLASH_FMC) & FLASH_FMC_WRITE)
        Task_sleep(1);
}

void programFlash(unsigned int addr, unsigned int *data, int wordLen)
{
    addr &= 0xFFFFFFFC;
    for(; wordLen > 0; wordLen--)
    {
        programFlashWord(addr, *data);
        addr += 4;
        data++;
    }
}

void ReadFlash(unsigned int addr, unsigned char *data, int byteLen)
{
    union
    {
        unsigned int d;
        unsigned char c[4];
    };
    addr &= 0xFFFFFFFC;
    for(; byteLen >= 4; byteLen -= 4, addr += 4)
    {
        d = *(volatile unsigned int *)addr;
        *(data++) = c[0];
        *(data++) = c[1];
        *(data++) = c[2];
        *(data++) = c[3];
    }
    d = *(volatile unsigned int *)addr;
    for(int i = 0; byteLen > 0; byteLen--, i++)
    {
        *(data++) = c[i];
    }
}

void linEquSolve(float *mat, int n)
{
    int k, j, i;
    // to up triangular mat
    for(k = 0; k < n - 1; k++)
    {
        // find major row
        int major = k;
        for(j = k + 1; j < n; j++)
        {
            if(mat[j * (n + 1) + k] > mat[major * (n + 1) + k])
                major = j;
        }
        // swap major row
        if(major != k)
        {
            float t;
            for(i = 0; i < n + 1; i++)
            {
                t = mat[k * (n + 1) + i];
                mat[k * (n + 1) + i] = mat[major * (n + 1) + i];
                mat[major * (n + 1) + i] = t;
            }
        }
        // eliminating column k, form row k + 1 to n - 1
        for(j = k + 1; j < n; j++)
        {
            float c = mat[j * (n + 1) + k] / mat[k * (n + 1) + k];
            for(i = k; i < n + 1; i++)
            {
                mat[j * (n + 1) + i] -= mat[k * (n + 1) + i] * c;
            }
        }
    }
    // to 1
    for(k = 0; k < n; k++)
    {
        float c = mat[k * (n + 1) + k];
        for(i = k; i < n + 1; i++)
        {
            mat[k * (n + 1) + i] /= c;
        }
    }
    //
    for(k = n - 1; k >= 1; k--)
    {
        for(j = k - 1; j >= 0; j--)
        {
            float c = mat[j * (n + 1) + k];
            for(i = k; i < n + 1; i++)
            {
                mat[j * (n + 1) + i] -= mat[k * (n + 1) + i] * c;
            }
        }
    }
}

void irApprox2nd(float *x, float *y, int n, float *coef)
{
    int i, j, k;
    float equ[3][4] = {0.f};
    for(i = 0; i < n; i++)
    {
        y[i] = logf(y[i]);
        x[i] = logf(x[i]);
    }
    for(j = 0; j < 3; j++)
    {
        for(i = 0; i < 3; i++)
        {
            for(k = 0; k < n; k++)
                equ[j][i] += powf(x[k], (float)(i + j));
        }
        for(k = 0; k < n; k++)
            equ[j][i] += powf(x[k], (float)(j)) * y[k];
    }
    Task_sleep(1);
    linEquSolve(&equ[0][0], 3);
    Task_sleep(1);
    coef[0] = equ[0][3];
    coef[1] = equ[1][3];
    coef[2] = equ[2][3];
}


void irApprox(float *x, float *y, int n, float *a, float *b)
{
    int i;
    float sumx = 0.0f, sumy = 0.0f, sumx2 = 0.0f, sumxy = 0.0f;
    float equ[2][3];
    for(i = 0; i < n; i++)
    {
        y[i] = logf(y[i]);
        x[i] = logf(x[i]);
    }
    for(i = 0; i < n; i++)
    {
        sumx += x[i];
        sumy += y[i];
        sumx2 += x[i] * x[i];
        sumxy += x[i] * y[i];
    }
    equ[0][0] = n;
    equ[0][1] = sumx;
    equ[0][2] = sumy;
    equ[1][0] = sumx;
    equ[1][1] = sumx2;
    equ[1][2] = sumxy;
    Task_sleep(1);
    linEquSolve(&equ[0][0], 2);
    Task_sleep(1);
    *a = equ[0][2];
    *b = equ[1][2];
}

#if test_irApproSide
void irApproxSide(float *x, float *y, int n, float *a, float *b)
{
    int i;
    float sumx = 0.0f, sumy = 0.0f, sumx2 = 0.0f, sumxy = 0.0f;
    float equ[2][3];
    for(i = 0; i < n; i++)
    {
        //y[i] = logf(y[i]);         distance don't use ln
        x[i] = logf(x[i]);
    }
    for(i = 0; i < n; i++)
    {
        sumx += x[i];
        sumy += y[i];
        sumx2 += x[i] * x[i];
        sumxy += x[i] * y[i];
    }
    equ[0][0] = n;
    equ[0][1] = sumx;
    equ[0][2] = sumy;
    equ[1][0] = sumx;
    equ[1][1] = sumx2;
    equ[1][2] = sumxy;
    Task_sleep(1);
    linEquSolve(&equ[0][0], 2);
    Task_sleep(1);
    *a = equ[0][2];
    *b = equ[1][2];
}
#endif

void doIrCorrection()
{
    char dbgStr[128];
    int i, n;
    float flnsDist[9]; float flnsInts[9]; int flnsIdx = 0;
//    float flwsDist[8]; float flwsInts[8]; int flwsIdx = 0;
    float frnsDist[9]; float frnsInts[9]; int frnsIdx = 0;
//    float frwsDist[8]; float frwsInts[8]; int frwsIdx = 0;
    float lsDist[9]; float lsInts[9]; int lsIdx = 0;
//    float lfDist[8]; float lfInts[8]; int lfIdx = 0;
    float rsDist[9]; float rsInts[9]; int rsIdx = 0;
//    float rfDist[8]; float rfInts[8]; int rfIdx = 0;

    float distZero;
    int aim;

    DbgUartPutLine("Arrange walls like this:\n", true);
    DbgUartPutLine("\t+---+---+\n", true);
    DbgUartPutLine("\t|       |\n", true);
    DbgUartPutLine("\t+       +\n", true);
    DbgUartPutLine("\t|   @   |\n", true);
    DbgUartPutLine("\t+---+---+\n", true);
    DbgUartPutLine("Place mouse at the begining of center line.\n\tTouch Ir when ready.\n", true);
    WaitIrTouch(IrCh::FL | IrCh::FR, 1500, 1000);
    Task_sleep(750);
    DbgUartPutLine("Place two walls every time.\n", true);
    Task_sleep(250);
//    for(aim = 0.f; aim < .221f; aim += 0.03f)   // 9 times
    for(aim = 0; aim < 9; aim++)   // 9 times
    {
        // while(TskMotor::DistanceAcc - distZero < aim)
        //     Task_sleep(1);
        WaitIrTouch(IrCh::FL | IrCh::FR, 1500, 1000);
        frnsDist[frnsIdx] = flnsDist[flnsIdx] = (2.0f * PP::GridSize - PP::WallThick - PP::TailBack - PP::IrFFwd) - 3 * PP::WallThick;
        flnsInts[flnsIdx] = IrInts.fl;
        frnsInts[frnsIdx] = IrInts.fr;
//        if(TskMotor::DistanceAcc - distZero < 0.061f)   // 3 times
        if(aim < 3)   // once
        {
            rsDist[rsIdx] = lsDist[lsIdx] = PP::GridSize - PP::WallThick / 2.0f - PP::IrSSide;
            lsInts[lsIdx] = IrInts.sl;
            rsInts[rsIdx] = IrInts.sr;
        }
        sprintf(dbgStr, "Place %2d walls:\n", aim * 2);
        DbgUartPutLine(dbgStr, true);
        sprintf(dbgStr, "\tFLns[%2d] = %4d @%3dmm\n", flnsIdx, (int)flnsInts[flnsIdx], (int)(0.5f + 1000.0f * flnsDist[flnsIdx]));
        DbgUartPutLine(dbgStr, true);
        sprintf(dbgStr, "\tFRns[%2d] = %4d @%3dmm\n", frnsIdx, (int)frnsInts[frnsIdx], (int)(0.5f + 1000.0f * frnsDist[frnsIdx]));
        DbgUartPutLine(dbgStr, true);
        flnsIdx++;frnsIdx++;
        if(aim < 3)   // 3 times
        {
            sprintf(dbgStr, "\t  LS[%2d] = %4d @%3dmm\n", lsIdx, (int)lsInts[lsIdx], (int)(0.5f + 1000.0f * lsDist[lsIdx]));
            DbgUartPutLine(dbgStr, true);
            Task_sleep(5);
            sprintf(dbgStr, "\t  RS[%2d] = %4d @%3dmm\n", rsIdx, (int)rsInts[rsIdx], (int)(0.5f + 1000.0f * rsDist[rsIdx]));
            DbgUartPutLine(dbgStr, true);
            Task_sleep(5);
            lsIdx++;rsIdx++;
        }
    }

    DbgUartPutLine("Acquiring center line data finished.\n", true);
    DbgUartPutLine("Place mouse like this:\n", true);
    DbgUartPutLine("\t+---+\n", true);
    DbgUartPutLine("\t|   |\n", true);
    DbgUartPutLine("\t+   +\n", true);
    DbgUartPutLine("\t|@  |\n", true);
    DbgUartPutLine("\t+---+\n", true);
    DbgUartPutLine("Touch Ir when ready.\n", true);
    WaitIrTouch(IrCh::FR, 1500, 1000);
    {
        Task_sleep(750);
        //lsDist[lsIdx] = PP::BodyWidth * .5f - PP::IrSSide;
        rsDist[rsIdx] = PP::GridSize - PP::WallThick - PP::BodyWidth * .5f - PP::IrSSide;
        //lsInts[lsIdx] = IrInts.sl;
        rsInts[rsIdx] = IrInts.sr;
        sprintf(dbgStr, "\t  LS[%2d] = %4d @%3dmm\n", lsIdx, (int)lsInts[lsIdx], (int)(0.5f + 1000.0f * lsDist[lsIdx]));
        DbgUartPutLine(dbgStr, true);
        sprintf(dbgStr, "\t  RS[%2d] = %4d @%3dmm\n", rsIdx, (int)rsInts[rsIdx], (int)(0.5f + 1000.0f * rsDist[rsIdx]));
        DbgUartPutLine(dbgStr, true);
        //lsIdx++;
        rsIdx++;
        Task_sleep(250);
        //lsDist[lsIdx] = PP::BodyWidth * .5f - PP::IrSSide;
        rsDist[rsIdx] = PP::GridSize - PP::WallThick - PP::BodyWidth * .5f - PP::IrSSide;
        //lsInts[lsIdx] = IrInts.sl;
        rsInts[rsIdx] = IrInts.sr;
        sprintf(dbgStr, "\t  LS[%2d] = %4d @%3dmm\n", lsIdx, (int)lsInts[lsIdx], (int)(0.5f + 1000.0f * lsDist[lsIdx]));
        DbgUartPutLine(dbgStr, true);
        sprintf(dbgStr, "\t  RS[%2d] = %4d @%3dmm\n", rsIdx, (int)rsInts[rsIdx], (int)(0.5f + 1000.0f * rsDist[rsIdx]));
        DbgUartPutLine(dbgStr, true);
        //lsIdx++;
        rsIdx++;
    }

    DbgUartPutLine("Now place mouse like this:\n", true);
    DbgUartPutLine("\t+---+\n", true);
    DbgUartPutLine("\t|   |\n", true);
    DbgUartPutLine("\t+   +\n", true);
    DbgUartPutLine("\t|  @|\n", true);
    DbgUartPutLine("\t+---+\n", true);
    DbgUartPutLine("Touch Ir when ready.\n", true);
    WaitIrTouch(IrCh::FL, 1500, 1000);
    {
        Task_sleep(750);
        lsDist[lsIdx] = PP::GridSize - PP::WallThick - PP::BodyWidth * .5f - PP::IrSSide;
        //rsDist[rsIdx] = PP::BodyWidth * .5f - PP::IrSSide;
        lsInts[lsIdx] = IrInts.sl;
        //rsInts[rsIdx] = IrInts.sr;
        sprintf(dbgStr, "\t  LS[%2d] = %4d @%3dmm\n", lsIdx, (int)lsInts[lsIdx], (int)(0.5f + 1000.0f * lsDist[lsIdx]));
        DbgUartPutLine(dbgStr, true);
        sprintf(dbgStr, "\t  RS[%2d] = %4d @%3dmm\n", rsIdx, (int)rsInts[rsIdx], (int)(0.5f + 1000.0f * rsDist[rsIdx]));
        DbgUartPutLine(dbgStr, true);
        lsIdx++;
        //rsIdx++;
        Task_sleep(250);
        lsDist[lsIdx] = PP::GridSize - PP::WallThick - PP::BodyWidth * .5f - PP::IrSSide;
        //rsDist[rsIdx] = PP::BodyWidth * .5f - PP::IrSSide;
        lsInts[lsIdx] = IrInts.sl;
        //rsInts[rsIdx] = IrInts.sr;
        sprintf(dbgStr, "\t  LS[%2d] = %4d @%3dmm\n", lsIdx, (int)lsInts[lsIdx], (int)(0.5f + 1000.0f * lsDist[lsIdx]));
        DbgUartPutLine(dbgStr, true);
        sprintf(dbgStr, "\t  RS[%2d] = %4d @%3dmm\n", rsIdx, (int)rsInts[rsIdx], (int)(0.5f + 1000.0f * rsDist[rsIdx]));
        DbgUartPutLine(dbgStr, true);
        lsIdx++;
        //rsIdx++;
    }

    DbgUartPutLine("Now place mouse like this:\n", true);
    DbgUartPutLine("\t+---+\n", true);
    DbgUartPutLine("\t|   |\n", true);
    DbgUartPutLine("\t+   +\n", true);
    DbgUartPutLine("\t| @ |\n", true);
    DbgUartPutLine("\t+---+\n", true);
    DbgUartPutLine("Touch Ir when ready.\n", true);
    WaitIrTouch(IrCh::FL | IrCh::FR, 1500, 1000);
    {
        Task_sleep(750);
        rsDist[rsIdx] = lsDist[lsIdx] = PP::CenterToWall - PP::IrSSide;
        lsInts[lsIdx] = IrInts.sl;
        rsInts[rsIdx] = IrInts.sr;
        sprintf(dbgStr, "\t  LS[%2d] = %4d @%3dmm\n", lsIdx, (int)lsInts[lsIdx], (int)(0.5f + 1000.0f * lsDist[lsIdx]));
        DbgUartPutLine(dbgStr, true);
        sprintf(dbgStr, "\t  RS[%2d] = %4d @%3dmm\n", rsIdx, (int)rsInts[rsIdx], (int)(0.5f + 1000.0f * rsDist[rsIdx]));
        DbgUartPutLine(dbgStr, true);
        lsIdx++; rsIdx++;
        Task_sleep(250);
        rsDist[rsIdx] = lsDist[lsIdx] = PP::CenterToWall - PP::IrSSide;
        lsInts[lsIdx] = IrInts.sl;
        rsInts[rsIdx] = IrInts.sr;
        sprintf(dbgStr, "\t  LS[%2d] = %4d @%3dmm\n", lsIdx, (int)lsInts[lsIdx], (int)(0.5f + 1000.0f * lsDist[lsIdx]));
        DbgUartPutLine(dbgStr, true);
        sprintf(dbgStr, "\t  RS[%2d] = %4d @%3dmm\n", rsIdx, (int)rsInts[rsIdx], (int)(0.5f + 1000.0f * rsDist[rsIdx]));
        DbgUartPutLine(dbgStr, true);
        lsIdx++; rsIdx++;
    }

    DbgUartPutLine("Calculating approximation...\n", true);
#if(IrApproxOrder == 1)
    irApprox(flnsDist, flnsInts, flnsIdx, (float *)&(IrACs.FLns[0]), (float *)&(IrACs.FLns[1]));
//    irApprox(flwsDist, flwsInts, flwsIdx, (float *)&(IrACs.FLws[0]), (float *)&(IrACs.FLws[1]));
    irApprox(frnsDist, frnsInts, frnsIdx, (float *)&(IrACs.FRns[0]), (float *)&(IrACs.FRns[1]));
//    irApprox(frwsDist, frwsInts, frwsIdx, (float *)&(IrACs.FRws[0]), (float *)&(IrACs.FRws[1]));
    irApprox(lsDist, lsInts, lsIdx, (float *)&(IrACs.LS[0]), (float *)&(IrACs.LS[1]));
//    irApprox(lfDist, lfInts, lfIdx, (float *)&(IrACs.LF[0]), (float *)&(IrACs.LF[1]));
    irApprox(rsDist, rsInts, rsIdx, (float *)&(IrACs.RS[0]), (float *)&(IrACs.RS[1]));
//    irApprox(rfDist, rfInts, rfIdx, (float *)&(IrACs.RF[0]), (float *)&(IrACs.RF[1]));
    DbgUartPutLine("Approximation result:\n", true);
    for(n = 0; n < 4; n++)
    {
        // Intensity = e^k0 * d^k1
        sprintf(dbgStr, "%s: i = e^%8f * d^%8f\n", IrChNames[n], IrACs.k[n][0], IrACs.k[n][1]);
        DbgUartPutLine(dbgStr, true);
    }
    DbgUartPutLine("Calculating lookup table...\n", true);
    float ints, dist, rat;
    rat = powf(10.f, 1.0f/(float)IrLookupTableLen);
    for(i = 0, dist = 0.02f; i < IrLookupTableLen; dist *= rat, i++)
    {
        IrLTs.Dists[i] = (unsigned short)(dist * 10000.0f + 0.5f);
        for(n = 0; n < 4; n++)
        {
            ints = powf(dist, IrACs.k[n][1]) * expf(IrACs.k[n][0]);
            if(ints >= 65534.5f)
                IrLTs.Ints[n][i] = 65535;
            else if(ints <= 0.5f)
                IrLTs.Ints[n][i] = 0;
            else
                IrLTs.Ints[n][i] = (unsigned short)(ints + 0.5f);
            Task_sleep(1);
        }
    }
    eraseFlashBlock(253);
    programFlash(253 * 1024, (unsigned int *)&IrLTs.Dists[0], (sizeof(IrLookupTable) + 3) / 4);
    programFlash(253 * 1024 + 320, (unsigned int *)&IrACs.k[0], (sizeof(IrApproxCoef) + 3) / 4);
    DbgUartPutLine("Calculate lookup table finished.\n", true);
    DbgUartPutLine("D(mm)\tIrFL\tIrFR\tIrSL\tIrSR\n", true);
    for(int i = 0; i < IrLookupTableLen; i++)
    {
        Task_sleep(200);
        sprintf(dbgStr, "%5.1f\t%4d\t%4d\t%4d\t%4d\n", (float)IrLTs.Dists[i] * .1f,
                IrLTs.FLnsInts[i], IrLTs.FRnsInts[i],
                IrLTs.LSInts[i], IrLTs.RSInts[i]);
        DbgUartPutLine(dbgStr, true);
    }
#elif(IrApproxOrder == 2)
    irApprox2nd(flnsInts, flnsDist, flnsIdx, IrACs.k[0]);
    irApprox2nd(frnsInts, frnsDist, frnsIdx, IrACs.k[1]);
    irApprox2nd(lsInts, lsDist, lsIdx, IrACs.k[2]);
    irApprox2nd(rsInts, rsDist, rsIdx, IrACs.k[3]);
    DbgUartPutLine("Approximation result:\n", true);
    Task_sleep(250);
    for(n = 0; n < 4; n++)
    {
        // Dist = Exp(c0 + c1 * log(Int) + c2 * log^2(Int))
        sprintf(dbgStr, "%s: d = e^(%8.5f + %8.5f*log(i) + %8.5f*log^2(i))\n", IrChNames[n], IrACs.k[n][0], IrACs.k[n][1], IrACs.k[n][2]);
        DbgUartPutLine(dbgStr, true);
        Task_sleep(250);
    }
    DbgUartPutLine("Calculating lookup table...\n", true);
    Task_sleep(250);
    float ints, dist, rat;
    float a, b, c;
    rat = powf(10.f, 1.0f/(float)IrLookupTableLen);
    for(i = 0, dist = 0.02f; i < IrLookupTableLen; dist *= rat, i++)
    {
        IrLTs.Dists[i] = (unsigned short)(dist * 10000.0f + 0.5f);
        for(n = 0; n < 4; n++)
        {
            a = IrACs.k[n][2]; b = IrACs.k[n][1]; c = IrACs.k[n][0] - logf(dist);
            ints = expf((-b - sqrtf(b * b - 4.f * a * c)) / (2.f * a));
            if(ints >= 65534.5f)
                IrLTs.Ints[n][i] = 65535;
            else if(ints <= 0.5f)
                IrLTs.Ints[n][i] = 0;
            else
                IrLTs.Ints[n][i] = (unsigned short)(ints + 0.5f);
            Task_sleep(1);
        }
    }
    eraseFlashBlock(253);
    programFlash(253 * 1024, (unsigned int *)&IrLTs.Dists[0], (sizeof(IrLookupTable) + 3) / 4);
    programFlash(253 * 1024 + 320, (unsigned int *)&IrACs.k[0], (sizeof(IrApproxCoef) + 3) / 4);
    DbgUartPutLine("Calculate lookup table finished.\n", true);
    DbgUartPutLine("D(mm)\tIrFL\tIrFR\tIrSL\tIrSR\n", true);
    for(int i = 0; i < IrLookupTableLen; i++)
    {
        Task_sleep(250);
        sprintf(dbgStr, "%5.1f\t%4d\t%4d\t%4d\t%4d\n", (float)IrLTs.Dists[i] * .1f,
                IrLTs.FLnsInts[i], IrLTs.FRnsInts[i],
                IrLTs.LSInts[i], IrLTs.RSInts[i]);
        DbgUartPutLine(dbgStr, true);
    }
#endif
    Task_sleep(200);
    DbgUartPutLine("\n+--------------------------+\n", true);
    DbgUartPutLine("|  !!! Congratulation !!!  |\n", true);
    DbgUartPutLine("|  Ir correction finished. |\n", true);
    DbgUartPutLine("+--------------------------+\n", true);
    Task_sleep(100);
    TskTop::Mode = TskTop::MouseMode::Idle;
}
}
