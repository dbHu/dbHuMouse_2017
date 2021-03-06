/*
 * IrCorr.cc
 *
 *  Created on: Aug 15, 2016
 *      Author: loywong
 */
#include <stdio.h>
#include <xdc/runtime/Assert.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/BIOS.h>

#include "TskIr.h"
#include "../TskMotor/TskMotor.h"
//#include "../TskMotor/WheelEnc.h"
#include "../dbmouse_chassis.h"
#include "../physparams.h"
#include "../TskTop/DbgUart.h"
#include "../TskTop/TskTop.h"

#include <ti/drivers/GPIO.h>

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
// blkIdx: 0~63
void eraseFlashBlock(int blkIdx)
{
    unsigned int key;
    key = (HWREG(FLASH_BOOTCFG) & FLASH_BOOTCFG_KEY) ? 0xA4420000 : 0x71D50000;
    HWREG(FLASH_FMA) = blkIdx * 0x4000;
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
    char input[128];
    bool i=0,rtn;
    int n;
    bool exit_flag = 0;
    float flnsDist[6]; float flnsInts[6]; int flnsIdx = 0;
    float frnsDist[6]; float frnsInts[6]; int frnsIdx = 0;
    float lsDist[6]; float lsInts[6]; int lsIdx = 0;
    float rsDist[6]; float rsInts[6]; int rsIdx = 0;

    int aim;

    GPIO_write(DBMOUSE_LED_0, DBMOUSE_LED_OFF);
    
    sprintf(TskTop::dbgStr, "input fwd choose FWD Corr\n");
    rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);
    Task_sleep(50);
    sprintf(TskTop::dbgStr, "input side choose SIDE Corr\n");
    rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);
    Task_sleep(50);
    sprintf(TskTop::dbgStr, "input save to SAVE params to flash\n");
    rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);
    Task_sleep(50);
    sprintf(TskTop::dbgStr, "input exit to EXIT correction\n");
    rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);
    Task_sleep(200);

    while(true){
        sprintf(TskTop::dbgStr, "Please input command...\n");
        rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(100);

        scanf("%s", input);

        if(!strcmp(input,"fwd")){
            sprintf(TskTop::dbgStr, "Arrange walls like this:\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "\t+---+---+\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "\t|       |\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "\t+       +\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "\t|   @   |\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "\t+---+---+\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "Place mouse at the begining of center line.\n\tTouch Ir when ready.\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(100);
        //    for(aim = 0.f; aim < .221f; aim += 0.03f)   // 9 times
            for(aim = 0; aim < 6; aim++){
                // while(TskMotor::DistanceAcc - distZero < aim)
                //     Task_sleep(1);
                WaitIrTouch(IrCh::SL | IrCh::SR, 2200, 1900);

                GPIO_write(DBMOUSE_LED_1, DBMOUSE_LED_ON);
                Task_sleep(2000);
                GPIO_write(DBMOUSE_LED_1, DBMOUSE_LED_OFF);
        //        if(TskMotor::DistanceAcc - distZero < 0.061f)   // 3 times

                frnsDist[frnsIdx] = flnsDist[flnsIdx] = (2.0f * PP::GridSize - PP::WallThick - PP::TailBack - PP::IrFFwd) - (float)aim * 4.f * PP::WallThick;

                flnsInts[flnsIdx] = IrInts.fl;
                frnsInts[frnsIdx] = IrInts.fr;
                Task_sleep(10);
                sprintf(TskTop::dbgStr, "\tFLns[%2d] = %4d @%3dmm\n", flnsIdx, (int)flnsInts[flnsIdx], (int)(0.5f + 1000.0f * flnsDist[flnsIdx]));
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                sprintf(TskTop::dbgStr, "\tFRns[%2d] = %4d @%3dmm\n", frnsIdx, (int)frnsInts[frnsIdx], (int)(0.5f + 1000.0f * frnsDist[frnsIdx]));
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                flnsIdx++;frnsIdx++;
            }
            GPIO_write(DBMOUSE_LED_1, DBMOUSE_LED_ON);
            sprintf(TskTop::dbgStr, "Calculating approximation...\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);

            irApprox2nd(flnsInts, flnsDist, flnsIdx, IrACs.k[0]);
            irApprox2nd(frnsInts, frnsDist, frnsIdx, IrACs.k[1]);
            Task_sleep(1000);
        }

        else if(!strcmp(input,"side")){
            sprintf(TskTop::dbgStr, "Acquiring center line data finished.\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "Place mouse like this:\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "\t+---+---+\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "\t|       |\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "\t+       +\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "\t|   @   |\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "\t+---+---+\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "Touch Ir when ready.\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            Task_sleep(10);
            WaitIrTouch(IrCh::FR, 1500, 1000);
            {
                GPIO_write(DBMOUSE_LED_1, DBMOUSE_LED_OFF);
                Task_sleep(750);
                GPIO_write(DBMOUSE_LED_1, DBMOUSE_LED_ON);
                //lsDist[lsIdx] = PP::BodyWidth * .5f - PP::IrSSide;
                lsDist[lsIdx] = rsDist[rsIdx] = PP::GridSize - PP::WallThick / 2.0f - PP::IrSSide;
                lsInts[lsIdx] = IrInts.sl;
                rsInts[rsIdx] = IrInts.sr;
                sprintf(TskTop::dbgStr, "\t  LS[%2d] = %4d @%3dmm\n", lsIdx, (int)lsInts[lsIdx], (int)(0.5f + 1000.0f * lsDist[lsIdx]));
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                sprintf(TskTop::dbgStr, "\t  RS[%2d] = %4d @%3dmm\n", rsIdx, (int)rsInts[rsIdx], (int)(0.5f + 1000.0f * rsDist[rsIdx]));
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                lsIdx++;
                rsIdx++;
                Task_sleep(250);
                //lsDist[lsIdx] = PP::BodyWidth * .5f - PP::IrSSide;
                lsDist[lsIdx] = rsDist[rsIdx] = PP::GridSize - PP::WallThick / 2.0f - PP::IrSSide;
                lsInts[lsIdx] = IrInts.sl;
                rsInts[rsIdx] = IrInts.sr;
                sprintf(TskTop::dbgStr, "\t  LS[%2d] = %4d @%3dmm\n", lsIdx, (int)lsInts[lsIdx], (int)(0.5f + 1000.0f * lsDist[lsIdx]));
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                sprintf(TskTop::dbgStr, "\t  RS[%2d] = %4d @%3dmm\n", rsIdx, (int)rsInts[rsIdx], (int)(0.5f + 1000.0f * rsDist[rsIdx]));
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                lsIdx++;
                rsIdx++;
            }

            sprintf(TskTop::dbgStr, "Acquiring center line data finished.\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "Place mouse like this:\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "\t+---+\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "\t|   |\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "\t+   +\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "\t|@  |\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "\t+---+\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "Touch Ir when ready.\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            Task_sleep(10);
            WaitIrTouch(IrCh::FR, 1500, 1000);
            {
                GPIO_write(DBMOUSE_LED_1, DBMOUSE_LED_OFF);
                Task_sleep(750);
                GPIO_write(DBMOUSE_LED_1, DBMOUSE_LED_ON);
                //lsDist[lsIdx] = PP::BodyWidth * .5f - PP::IrSSide;
                rsDist[rsIdx] = PP::GridSize - PP::WallThick - PP::BodyWidth * .5f - PP::IrSSide;
                //lsInts[lsIdx] = IrInts.sl;
                rsInts[rsIdx] = IrInts.sr;
                sprintf(TskTop::dbgStr, "\t  LS[%2d] = %4d @%3dmm\n", lsIdx, (int)lsInts[lsIdx], (int)(0.5f + 1000.0f * lsDist[lsIdx]));
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                sprintf(TskTop::dbgStr, "\t  RS[%2d] = %4d @%3dmm\n", rsIdx, (int)rsInts[rsIdx], (int)(0.5f + 1000.0f * rsDist[rsIdx]));
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                //lsIdx++;
                rsIdx++;
                Task_sleep(250);
                //lsDist[lsIdx] = PP::BodyWidth * .5f - PP::IrSSide;
                rsDist[rsIdx] = PP::GridSize - PP::WallThick - PP::BodyWidth * .5f - PP::IrSSide;
                //lsInts[lsIdx] = IrInts.sl;
                rsInts[rsIdx] = IrInts.sr;
                sprintf(TskTop::dbgStr, "\t  LS[%2d] = %4d @%3dmm\n", lsIdx, (int)lsInts[lsIdx], (int)(0.5f + 1000.0f * lsDist[lsIdx]));
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                sprintf(TskTop::dbgStr, "\t  RS[%2d] = %4d @%3dmm\n", rsIdx, (int)rsInts[rsIdx], (int)(0.5f + 1000.0f * rsDist[rsIdx]));
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                //lsIdx++;
                rsIdx++;
            }

            sprintf(TskTop::dbgStr, "Now place mouse like this:\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(10);
            sprintf(TskTop::dbgStr, "\t+---+\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "\t|   |\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "\t+   +\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "\t|  @|\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "\t+---+\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            Task_sleep(10);
            sprintf(TskTop::dbgStr, "Touch Ir when ready.\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            WaitIrTouch(IrCh::FL, 1500, 1000);
            {
                GPIO_write(DBMOUSE_LED_1, DBMOUSE_LED_OFF);
                Task_sleep(750);
                GPIO_write(DBMOUSE_LED_1, DBMOUSE_LED_ON);
                lsDist[lsIdx] = PP::GridSize - PP::WallThick - PP::BodyWidth * .5f - PP::IrSSide;
                //rsDist[rsIdx] = PP::BodyWidth * .5f - PP::IrSSide;
                lsInts[lsIdx] = IrInts.sl;
                //rsInts[rsIdx] = IrInts.sr;
                sprintf(TskTop::dbgStr, "\t  LS[%2d] = %4d @%3dmm\n", lsIdx, (int)lsInts[lsIdx], (int)(0.5f + 1000.0f * lsDist[lsIdx]));
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                sprintf(TskTop::dbgStr, "\t  RS[%2d] = %4d @%3dmm\n", rsIdx, (int)rsInts[rsIdx], (int)(0.5f + 1000.0f * rsDist[rsIdx]));
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                lsIdx++;
                //rsIdx++;
                Task_sleep(250);
                lsDist[lsIdx] = PP::GridSize - PP::WallThick - PP::BodyWidth * .5f - PP::IrSSide;
                //rsDist[rsIdx] = PP::BodyWidth * .5f - PP::IrSSide;
                lsInts[lsIdx] = IrInts.sl;
                Task_sleep(10);
                //rsInts[rsIdx] = IrInts.sr;
                sprintf(TskTop::dbgStr, "\t  LS[%2d] = %4d @%3dmm\n", lsIdx, (int)lsInts[lsIdx], (int)(0.5f + 1000.0f * lsDist[lsIdx]));
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                sprintf(TskTop::dbgStr, "\t  RS[%2d] = %4d @%3dmm\n", rsIdx, (int)rsInts[rsIdx], (int)(0.5f + 1000.0f * rsDist[rsIdx]));
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                lsIdx++;
                //rsIdx++;
            }

            sprintf(TskTop::dbgStr, "Now place mouse like this:\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(10);
            sprintf(TskTop::dbgStr, "\t+---+\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "\t|   |\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "\t+   +\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "\t| @ |\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "\t+---+\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "Touch Ir when ready.\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            WaitIrTouch(IrCh::FL | IrCh::FR, 1500, 1000);
            {
                GPIO_write(DBMOUSE_LED_1, DBMOUSE_LED_OFF);
                Task_sleep(750);
                GPIO_write(DBMOUSE_LED_1, DBMOUSE_LED_ON);
                rsDist[rsIdx] = lsDist[lsIdx] = PP::CenterToWall - PP::IrSSide;
                lsInts[lsIdx] = IrInts.sl;
                rsInts[rsIdx] = IrInts.sr;
                sprintf(TskTop::dbgStr, "\t  LS[%2d] = %4d @%3dmm\n", lsIdx, (int)lsInts[lsIdx], (int)(0.5f + 1000.0f * lsDist[lsIdx]));
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                sprintf(TskTop::dbgStr, "\t  RS[%2d] = %4d @%3dmm\n", rsIdx, (int)rsInts[rsIdx], (int)(0.5f + 1000.0f * rsDist[rsIdx]));
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                lsIdx++; rsIdx++;
                Task_sleep(250);
                rsDist[rsIdx] = lsDist[lsIdx] = PP::CenterToWall - PP::IrSSide;
                lsInts[lsIdx] = IrInts.sl;
                rsInts[rsIdx] = IrInts.sr;
                sprintf(TskTop::dbgStr, "\t  LS[%2d] = %4d @%3dmm\n", lsIdx, (int)lsInts[lsIdx], (int)(0.5f + 1000.0f * lsDist[lsIdx]));
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                sprintf(TskTop::dbgStr, "\t  RS[%2d] = %4d @%3dmm\n", rsIdx, (int)rsInts[rsIdx], (int)(0.5f + 1000.0f * rsDist[rsIdx]));
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                lsIdx++; rsIdx++;
            }
            GPIO_write(DBMOUSE_LED_1, DBMOUSE_LED_OFF);
            sprintf(TskTop::dbgStr, "Calculating approximation...\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);

            irApprox2nd(lsInts, lsDist, lsIdx, IrACs.k[2]);
            irApprox2nd(rsInts, rsDist, rsIdx, IrACs.k[3]);
            Task_sleep(1000);

        }

        else if(!strcmp(input,"save")){
            eraseFlashBlock(63);
            programFlash(63 * 1024 * 16, (unsigned int *)&IrACs.k[0], sizeof(IrApproxCoef) / 4);
            Task_sleep(1000);
            sprintf(TskTop::dbgStr, "save IR Correction result...\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            i = 1;
        }

        else if(!strcmp(input,"exit")){
            sprintf(TskTop::dbgStr, "Exit IR Correction...\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            exit_flag = 1;
        }

        if(exit_flag) break;
        Task_sleep(1000);
    }

    if(i==1){

        sprintf(TskTop::dbgStr, "Approximation result:\n");
        rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(50);
        Task_sleep(250);
        for(n = 0; n < 4; n++){
            // Dist = Exp(c0 + c1 * log(Int) + c2 * log^2(Int))
            sprintf(TskTop::dbgStr, "%s: d = e^(%8.5f + %8.5f*log(i) + %8.5f*log^2(i))\n", IrChNames[n], IrACs.k[n][0], IrACs.k[n][1], IrACs.k[n][2]);
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(250);
        }
        sprintf(TskTop::dbgStr, "Calculating lookup table...\n");
        rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(250);

        sprintf(TskTop::dbgStr, "Calculate lookup table finished.\n");
        rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);

        Task_sleep(200);
        sprintf(TskTop::dbgStr, "\n+--------------------------+\n");
        rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(50);
        sprintf(TskTop::dbgStr, "|  !!! Congratulation !!!  |\n");
        rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(50);
        sprintf(TskTop::dbgStr, "|  Ir correction finished. |\n");
        rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(50);
        sprintf(TskTop::dbgStr, "+--------------------------+\n");
        rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(100);
    }
    TskTop::Mode = TskTop::MouseMode::Idle;
}
}
