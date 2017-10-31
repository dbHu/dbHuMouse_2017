/*
 * TskIr.cc
 *
 *  Created on: Aug 13, 2016
 *      Author: loywong
 */
#include <math.h>
#include <stdio.h>
#include <stdint.h>

#include <inc/hw_adc.h>
#include <inc/hw_sysctl.h>
#include <inc/hw_gpio.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_timer.h>
#include <inc/hw_ints.h>

#include "driverlib/interrupt.h"

#include "../TskIr/IrCorr.h"
#include "../TskIr/TskIr.h"
#include "../TskTop/DbgUart.h"
#include "TskMotor/WheelEnc.h"
#include "PinConfig/pinout.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"

namespace TskIr
{

const int tskPrio = 10;
const int tskStkSize = 1024;

SemaphoreHandle_t SemIrAdcFinish;

QueueHandle_t MbCmd;

IrLookupTable IrLTs = {0};
IrApproxCoef IrACs = {0.f};

volatile IrIntensity IrInts = {{1, 1, 1, 1}};
volatile IrDist IrDists = {{9.9f, 9.9f, 9.9f, 9.9f}};//, 9.9f, 9.9f, 9.9f, 9.9f}};
volatile IrDistBinThs IrBinThs;
volatile IrDistBins IrBins;
volatile IrHeadingYaw IrYaw;

volatile unsigned short irZeros[4] = {1, 1, 1, 1};

char dbgStr[128];

inline float irDistFwd()
{
    if(IrBins.LS == 0)
    {
        if(IrBins.RS)
        {
            return IrDists.FLns;
        }
        else     
            return .5f * (IrDists.FLns + IrDists.FRns);
    }
    else
    {
        if(IrBins.RS == 0)
        {
            return IrDists.FRns;
        }
        else
            return ((IrBins.LS > IrBins.RS)? IrBins.LS:IrBins.RS);
    } 
    // return .5f * ((IrBins.LS? IrDists.FLws : IrDists.FLns) + (IrBins.RS? IrDists.FRws : IrDists.FRns));

}

//          \   /
//      |           |
//     fl   l   r   fr
const char *IrChNames[] = {
        "FLns", // fwd-left ir dist to fwd wall when left wall not exist
//        "FLws", // fwd-left ir dist to fwd wall when left wall exist
        "FRns", // fwd-right ir dist to fwd wall when right wall not exist
//        "FRws", // fwd-right ir dist to fwd wall when right wall exist
        "LS  ", // left ir dist to left wall
//        "LF  ", // left ir dist to fwd wall
        "RS  ", // right ir dist to right wall
//        "RF  "  // right ir dist to fwd wall
};

volatile bool irEmitEnable = false;

volatile int irEmitterNo = DBMOUSE_IR_FL;

const int adcCh[4] = {0x3, 0x7, 0x1, 0x5};

inline void adcStart(int ch)
{
    HWREG(ADC0_BASE | ADC_O_SSMUX3) = ch;
    HWREG(ADC0_BASE | ADC_O_PSSI)   = 0x00000008;   // initiate ss3
}

inline unsigned short adcReadCode()
{
    return HWREG(ADC0_BASE | ADC_O_SSFIFO3);
}

void irTimerISR(void)
{
//    static int i = 0;
//	unsigned int key;
//	key = Hwi_disable();
    BaseType_t rtn, Wake;

    HWREG(TIMER5_BASE | TIMER_O_CTL) &= ~0x00000001; // GPTM Timer A disable
    HWREG(TIMER5_BASE | TIMER_O_ICR) |= 0x00000001;      // clear TimerA timeout int

    if(irEmitterNo == 0)
    {
    	adcReadCode();
        // start adc0
        adcStart(adcCh[irEmitterNo]);
        // 0off1on
        IR_write(irEmitterNo, 0);
        irEmitterNo = (irEmitterNo + 1);
        IR_write(irEmitterNo, irEmitEnable? 1 : 0);

        HWREG(TIMER5_BASE | TIMER_O_TAV) = 1200; // GPTM Timer A value 1200 10us
        HWREG(TIMER5_BASE | TIMER_O_CTL) |= 0x00000001; // GPTM Timer A enable
    }
    else if(irEmitterNo < 3)
    {
        // read adc 0, 1
        IrInts.ch[irEmitterNo - 1] = adcReadCode();// - irZeros[irEmitterNo - 1];
        // start adc 1, 2
        adcStart(adcCh[irEmitterNo]);
        // 1off2on, 2off3on
        IR_write(irEmitterNo, 0);
        irEmitterNo = (irEmitterNo + 1);
        IR_write(irEmitterNo, irEmitEnable? 1 : 0);
        HWREG(TIMER5_BASE | TIMER_O_TAV) = 1200; // GPTM Timer A value 1200 10us
        HWREG(TIMER5_BASE | TIMER_O_CTL) |= 0x00000001; // GPTM Timer A enable
    }
    else if(irEmitterNo == 3)
    {
        // read adc2
        IrInts.ch[irEmitterNo - 1] = adcReadCode();// - irZeros[irEmitterNo - 1];
        // start adc3
        adcStart(adcCh[irEmitterNo]);
        // 3off
        IR_write(irEmitterNo, 0);
        irEmitterNo = (irEmitterNo + 1);
        HWREG(TIMER5_BASE | TIMER_O_TAV) = 1200; // GPTM Timer A value 1200 10us
        HWREG(TIMER5_BASE | TIMER_O_CTL) |= 0x00000001; // GPTM Timer A enable
    }
    else if(irEmitterNo == 4)
    {
        // read adc3
        IrInts.ch[irEmitterNo - 1] = adcReadCode();// - irZeros[irEmitterNo - 1];
        irEmitterNo = DBMOUSE_IR_FL;
        // inform tskIr
        rtn = xSemaphorePost(SemIrAdcFinish, &Wake);        
        configASSERT(rtn == pdPASS || rtn == errQUEUE_FULL);
    }

}

void irDetStart()
{
    irEmitterNo = DBMOUSE_IR_FL;
    IR_write(irEmitterNo, irEmitEnable? 1 : 0);
//    Timer_setPeriodMicroSecs(irTimer, irEmitTime);
    HWREG(TIMER5_BASE | TIMER_O_TAV) = 1200; // GPTM Timer A value 1200 10us
    HWREG(TIMER5_BASE | TIMER_O_CTL) |= 0x00000001; // GPTM Timer A enable
}

void irDetInit()
{
    HWREG(SYSCTL_RCGCTIMER) |= 0x000000020;            // enable Timer5
    SemIrAdcFinish = xSemaphoreCreateBinary();
    configASSERT(SemMotTick != NULL);
    HWREG(TIMER5_BASE | TIMER_O_CTL) &= ~0x00000001; // GPTM Timer A clear
    HWREG(TIMER5_BASE | TIMER_O_CFG) = 0x00000000;   // 32-bit timer configuration
    HWREG(TIMER5_BASE | TIMER_O_TAMR) = 0x00000002;  // Timer A period mode.
    HWREG(TIMER5_BASE | TIMER_O_TAILR) = 1200; // Timer A count down 1200  10us
    HWREG(TIMER5_BASE | TIMER_O_IMR) = 0x00000001;  // Timer A timeout interrupt
    IntRegister(INT_TIMER5A_TM4C129, irTimerISR);
    IntEnable(INT_TIMER5A_TM4C129);
    // HWREG(TIMER5_BASE | TIMER_O_CTL) |= 0x00000001;  // GPTM Timer A enable
    HWREG(TIMER5_BASE | TIMER_O_ICR) = 0x00000001;  // clear TimerA timeout int

    // init ADC
    HWREG(SYSCTL_RCGCADC) = 0x01;   // enable ADC0

    HWREG(GPIO_PORTE_BASE | GPIO_O_AFSEL) |=  0x00000001;    // PE0
    HWREG(GPIO_PORTE_BASE | GPIO_O_DEN)   &= ~0x00000001;    // PE0
    HWREG(GPIO_PORTE_BASE | GPIO_O_AMSEL) |=  0x00000001;    // PE0, AIN3

    HWREG(GPIO_PORTD_BASE | GPIO_O_AFSEL) |=  0x00000010;    // PD4
    HWREG(GPIO_PORTD_BASE | GPIO_O_DEN)   &= ~0x00000010;    // PD4
    HWREG(GPIO_PORTD_BASE | GPIO_O_AMSEL) |=  0x00000010;    // PD4, AIN7

    HWREG(GPIO_PORTE_BASE | GPIO_O_AFSEL) |=  0x00000004;    // PE2
    HWREG(GPIO_PORTE_BASE | GPIO_O_DEN)   &= ~0x00000004;    // PE2
    HWREG(GPIO_PORTE_BASE | GPIO_O_AMSEL) |=  0x00000004;    // PE2, AIN1

    HWREG(GPIO_PORTD_BASE | GPIO_O_AFSEL) |=  0x00000040;    // PD6
    HWREG(GPIO_PORTD_BASE | GPIO_O_DEN)   &= ~0x00000040;    // PD6
    HWREG(GPIO_PORTD_BASE | GPIO_O_AMSEL) |=  0x00000040;    // PD6, AIN5

    HWREG(ADC0_BASE | ADC_O_ACTSS)  = 0x00000000;   // disable all ss
    HWREG(ADC0_BASE | ADC_O_EMUX)   = 0x00000000;   // initiate by sw
    HWREG(ADC0_BASE | ADC_O_SSMUX3) = 0x00000003;   // ss3 sel ain3 1st
    HWREG(ADC0_BASE | ADC_O_SSCTL3) = 0x00000002;   // ss3 1st smp as end
    HWREG(ADC0_BASE | ADC_O_ACTSS)  = 0x00000008;   // enable ss3
}

void irCalcs()
{
    int i;
    if(!isnormal(IrACs.k[0][0]))
        return;

    // calc distance
    for(i = 0; i < 4; i++)
    {
#if(IrApproxOrder == 1)
        if(i < 2)
            IrDists.ch[i] = expf((logf(IrInts.ch[i]) - IrACs.k[i][0]) * IrACs.k[i][1]) + PP::IrFFwd;
        else
            IrDists.ch[i] = expf((logf(IrInts.ch[i]) - IrACs.k[i][0]) * IrACs.k[i][1]) + PP::IrSSide;
#elif(IrApproxOrder == 2)
        if((float)IrInts.ch[i] > 0)
        {
        	float ln = logf((float)IrInts.ch[i]);
            if(i < 2)
                IrDists.ch[i] = expf(IrACs.k[i][0] + IrACs.k[i][1] * ln + IrACs.k[i][2] * ln * ln) + PP::IrFFwd;
            else
                IrDists.ch[i] = expf(IrACs.k[i][0] + IrACs.k[i][1] * ln + IrACs.k[i][2] * ln * ln) + PP::IrSSide;
        }
        else
        {
        	IrDists.ch[i] = 9.9f;
        }

#endif
    }
    // dist binary
    for(i = 0; i < 4; i++)
    {
        if(IrBins.ch[i])    // 1: wall near
        {
            if(IrDists.ch[i] > 0.001f * (float)IrBinThs.ch[i].ThHi)
                IrBins.ch[i] = 0;
        }
        else
        {
            if(IrDists.ch[i] < 0.001f * (float)IrBinThs.ch[i].ThLo)
                IrBins.ch[i] = 1;
        }
    }
    if(IrBins.Fwd)
    {
        if(irDistFwd() > 0.001f * (float)IrBinThs.Fwd.ThHi)
            IrBins.Fwd = 0;
    }
    else
    {
        if(irDistFwd() < 0.001f * (float)IrBinThs.Fwd.ThLo)
            IrBins.Fwd = 1;
    }
    // side yaw
    IrYaw.byLS = (PP::CenterToWall - IrDists.LS) / PP::IrSFwd;
    IrYaw.byRS = (IrDists.RS - PP::CenterToWall) / PP::IrSFwd;
//    IrYaw.byLRSF = (IrDists.LF - IrDists.RF) / (8.0f * IR_SIDE); // 8.0f? tested
    IrYaw.byFLR = (IrDists.FLns - IrDists.FRns) / (2.f * PP::IrFSide);
}

void task(void *pvParameters)
{
    bool rtn;
    IrMsg::MsgType msg;

    int staticCnt = 0, staticCntCyced;
    unsigned short irZerosAcc[4] = {0};

    ReadFlash(63 * 1024 * 16, (unsigned char *)&IrACs.k[0], sizeof(IrApproxCoef));
    // calc reciprocal of approx coef b, prepare for Ir Distance calculting
#if(IrApproxOrder == 1)
    for(int i = 0; i < 4; i++)
        IrACs.k[i][1] = 1.f / IrACs.k[i][1];
#elif(IrApproxOrder == 2)
#endif

    //TODO
    IrBins.ch[0] = 0; IrBinThs.ch[0].Th = (105 | (125 << 16));
    IrBins.ch[1] = 0; IrBinThs.ch[1].Th = (100 | (120 << 16));
    IrBins.ch[2] = 1; IrBinThs.ch[2].Th = (55 | (75 << 16));
    IrBins.ch[3] = 1; IrBinThs.ch[3].Th = (60 | (80 << 16));
    IrBins.Fwd   = 0; IrBinThs.Fwd.Th   = (100 | (125 << 16));
    // IrBins.ch[0] = 0; IrBinThs.ch[0].Th = (250 | (285 << 16));
    // IrBins.ch[1] = 0; IrBinThs.ch[1].Th = (250 | (285 << 16));
    // IrBins.ch[2] = 1; IrBinThs.ch[2].Th = (100 | (150 << 16));
    // IrBins.ch[3] = 1; IrBinThs.ch[3].Th = (100 | (150 << 16));
//	IrBins.ch[0] = 0; IrBinThs.ch[0].Th = (229 | (273 << 16));
//	IrBins.ch[1] = 0; IrBinThs.ch[1].Th = (229 | (273 << 16));
//	IrBins.ch[2] = 1; IrBinThs.ch[2].Th = (100 | (140 << 16));
//	IrBins.ch[3] = 1; IrBinThs.ch[3].Th = (100 | (140 << 16));

    irDetInit();

    while(true)
    {
//        LED_write(DBMOUSE_LED_1, DBMOUSE_LED_OFF);
        rtn=xSemaphorePend(SemIrTick, 2);
        configASSERT(rtn==pdPASS);
//        LED_write(DBMOUSE_LED_1, DBMOUSE_LED_ON);

        irDetStart();

//        LED_write(DBMOUSE_LED_1, DBMOUSE_LED_OFF);
        rtn = xSemaphorePend(SemIrAdcFinish, 2);
        configASSERT(rtn == pdPASS);
//        LED_write(DBMOUSE_LED_1, DBMOUSE_LED_ON);

        // calculate distances & bins
        irCalcs();

        if(!irEmitEnable)
        {
            staticCnt++;
            staticCntCyced = (staticCnt & 0x1FFF);  // cyced per 8.192s(@Ts=1ms)
            if(staticCntCyced == 7)
            {
                for(int i = 0; i < 4; i++)
                    irZerosAcc[i] = 0;
            }
            else if(staticCntCyced >= 8 && staticCntCyced < 520)
            {
                for(int i = 0; i < 4; i++)
                    irZerosAcc[i] += IrInts.ch[i];
            }
            else if(staticCntCyced == 520)
            {
                for(int i = 0; i < 4; i++)
                    irZeros[i] = irZerosAcc[i] * (1.f / 512.f);
            }
        }
        else
            staticCnt = 0;

        if(xQueuePend(MbCmd, &msg, (TickType_t)0))
        {
            switch(msg & 0xFFFF0000)
            {
            case IrMsg::EnableEmitt:
                irEmitEnable = true;
                break;
            case IrMsg::DisableEmitt:
                irEmitEnable = false;
                break;
            default:
                break;
            }
        }
    }
}

void Init()
{
    BaseType_t rtn;

    MbCmd = xQueueCreate(4, sizeof(IrMsg::MsgType));
    configASSERT(MbCmd);

    // Create tasks
    rtn = xTaskCreate(task, (const portCHAR *)"TopIr",
                tskStkSize, NULL, tskPrio, NULL);
    configASSERT(rtn==pdPASS);
}

// this func test ir touch frist,
// if touched, it will wait until untouch,
// otherwise, it return immediately.
bool TestIrTouch(unsigned char chMask, int hTh, int lTh)
{
    if(     ((IrInts.fl > hTh) && (chMask & IrCh::FL)) ||
            ((IrInts.fr > hTh) && (chMask & IrCh::FR)) ||
            ((IrInts.sl > hTh) && (chMask & IrCh::SL)) ||
            ((IrInts.sr > hTh) && (chMask & IrCh::SR))
            )
    {
        while(true)
        {
            vTaskDelay(50);
            if(     ((IrInts.fl < lTh) || !(chMask & IrCh::FL)) &&
                    ((IrInts.fr < lTh) || !(chMask & IrCh::FR)) &&
                    ((IrInts.sl < lTh) || !(chMask & IrCh::SL)) &&
                    ((IrInts.sr < lTh) || !(chMask & IrCh::SR))
                    )
                break;
        }
        return true;
    }
    else
    {
        return false;
    }
}

// this func will block!
void WaitIrTouch(unsigned char chMask, int hTh, int lTh)
{
    while(true)
    {
        vTaskDelay(50);
        if(     ((IrInts.fl < lTh) || !(chMask & IrCh::FL)) &&
                ((IrInts.fr < lTh) || !(chMask & IrCh::FR)) &&
                ((IrInts.sl < lTh) || !(chMask & IrCh::SL)) &&
                ((IrInts.sr < lTh) || !(chMask & IrCh::SR))
                )
            break;
    }
    while(true)
    {
        vTaskDelay(50);
        if(     ((IrInts.fl > hTh) && (chMask & IrCh::FL)) ||
                ((IrInts.fr > hTh) && (chMask & IrCh::FR)) ||
                ((IrInts.sl > hTh) && (chMask & IrCh::SL)) ||
                ((IrInts.sr > hTh) && (chMask & IrCh::SR))
                )
            break;
    }
    while(true)
    {
        vTaskDelay(50);
        if(     ((IrInts.fl < lTh) || !(chMask & IrCh::FL)) &&
                ((IrInts.fr < lTh) || !(chMask & IrCh::FR)) &&
                ((IrInts.sl < lTh) || !(chMask & IrCh::SL)) &&
                ((IrInts.sr < lTh) || !(chMask & IrCh::SR))
                )
            break;
    }
}

}
