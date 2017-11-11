/*
 * motion.cpp
 *
 *  Created on: Aug 5, 2014
 *      Author: loywong
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "../physparams.h"
#include "../TskMotor/TskMotor.h"

namespace TskAction
{

int MotionSeqLen(void)
{
    return (TskMotor::QMotor->Len());
}

void MotionSeqFlush(void)
{
    // clear motion seq
    TskMotor::QMotor->Clear();
}

void MotionSeqWrite(float vel, float omg)
{
    TskMotor::QMotor->En(TskMotor::VelOmega(vel,omg));
}

int MotionCalcFwd(float v0, float v1, float s, float *vs, float *accl)
{
    s *= 2.0f;
    float t = s / (v0 + v1);
    float a = (v1*v1-v0*v0)/s;
    int i, imax = t / PP::Ts;

    *accl = a;
    for(i = 1; i <= imax; i++)
    {
        vs[i - 1] = v0 + i * PP::Ts * a;
    }
    vs[i - 1] = v1;
    return imax + 1;
}

int MotionCalcRotate(float ang, float mu, float *omgs)
{
    float beta = PP::g * mu / PP::W;
    float ha = ang > 0.0f ? ang / 2.0f : ang / -2.0f;
    float omg = 0.0f, tht = 0.0f;
    int n = 0, i;
    do
    {
        omg += beta * PP::Ts;
        omgs[n++] = ang > 0.0f ? omg : -omg;
        tht += omg * PP::Ts;
    }while(tht < ha);
    for(i = n - 2; i >= 0; i--)
    {
        omgs[2 * n - 2 - i] = omgs[i];
    }
    omgs[2 * n - 1] = 0.0f;
    return 2 * n;
}

int MotionCalcTurn(float v, float ang, float mu, float *omgs, float *requ)
{
    if(v < 0.001f && v > -0.001f)
        return MotionCalcRotate(ang, mu, omgs);

        float ha = ang > 0.0f ? ang / 2.0f : ang / -2.0f;
        float omg = 0.0f, tht = 0.0f, x = 0.0f, y = 0.0f;
        float k = sqrt(PP::W*PP::W - mu*mu * PP::H*PP::H);
        float tv = PP::W*PP::W/k/v * acosf(mu*PP::H/PP::W);
        float c = mu*PP::g*PP::W/v/k;
        float t, l;
        int n = 0, i, imax = tv/PP::Ts;
        for(i = 1; i <= imax; i++)
        {
            t = PP::Ts * i;
            l = t*v*k/PP::W/PP::W;
            omg = mu*c*PP::H/k*(cosf(l)-1.0f)+c*sinf(l);
            omgs[n++] = ang > 0.0f ? omg : -omg;
            tht += omg * PP::Ts;
            x += v * PP::Ts * cosf(tht);
            y += v * PP::Ts * sinf(tht);
            if(tht >= ha)
                break;
        }
        if(i > imax)   // large turn
        {
            do
            {
                omgs[n++] = ang > 0.0f ? omg : -omg;
                tht += omg * PP::Ts;
                x += v * PP::Ts * cosf(tht);
                y += v * PP::Ts * sinf(tht);
            } while(tht < ha);
        }
        for(i = n - 2; i >= 0; i--)
        {
            omgs[2 * n - 2 - i] = omgs[i];
        }
        omgs[2 * n - 1] = 0.0f;
         *requ = x / tanf(tht) + y;
        return 2 * n;
}

}
