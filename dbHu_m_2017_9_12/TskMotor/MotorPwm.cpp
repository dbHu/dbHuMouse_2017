/*
 * MotorPwm.cc
 *
 *  Created on: Aug 3, 2016
 *      Author: loywong
 */


#include <inc/hw_memmap.h>
#include <inc/hw_pwm.h>
#include <inc/hw_types.h>
#include <inc/hw_gpio.h>
#include <inc/hw_sysctl.h>

#include <xdc/std.h>

void MotorPwmCoast()
{   // p - low, n - low
    // zero - fall

    // motor right
    //M0PWM4 gen2 A  MOTRP       
    //M0PWM5 gen2 B  MOTRN
    HWREG(PWM0_BASE | PWM_O_2_GENA) = 0x002;    // zero - fall
    HWREG(PWM0_BASE | PWM_O_2_GENB) = 0x002;    // zero - fall

    // motor left
    //M0PWM0 gen0 A MOTLN
    //M0PWM1 gen0 B MOTLP
    HWREG(PWM0_BASE | PWM_O_0_GENA) = 0x002;    // zero - fall
    HWREG(PWM0_BASE | PWM_O_0_GENB) = 0x002;    // zero - fall

    // update
    HWREG(PWM0_BASE | PWM_O_CTL) = 0x5; // update gen0, gen2
}

// r,l -> [-479, 479]
void MotorPwmSetDuty(short r, short l)
{
    //M0PWM4 gen2 A  MOTRP       
    //M0PWM5 gen2 B  MOTRN
    if(r < 0)
    {
        //          gen a -> pwm4 -> rp : -> abs(r)
        // cmp a -> gen b -> pwm5 -> rn : steady high
        HWREG(PWM0_BASE | PWM_O_2_CMPA) = -r;
        HWREG(PWM0_BASE | PWM_O_2_GENB) = 0x083;
        HWREG(PWM0_BASE | PWM_O_2_GENA) = 0x003;
    }
    else
    {
        // cmp a -> gen a -> pwm4 -> rp : steady high
        //          gen b -> pwm5 -> rn : -> abs(r)
        HWREG(PWM0_BASE | PWM_O_2_CMPA) = r;
        HWREG(PWM0_BASE | PWM_O_2_GENB) = 0x003;
        HWREG(PWM0_BASE | PWM_O_2_GENA) = 0x083;
    }

    //M0PWM0 gen0 A MOTLN
    //M0PWM1 gen0 B MOTLP
    if(l < 0)
    {
        // cmp a -> gen b -> pwm1 -> lp : -> abs(l)
        //          gen a -> pwm0 -> ln : steady high
        HWREG(PWM0_BASE | PWM_O_0_CMPA) = -l;
        HWREG(PWM0_BASE | PWM_O_0_GENB) = 0x003;
        HWREG(PWM0_BASE | PWM_O_0_GENA) = 0x083;
    }
    else
    {
        //          gen b -> pwm1 -> lp : steady high
        // cmp a -> gen a -> pwm0 -> ln : -> abs(l)
        HWREG(PWM0_BASE | PWM_O_0_CMPA) = l;
        HWREG(PWM0_BASE | PWM_O_0_GENB) = 0x083;
        HWREG(PWM0_BASE | PWM_O_0_GENA) = 0x003;
    }
    // update
    HWREG(PWM0_BASE | PWM_O_CTL) = 0x5; // update gen0, gen2
}

void MotorPwmInit()
{
    HWREG(SYSCTL_RCC) &= ~SYSCTL_RCC_USEPWMDIV; // pwm clock use sysclock
//    HWREG(SYSCTL_RCGCPWM) = HWREG(SYSCTL_RCGCPWM) & 0xFFFFFFFC | 0x00000002;    // enable PWM1
    HWREG(SYSCTL_RCGCPWM) = 0x00000001;    // enable PWM0, disable PWM1

    HWREG(GPIO_PORTF_BASE | GPIO_O_AFSEL) |= 0x00000003;    // port f[1..0] as alternate func
    HWREG(GPIO_PORTF_BASE | GPIO_O_DEN) |= 0x00000003;    // port f[1..0] digital enable
    HWREG(GPIO_PORTF_BASE | GPIO_O_PCTL) = HWREG(GPIO_PORTF_BASE | GPIO_O_PCTL) & 0xFFFFFF00 | 0x00000066;  // port f[1..0] as af6(pwm)
 
    HWREG(GPIO_PORTG_BASE | GPIO_O_AFSEL) |= 0x00000003;    // port g[1..0] as alternate func
    HWREG(GPIO_PORTG_BASE | GPIO_O_DEN) |= 0x00000003;    // port g[1..0] digital enable
    HWREG(GPIO_PORTG_BASE | GPIO_O_PCTL) = HWREG(GPIO_PORTG_BASE | GPIO_O_PCTL) & 0xFFFFFF00 | 0x00000066;  // port g[1..0] as af6(pwm)

//    HWREG(PWM1_BASE | PWM_O_ENUPD) = 0x0000FF00;    // pwm 4~7 enable update by glb sync

    HWREG(PWM0_BASE | PWM_O_2_LOAD) = 479;   // pwm gen 2 freq 120M/480 = 250k
    HWREG(PWM0_BASE | PWM_O_0_LOAD) = 479;   // pwm gen 0 freq 120M/480 = 250k

    HWREG(PWM0_BASE | PWM_O_2_CTL) = 0x000003FD;   // enable pwm gen 2; cnt down mode; run in debug; load, cmp, gen update by glb sync
    HWREG(PWM0_BASE | PWM_O_0_CTL) = 0x000003FD;   // enable pwm gen 0; ...

    HWREG(PWM0_BASE | PWM_O_SYNC) = 0x00000005; // sycn cnt 2 & cnt 0

    MotorPwmCoast();

    HWREG(PWM0_BASE | PWM_O_ENABLE) = 0x33; // enable output 4銆�銆�銆�
//    HWREG(PWM1_BASE | PWM_O_CTL) = 0xC; // update gen3, gen2

}

