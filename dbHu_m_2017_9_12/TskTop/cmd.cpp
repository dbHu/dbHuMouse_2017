
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Assert.h>

#include <ti/drivers/GPIO.h>
#include <ti/sysbios/BIOS.h>

#include "../physparams.h"
#include "TskTop/DbgUart.h"
#include "action/action.h"
#include "TskTop/TskTop.h"
#include "../TskMotor/TskMotor.h"
#include "../TskIr/TskIr.h"
#include "TskTop/TskTop.h"
#include "cmd.h"

void MotorStart(void)
{
    bool rtn;
    TskMotor::MotorMsg::MsgType motMsg;

    motMsg = TskMotor::MotorMsg::DisableMotors;
    rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_WAIT_FOREVER);
    Assert_isTrue(rtn,NULL);
    motMsg = TskMotor::MotorMsg::EnableAcqZeros;
    rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_WAIT_FOREVER);
    Assert_isTrue(rtn,NULL);

    Task_sleep(1000);// wait for getting imu zeros & getting ir zeros
    TskTop::SetLeds(0x0);

    sprintf(TskTop::dbgStr, "gzero:%7.3f,g:%7.3f,azero:%7.3f,a:%7.3f\n",TskMotor::GyroZZero,TskMotor::AV,TskMotor::AcclXZero,TskMotor::AcclX);
    rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_WAIT_FOREVER);
    Assert_isTrue(rtn,NULL);
    Task_sleep(50);

//  motMsg = TskMotor::MotorMsg::DisableAcqZeros;
//  rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_WAIT_FOREVER);
    motMsg = TskMotor::MotorMsg::EnableGyro;
    rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_WAIT_FOREVER);
    Assert_isTrue(rtn,NULL);
    motMsg = TskMotor::MotorMsg::EnableAccl;
    rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_WAIT_FOREVER);
    Assert_isTrue(rtn,NULL);
    motMsg = TskMotor::MotorMsg::EnableMotors;
    rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_WAIT_FOREVER);
    Assert_isTrue(rtn,NULL);

    Task_sleep(100);// wait for all HW Enable

}

void MotorStop(void)
{
    bool rtn;
    TskMotor::MotorMsg::MsgType motMsg;

    motMsg = TskMotor::MotorMsg::DisableAcqZeros;
    rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_WAIT_FOREVER);
    Assert_isTrue(rtn,NULL);
    motMsg = TskMotor::MotorMsg::DisableMotors;
    rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_WAIT_FOREVER);
    Assert_isTrue(rtn,NULL);
    motMsg = TskMotor::MotorMsg::DisableAccl;
    rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_WAIT_FOREVER);
    Assert_isTrue(rtn,NULL);
    motMsg = TskMotor::MotorMsg::DisableGyro;
    rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_WAIT_FOREVER);
    Assert_isTrue(rtn,NULL);

    Task_sleep(100);// wait for all HW Enable

}

char input[128];
void cmd_shell(void)
{
    bool exit_flag = 0,rtn;
    TskTop::SetLeds(0x0);

        /* printf goes to the UART com port */
    sprintf(TskTop::dbgStr, "\f======== Welcome to the Console ========\r\n");
    rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_WAIT_FOREVER);
    Assert_isTrue(rtn,NULL);
    Task_sleep(50);

    sprintf(TskTop::dbgStr, "Enter a command followed by return.\r\nType help for a list of commands.\r\n");
    rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_WAIT_FOREVER);
    Assert_isTrue(rtn,NULL);
    Task_sleep(50);
    /* Loop forever receiving commands */
    while(true) {
        /* Get the user's input */
        // while(input[0] == 0)
        // {
        //  DbgUartGetLine(input);
        //  if(input == NULL)
        //      Task_sleep(10);
        //  else
        //      break;
        // }
        scanf("%s",input);

        if(!strcmp(input, "pid")) {
            sprintf(TskTop::dbgStr, "start PID test\r\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            pidTest();
        }

        else if(!strcmp(input, "actionlr")) {
            sprintf(TskTop::dbgStr, "start ACTION LR turn test to change param\r\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            ActionTestLR();
        }

        else if(!strcmp(input, "actionir")) {
            sprintf(TskTop::dbgStr, "start ACTION corr by ir\r\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            ActionIr();
        }

        else if(!strcmp(input, "random")) {
            sprintf(TskTop::dbgStr, "start random mode\r\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            RandomMode();
        }

        else if (!strcmp(input, "exit")) {
            /* Exit the console task */
            sprintf(TskTop::dbgStr, "Are you sure you want to exit the console? Y/N: \r\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);

            scanf("%s",input);
            fflush(stdin);

            if ((input[0] == 'y' || input[0] == 'Y') && input[1] == 0x00) {
                sprintf(TskTop::dbgStr, "Exiting console, goodbye.\r\n");
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                exit_flag = 1;
                break;
            }
        }

        else {
            /* Print a list of valid commands. */
            sprintf(TskTop::dbgStr, "Valid commands:\r\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "- pid: start pid test.\r\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "- actionlr: start ACTION LR turn test.\r\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            Task_sleep(10);
            sprintf(TskTop::dbgStr, "- actionir: start ACTION corr by ir.\r\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "- random: start random mode.\r\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "- exit: Exit the console task.\r\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(10);
            Task_sleep(5000);
        }
        if(exit_flag)
            break;
    }
}

void pidTest(void)
{
    int i=0;
    bool rtn;
    TskTop::SetLeds(0x7);

    sprintf(TskTop::dbgStr, "triggle IR L for V pid, triggle IR R for W PID\r\n");
    rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);
    Task_sleep(50);
    while(true){
#if 1
        while(1){
            TskTop::SetLeds(0x1);
            if(TskIr::TestIrTouch(TskIr::IrCh::FL, 1600, 1200))
            {
                sprintf(TskTop::dbgStr, "start Velocity pid test\n");
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                MotorStart();
                TskTop::SetLeds(0x0);
                for(i = 0; i < 200; i++)
                    TskMotor::QMotor->En(TskMotor::VelOmega(0.2f,0.f));
                TskMotor::QMotor->En(TskMotor::VelOmega(0.f,0.f));
                sprintf(TskTop::dbgStr, "QMotor is over\n");
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                TskAction::WaitQEnd();
                Task_sleep(50);
                MotorStop();
                break;
            }

            else if(TskIr::TestIrTouch(TskIr::IrCh::FR, 1600, 1200))
            {
                sprintf(TskTop::dbgStr, "start Omega pid test\n");
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                MotorStart();
                TskTop::SetLeds(0x0);
                for(i = 0; i < 200; i++)
                    TskMotor::QMotor->En(TskMotor::VelOmega(0.f,5.f));
                TskMotor::QMotor->En(TskMotor::VelOmega(0.f,0.f));
                TskAction::WaitQEnd();
                Task_sleep(50);
                MotorStop();
                break;
            }
        }
        i = 0;
#endif
        while(i < 200)
        {
            sprintf(TskTop::dbgStr, "%4.3f,%4.3f\n",
                TskAction::Desire[i],
                TskAction::Info[i]);
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            i++;
        }
        if(i == 200) {
            Task_sleep(100);
            sprintf(TskTop::dbgStr, "over\r\n");
            i = 0;
            sprintf(TskTop::dbgStr, "%8.3f,%8.3f,%8.3f\r\n",
                  TskMotor::DistanceAcc,
                  TskMotor::DistanceAcc_en,
                  TskMotor::AngleAcc);
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);

        }

        sprintf(TskTop::dbgStr, "input other command\n");
        rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(50);
        break;
    }
}

void SendActionAndWait(TskAction::Act::ActType actMsg)
{
    bool rtn;
    TskAction::ActMsg::MsgType end_Msg;
    rtn=Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);
    rtn=Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
    Assert_isTrue(rtn,NULL);
}

void ActionTestLR(void)
{
    float pre=0.f,post=0.f;
    bool rtn;
    TskTop::SetLeds(0x7);

    sprintf(TskTop::dbgStr, "please input l, r to test LR Turn90\r\n");
    rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);
    Task_sleep(50);
    while(true)
    {
        scanf("%s",input);
        fflush(stdin);

        TskTop::SetLeds(0x2);

        switch(input[0]){
            case 'l':
                sprintf(TskTop::dbgStr, "test L Turn90\r\n");
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                sprintf(TskTop::dbgStr, "input L Pre \r\n");
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                scanf("%f",&pre);
                fflush(stdin);

                sprintf(TskTop::dbgStr, "input L Post\r\n");
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                scanf("%f",&post);
                fflush(stdin);

                CP.TURNL90_PRE_ADJ += pre;
                CP.TURNL90_POST_ADJ += post;
                sprintf(TskTop::dbgStr, "L PRE:%5.3f, L POST:%5.3f\r\n",
                    CP.TURNL90_PRE_ADJ,
                    CP.TURNL90_POST_ADJ);
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                MotorStart();
                SendActionAndWait(TskAction::Act::Start);
                SendActionAndWait(TskAction::Act::L90);
                SendActionAndWait(TskAction::Act::Stop);
                TskAction::WaitQEnd();
                Task_sleep(50);
                MotorStop();
                break;
            case 'r':
                sprintf(TskTop::dbgStr, "test R Turn90\r\n");
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                sprintf(TskTop::dbgStr, "input R Pre\r\n");
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                scanf("%f", &pre);
                fflush(stdin);

                sprintf(TskTop::dbgStr, "input R Post\r\n");
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                scanf("%f",&post);
                fflush(stdin);

                CP.TURNR90_PRE_ADJ += pre;
                CP.TURNR90_POST_ADJ += post;
                sprintf(TskTop::dbgStr, "R PRE:%5.3f, R POST:%5.3f\r\n",
                    CP.TURNR90_PRE_ADJ,
                    CP.TURNR90_POST_ADJ);
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(50);
                MotorStart();
                SendActionAndWait((TskAction::Act::ActType) TskAction::Act::Start);
                SendActionAndWait((TskAction::Act::ActType) TskAction::Act::R90);
                SendActionAndWait((TskAction::Act::ActType) TskAction::Act::Stop);
                TskAction::WaitQEnd();
                Task_sleep(50);
                MotorStop();
                break;
            default:
                sprintf(TskTop::dbgStr, "Please input l or r\r\n");
                rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
                Assert_isTrue(rtn,NULL);
                Task_sleep(5000);
                break;
        }
        sprintf(TskTop::dbgStr, "%6.3f,%6.3f,%6.3f\r\n",
            TskMotor::DistanceAcc,
            TskMotor::DistanceAcc_en,
            TskMotor::AngleAcc);
        rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(50);
        sprintf(TskTop::dbgStr, "input other command\n");
        rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(50);
        break;
    }
}

/*
 *  FWD wall.left or wall.right HEADING_BY_SIRSIDE_START_DIST
 *                              LFWDEND_DIST_W2NW
 *                              RFWDEND_DIST_W2NW
 *      centipede               HEADING_BY_SIRFWD_BGNSTAT_POS
 *                              HEADING_BY_SIRFWD_BEGIN_POS
 *
 *  L90                         TURNLWAIT_DIST_ADJ
 *
 *  R90                         TURNRWAIT_DIST_ADJ
 *
 *  Restart                     RESTART_DIST_ADJ
 *
 *  Stop                        STOPEND_DIST_ADJ
 *
 *  Back                        FWDDISADJ
 *                              LRBACKANGLE_ADJ
 *                              FLRYAWERROR
 *                              LBACKCENTER_ADJ
 *                              RBACKCENTER_ADJ
 */
void ActionIr(void)
{
    bool rtn;
    float adj1=0.f,adj2=0.f,adj3=0.f,adj4=0.f,adj5=0.f;
    TskTop::SetLeds(0x7);

    sprintf(TskTop::dbgStr, "please input L90, R90, FWD, RESTART, STOP, BACK to choose action\r\n");
    rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);
    Task_sleep(50);
    while(true)
    {
        sprintf(TskTop::dbgStr, "input Action command\r\n");
        rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(50);
        scanf("%s",input);
        fflush(stdin);

        TskTop::SetLeds(0x2);

        if(!strcmp(input, "L90"))
        {
            sprintf(TskTop::dbgStr, "test L Turn90 Corr, input adj to adjust LTurnWaitDist\r\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            scanf("%f",&adj1);
            fflush(stdin);
            CP.TURNLWAIT_DIST_ADJ += adj1;
            sprintf(TskTop::dbgStr, "L ADJ DIST:%5.3f\r\n", CP.TURNLWAIT_DIST_ADJ);
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            MotorStart();
            SendActionAndWait((TskAction::Act::ActType) TskAction::Act::Start);
            SendActionAndWait((TskAction::Act::ActType) (TskAction::Act::L90 | TskAction::Act::Corr));
            SendActionAndWait((TskAction::Act::ActType) TskAction::Act::Stop);
            TskAction::WaitQEnd();
            Task_sleep(50);
            MotorStop();
            break;
        }

        else if(!strcmp(input, "R90"))
        {
            sprintf(TskTop::dbgStr, "test R Turn90 Corr, input adj to adjust RTurnWaitDist\r\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            scanf("%f",&adj1);
            fflush(stdin);
            CP.TURNRWAIT_DIST_ADJ += adj1;
            sprintf(TskTop::dbgStr, "R ADJ DIST:%5.3f\r\n", CP.TURNRWAIT_DIST_ADJ);
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            MotorStart();
            SendActionAndWait((TskAction::Act::ActType) TskAction::Act::Start);
            SendActionAndWait((TskAction::Act::ActType)(TskAction::Act::R90 | TskAction::Act::Corr));
            SendActionAndWait((TskAction::Act::ActType) TskAction::Act::Stop);
            TskAction::WaitQEnd();
            Task_sleep(50);
            MotorStop();
            break;
        }

        else if(!strcmp(input, "RESTART"))
        {
            sprintf(TskTop::dbgStr, "test RESTART Corr, input adj to adjust RestartDist\r\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            scanf("%f",&adj1);
            fflush(stdin);
            CP.RESTART_DIST_ADJ += adj1;
            sprintf(TskTop::dbgStr, "R ADJ DIST:%5.3f\r\n", CP.RESTART_DIST_ADJ);
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            MotorStart();
            SendActionAndWait((TskAction::Act::ActType) TskAction::Act::Start);
            SendActionAndWait((TskAction::Act::ActType) TskAction::Act::Stop);
            SendActionAndWait((TskAction::Act::ActType) TskAction::Act::Back);
            SendActionAndWait((TskAction::Act::ActType) (TskAction::Act::Restart | TskAction::Act::Corr));
            SendActionAndWait((TskAction::Act::ActType) TskAction::Act::Stop);
            TskAction::WaitQEnd();
            Task_sleep(50);
            MotorStop();
            break;
        }

        else if(!strcmp(input, "STOP"))
        {
            sprintf(TskTop::dbgStr, "test stop Corr, input adj to adjust StopEndDist\r\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            scanf("%f",&adj1);
            fflush(stdin);
            CP.STOPEND_DIST_ADJ += adj1;
            sprintf(TskTop::dbgStr, "R ADJ DIST:%5.3f\r\n", CP.STOPEND_DIST_ADJ);
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            MotorStart();
            SendActionAndWait((TskAction::Act::ActType) TskAction::Act::Start);
            SendActionAndWait((TskAction::Act::ActType) (TskAction::Act::Stop | TskAction::Act::Corr));
            TskAction::WaitQEnd();
            Task_sleep(50);
            MotorStop();
            break;
        }

        else if(!strcmp(input, "FWD"))
        {
            sprintf(TskTop::dbgStr, "test fwd Corr\r\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "please input StartDist change value\r\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            scanf("%f",&adj1);
            fflush(stdin);
            sprintf(TskTop::dbgStr, "please input BgnstatPos change value\r\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            scanf("%f",&adj2);
            fflush(stdin);
            sprintf(TskTop::dbgStr, "please input BeginPos change value\r\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            scanf("%f",&adj3);
            fflush(stdin);
            sprintf(TskTop::dbgStr, "please input LFwdEndDist change value\r\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            scanf("%f",&adj4);
            fflush(stdin);
            sprintf(TskTop::dbgStr, "please input RFwdEndDist change value\r\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            scanf("%f",&adj5);
            fflush(stdin);

            CP.HEADING_BY_SIRSIDE_START_DIST += adj1;
            CP.HEADING_BY_SIRFWD_BGNSTAT_POS += adj2;
            CP.HEADING_BY_SIRFWD_BEGIN_POS   += adj3;
            CP.LFWDEND_DIST_W2NW += adj4;
            CP.RFWDEND_DIST_W2NW += adj5;

            sprintf(TskTop::dbgStr, "Params as:%5.3f %5.3f %5.3f %5.3f %5.3f\r\n",
                CP.HEADING_BY_SIRSIDE_START_DIST,
                CP.HEADING_BY_SIRFWD_BGNSTAT_POS,
                CP.HEADING_BY_SIRFWD_BEGIN_POS,
                CP.LFWDEND_DIST_W2NW,
                CP.RFWDEND_DIST_W2NW);
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            MotorStart();
            SendActionAndWait(TskAction::Act::Start);
            SendActionAndWait((TskAction::Act::ActType) (TskAction::Act::Fwd | TskAction::Act::Corr));
            SendActionAndWait(TskAction::Act::Stop);
            TskAction::WaitQEnd();
            Task_sleep(50);
            MotorStop();
            break;
        }

        else if(!strcmp(input, "BACK"))
        {
            sprintf(TskTop::dbgStr, "test back Corr\r\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            sprintf(TskTop::dbgStr, "please input FwdDist change value\r\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            scanf("%f",&adj1);
            fflush(stdin);
            sprintf(TskTop::dbgStr, "please input LRBackAngle change value\r\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            scanf("%f",&adj2);
            fflush(stdin);
            sprintf(TskTop::dbgStr, "please input FLRYawError change value\r\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            scanf("%f",&adj3);
            fflush(stdin);
            sprintf(TskTop::dbgStr, "please input LBackCenter change value\r\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            scanf("%f",&adj4);
            fflush(stdin);
            sprintf(TskTop::dbgStr, "please input RBackCenter change value\r\n");
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            scanf("%f",&adj5);
            fflush(stdin);

            CP.FWDDISADJ       += adj1;
            CP.LRBACKANGLE_ADJ += adj2;
            CP.FLRYAWERROR     += adj3;
            CP.LBACKCENTER_ADJ += adj4;
            CP.RBACKCENTER_ADJ += adj5;

            sprintf(TskTop::dbgStr, "Params as:%5.3f %5.3f %5.3f %5.3f %5.3f\r\n",
                CP.FWDDISADJ,
                CP.LRBACKANGLE_ADJ,
                CP.FLRYAWERROR,
                CP.LBACKCENTER_ADJ,
                CP.RBACKCENTER_ADJ);
            rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
            Assert_isTrue(rtn,NULL);
            Task_sleep(50);
            MotorStart();
            SendActionAndWait(TskAction::Act::Start);
            SendActionAndWait(TskAction::Act::Stop);
            SendActionAndWait((TskAction::Act::ActType) (TskAction::Act::Back | TskAction::Act::Corr));
            TskAction::WaitQEnd();
            Task_sleep(50);
            MotorStop();
            break;
        }

        sprintf(TskTop::dbgStr, "%6.3f,%6.3f,%6.3f\r\n",
            TskMotor::DistanceAcc,
            TskMotor::DistanceAcc_en,
            TskMotor::AngleAcc);
        sprintf(TskTop::dbgStr, "input other command\n");
        rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(50);
        break;
    }
}

void RandomMode(void){
    int randint = 0,i = 100;
    bool rtn;

    TskTop::SetLeds(0x7);

    TskAction::Act::ActType actMsg;
    TskAction::ActMsg::MsgType end_Msg;

    sprintf(TskTop::dbgStr, "Start Random Mode\r\n");
    rtn=Mailbox_post(TskPrint::MbCmd,TskTop::dbgStr, BIOS_NO_WAIT);
    Assert_isTrue(rtn,NULL);
    Task_sleep(50);
    MotorStart();

    while(true){
        srand(i);
        randint = rand() % 3;
        //F L R
        if(randint == 0)
        {
            if(!TskIr::IrBins.Fwd)
            {
                actMsg = (TskAction::Act::ActType)(TskAction::Act::Fwd | TskAction::Act::Corr);
            }
            else if(!TskIr::IrBins.LS)
            {
                actMsg = (TskAction::Act::ActType)(TskAction::Act::L90 | TskAction::Act::Corr);
            }
            else if(!TskIr::IrBins.RS)
            {
                actMsg = (TskAction::Act::ActType)(TskAction::Act::R90 | TskAction::Act::Corr);
            }
            else
            {
                actMsg = (TskAction::Act::ActType)(TskAction::Act::TBackR | TskAction::Act::Corr);
            }
        }
        //F R L
        else if(randint == 1)
        {
            if(!TskIr::IrBins.Fwd)
            {
                actMsg = (TskAction::Act::ActType)(TskAction::Act::Fwd | TskAction::Act::Corr);
            }
            else if(!TskIr::IrBins.RS)
            {
                actMsg = (TskAction::Act::ActType)(TskAction::Act::R90 | TskAction::Act::Corr);
            }
            else if(!TskIr::IrBins.LS)
            {
                actMsg = (TskAction::Act::ActType)(TskAction::Act::L90 | TskAction::Act::Corr);
            }
            else
            {
                actMsg = (TskAction::Act::ActType)(TskAction::Act::TBackR | TskAction::Act::Corr);
            }
        }
        //L F R
        else if(randint == 2)
        {
            if(!TskIr::IrBins.LS)
            {
                actMsg = (TskAction::Act::ActType)(TskAction::Act::L90 | TskAction::Act::Corr);
            }
            else if(!TskIr::IrBins.Fwd)
            {
                actMsg = (TskAction::Act::ActType)(TskAction::Act::Fwd | TskAction::Act::Corr);
            }
            else if(!TskIr::IrBins.RS)
            {
                actMsg = (TskAction::Act::ActType)(TskAction::Act::R90 | TskAction::Act::Corr);
            }
            else
            {
                actMsg = (TskAction::Act::ActType)(TskAction::Act::TBackR | TskAction::Act::Corr);
            }
        }
        //R F L
        else if(randint == 3)
        {
            if(!TskIr::IrBins.RS)
            {
                actMsg = (TskAction::Act::ActType)(TskAction::Act::R90 | TskAction::Act::Corr);
            }
            else if(!TskIr::IrBins.Fwd)
            {
                actMsg = (TskAction::Act::ActType)(TskAction::Act::Fwd | TskAction::Act::Corr);
            }
            else if(!TskIr::IrBins.LS)
            {
                actMsg = (TskAction::Act::ActType)(TskAction::Act::L90 | TskAction::Act::Corr);
            }
            else
            {
                actMsg = (TskAction::Act::ActType)(TskAction::Act::TBackR | TskAction::Act::Corr);
            }
        }
        TskTop::actPrint(actMsg);
        rtn=Mailbox_post(TskAction::MbCmd, TskTop::dbgStr, BIOS_NO_WAIT);
        Assert_isTrue(rtn,NULL);
        Task_sleep(50);
        rtn=Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
        Assert_isTrue(rtn,NULL);
        i += 100;
    }
}
