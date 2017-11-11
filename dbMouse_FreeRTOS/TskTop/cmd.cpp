
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"

#include "../physparams.h"
#include "TskTop/DbgUart.h"
#include "action/action.h"
#include "TskTop/TskTop.h"
#include "../TskMotor/TskMotor.h"
#include "../TskIr/TskIr.h"
#include "TskTop/TskTop.h"
#include "cmd.h"
#include "paramsCorr.h"

char *cmd;

void print(char *dbgStr)
{
    bool rtn;
    rtn = xQueuePost(TskPrint::MbCmd, dbgStr, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
    vTaskDelay(50);
}

void MotorStart(void)
{
    bool rtn;
    TskMotor::MotorMsg::MsgType motMsg;

    motMsg = TskMotor::MotorMsg::DisableMotors;
    rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);

    motMsg = TskMotor::MotorMsg::EnableAcqZeros;
    rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);

    vTaskDelay(1000);// wait for getting imu zeros & getting ir zeros
    TskTop::SetLeds(0x0);

//  motMsg = TskMotor::MotorMsg::DisableAcqZeros;
//  rtn=Mailbox_post(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    motMsg = TskMotor::MotorMsg::EnableGyro;
    rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
    motMsg = TskMotor::MotorMsg::EnableAccl;
    rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
    motMsg = TskMotor::MotorMsg::EnableMotors;
    rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);

    vTaskDelay(100);// wait for all HW Enable

}

void MotorStop(void)
{
    bool rtn;
    TskMotor::MotorMsg::MsgType motMsg;

    motMsg = TskMotor::MotorMsg::DisableAcqZeros;
    rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
    motMsg = TskMotor::MotorMsg::DisableMotors;
    rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
    motMsg = TskMotor::MotorMsg::DisableAccl;
    rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
    motMsg = TskMotor::MotorMsg::DisableGyro;
    rtn = xQueuePost(TskMotor::MbCmd, &motMsg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);

    vTaskDelay(100);// wait for all HW Enable

}

char input[128];
void cmd_shell(void)
{
    bool exit_flag = 0,rtn;
    TskTop::SetLeds(0x0);

        /* printf goes to the UART com port */
    sprintf(TskTop::dbgStr, "\f======== Welcome to the Console ========\r\n");
    rtn = xQueuePost(TskPrint::MbCmd, TskTop::dbgStr, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
    vTaskDelay(50);

    sprintf(TskTop::dbgStr, "Enter a command followed by return.\r\nType help for a list of commands.\r\n");
    rtn = xQueuePost(TskPrint::MbCmd, TskTop::dbgStr, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
    vTaskDelay(50);
    /* Loop forever receiving commands */
    while(true) {
 
        TskPrint::UartGetLine(input);

        if(!strcmp(input, "pid")) {
            sprintf(TskTop::dbgStr, "start PID test\r\n");
            print(TskTop::dbgStr);
            pidTest();
        }

        else if(!strcmp(input, "action")) {
            sprintf(TskTop::dbgStr, "start ACTION corr by ir\r\n");
            print(TskTop::dbgStr);
            Action();
        }

        else if(!strcmp(input, "random")) {
            sprintf(TskTop::dbgStr, "start random mode\r\n");
            print(TskTop::dbgStr);
            RandomMode();
        }

        else if(!strcmp(input, "params")) {
            sprintf(TskTop::dbgStr, "start paramsCorr\r\n");
            print(TskTop::dbgStr);
            paramsCorr();
        }

        else if (!strcmp(input, "exit")) {
            /* Exit the console task */
            sprintf(TskTop::dbgStr, "Are you sure you want to exit the console? Y/N: \r\n");
            print(TskTop::dbgStr);

            TskPrint::UartGetLine(input);

            if ((input[0] == 'y' || input[0] == 'Y') && input[1] == 0x00) {
                sprintf(TskTop::dbgStr, "Exiting console, goodbye.\r\n");
                print(TskTop::dbgStr);
                exit_flag = 1;
                break;
            }
        }

        else {
            /* Print a list of valid commands. */
            sprintf(TskTop::dbgStr, "Valid commands:\r\n");
            print(TskTop::dbgStr);
            sprintf(TskTop::dbgStr, "- pid: start pid test.\r\n");
            print(TskTop::dbgStr);
            sprintf(TskTop::dbgStr, "- action: start ACTION .\r\n");
            print(TskTop::dbgStr);
            sprintf(TskTop::dbgStr, "- random: start random mode.\r\n");
            print(TskTop::dbgStr);
            sprintf(TskTop::dbgStr, "- params: start paramsCorr.\r\n");
            print(TskTop::dbgStr);
            sprintf(TskTop::dbgStr, "- exit: Exit the console task.\r\n");
            print(TskTop::dbgStr);
            vTaskDelay(5000);
        }
        if(exit_flag)
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
void paramsCorr(void)
{
    float temp;
    bool exit_flag = 0;
    const char *d = " ";

    sprintf(TskTop::dbgStr, "please input as  the format, don't more than 8 params\r\n");
    print(TskTop::dbgStr);
    sprintf(TskTop::dbgStr, "Format:    str num ...\r\n");
    print(TskTop::dbgStr);
    sprintf(TskTop::dbgStr, "String:    params to change\r\n");
    print(TskTop::dbgStr);
    sprintf(TskTop::dbgStr, "num:       value to change\r\n");
    print(TskTop::dbgStr);
    while(1)
    {
        TskPrint::UartGetLine(input);

        cmd = strtok(input, d);

        for(int i = 0; i < varnum; i++){
            if(!strcmp(cmd, varname[i].name)) {
                cmd = strtok(NULL, d);
                temp = atof(cmd);
                if(isnan(temp))
                    temp = *varname[i].value;
                sprintf(TskTop::dbgStr, "\t %s = %6.3f (was %6.3f)\r\n", varname[i].name,
                        *varname[i].value + temp, *varname[i].value);
                print(TskTop::dbgStr);
                *varname[i].value += temp;
            }
            else if(!strcmp(cmd, "exit")) {
                exit_flag = 1;
                break;
            }
        }

        sprintf(TskTop::dbgStr, "input other params\n");
        print(TskTop::dbgStr);

        if(exit_flag){
            sprintf(TskTop::dbgStr, "input other command\n");
            print(TskTop::dbgStr);
            break;
        }
    }
}

void pidTest(void)
{
    int i=0,cmd;
    TskTop::SetLeds(0x7);

    sprintf(TskTop::dbgStr, "please input NUMBER for pid test\r\n");
    print(TskTop::dbgStr);
    TskPrint::UartGetLine(input);
    cmd = atof(input);

    sprintf(TskTop::dbgStr, "triggle IR L for V pid, triggle IR R for W PID\r\n");
    print(TskTop::dbgStr);

    while(true){
#if 1
        while(1){
            TskTop::SetLeds(0x1);
            if(TskIr::TestIrTouch(TskIr::IrCh::FL, 1600, 1200))
            {
                sprintf(TskTop::dbgStr, "start Velocity pid test\n");
                print(TskTop::dbgStr);
                MotorStart();
                TskTop::SetLeds(0x0);
                for(i = 0; i < cmd; i++)
                    TskMotor::QMotor->En(TskMotor::VelOmega(0.2f,0.f));
                TskMotor::QMotor->En(TskMotor::VelOmega(0.f,0.f));
                sprintf(TskTop::dbgStr, "QMotor is over\n");
                print(TskTop::dbgStr);
                TskAction::WaitSeqEmpty();
                vTaskDelay(50);
                MotorStop();
                break;
            }

            else if(TskIr::TestIrTouch(TskIr::IrCh::FR, 1600, 1200))
            {
                sprintf(TskTop::dbgStr, "start Omega pid test\n");
                print(TskTop::dbgStr);
                MotorStart();
                TskTop::SetLeds(0x0);
                for(i = 0; i < cmd; i++)
                    TskMotor::QMotor->En(TskMotor::VelOmega(0.f,5.f));
                TskMotor::QMotor->En(TskMotor::VelOmega(0.f,0.f));
                TskAction::WaitSeqEmpty();
                vTaskDelay(50);
                MotorStop();
                break;
            }
        }
        i = 0;
#endif
        sprintf(TskTop::dbgStr, "input other command\n");
        print(TskTop::dbgStr);
        break;
    }
}

void SendActionAndWait(TskAction::Act::ActType actMsg)
{
    bool rtn;
    TskAction::ActMsg::MsgType end_Msg;
    rtn = xQueuePost(TskAction::MbCmd, &actMsg, 0);
    configASSERT(rtn == pdPASS);
    rtn = xQueuePend(TskTop::MbCmd, &end_Msg, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
}

void Action(void)
{
    bool act_flag = 0;
    TskTop::SetLeds(0x7);
    char *d = " ";
    char str[20];
    int num;
    TskAction::Act::ActType act[10];

    sprintf(TskTop::dbgStr, "please input action as format\r\n");
    print(TskTop::dbgStr);
    sprintf(TskTop::dbgStr, "format:[num start act... stop]\r\n");
    print(TskTop::dbgStr);

    while(true)
    {
        sprintf(TskTop::dbgStr, "please input action as format\r\n");
        print(TskTop::dbgStr);
        sprintf(TskTop::dbgStr, "[num] [s or r] [action] ...\r\n");
        print(TskTop::dbgStr);
        act_flag = 0;
        TskPrint::UartGetLine(input);
        cmd = strtok(input, d);

        if(!strcmp(cmd, "exit")){
            sprintf(TskTop::dbgStr, "exit action\r\n");
            print(TskTop::dbgStr);
            sprintf(TskTop::dbgStr, "input other command\r\n");
            print(TskTop::dbgStr);
            break;
        }

        num = (*cmd - '0');
        if(num > 9){
            sprintf(TskTop::dbgStr, "over the limitation\r\n");
            print(TskTop::dbgStr);
            num = 0;
        }
        TskTop::SetLeds(0x0);

        cmd = strtok(NULL, d);
        if(!strcmp(cmd, "s"))
        {
           act[0] = TskAction::Act::Start;
           act[num + 1] = TskAction::Act::Stop;
        }
        else if(!strcmp(cmd, "r"))
        {
           act[0] = TskAction::Act::RushStart;
           act[num + 1] = TskAction::Act::RushStop;
        }
        else
        {
           num = 0;
        }

        for(int i = 1; i < num + 1; i++)
        {
            act_flag = 0;
            cmd = strtok(NULL, d);
            for(int j = 0; j < actnum; j++)
            {
                str[0] = '\0';
                strcat(str, actname[j].name);
                strcat(str, "ir");
                if(!strcmp(cmd, actname[j].name))
                {
                    act[i] = actname[j].action;
                    act_flag = 1;
                    break;
                }
                else if(!strcmp(cmd, str))
                {
                    act[i] = (TskAction::Act::ActType)(actname[j].action | (TskAction::Act::Corr));
                    act_flag = 1;
                    break;
                }
            }

            if(!act_flag) {
                sprintf(TskTop::dbgStr, "%s doesn't exist", cmd);
                print(TskTop::dbgStr);
                break;
            } 
        }

        if((act[num] == TskAction::Act::Back)
           ||(act[num] == (TskAction::Act::Back | TskAction::Act::Corr))
           ||(act[num] == TskAction::Act::Stop)
           ||(act[num] ==  (TskAction::Act::Stop | TskAction::Act::Corr))
           || (act[num] == TskAction::Act::RushStop))
        {
            act[num + 1] = TskAction::Act::Null;
        }

        if(act_flag){
            MotorStart();
            for(int i = 0; i < num + 2; i++)
            {
                SendActionAndWait(act[i]);
            }
            MotorStop();
        }

    }
}

void RandomMode(void){
    int randint = 0;
    char wall[4];
    bool rtn;

    TskTop::SetLeds(0x7);

    TskAction::Act::ActType actMsg;

    MotorStart();
    SendActionAndWait(((TskAction::Act::ActType) TskAction::Act::Start));
    while(true){
        randint = TskIr::IrInts.ch[0];
//        randint = 0;
        wall[0] = '#';
        wall[1] = '#';
        wall[2] = '#';
        //L F R
        if((randint % 4) == 0)
        {
            if(!TskIr::IrBins.LS)
            {
                wall[0] = '_';
                actMsg = (TskAction::Act::ActType)(TskAction::Act::L90 | TskAction::Act::Corr);
            }
            else if(!TskIr::IrBins.Fwd)
            {
                TskTop::SetLeds(0x1);
                wall[1] = '_';
                actMsg = (TskAction::Act::ActType)(TskAction::Act::Fwd | TskAction::Act::Corr);
            }
            else if(!TskIr::IrBins.RS)
            {
                wall[2] = '_';
                actMsg = (TskAction::Act::ActType)(TskAction::Act::R90 | TskAction::Act::Corr);
            }
            else
            {
                actMsg = (TskAction::Act::ActType)(TskAction::Act::TBackR | TskAction::Act::Corr);
            }
        }

        //R F L
        else if((randint % 4) == 1)
        {
            if(!TskIr::IrBins.RS)
            {
                wall[2] = '_';
                actMsg = (TskAction::Act::ActType)(TskAction::Act::R90 | TskAction::Act::Corr);
            }
            else if(!TskIr::IrBins.Fwd)
            {
                TskTop::SetLeds(0x1);
                wall[1] = '_';
                actMsg = (TskAction::Act::ActType)(TskAction::Act::Fwd | TskAction::Act::Corr);
            }
            else if(!TskIr::IrBins.LS)
            {
                wall[0] = '_';
                actMsg = (TskAction::Act::ActType)(TskAction::Act::L90 | TskAction::Act::Corr);
            }
            else
            {
                actMsg = (TskAction::Act::ActType)(TskAction::Act::TBackR | TskAction::Act::Corr);
            }
        }
        //F L R
        else if((randint % 4)== 2)
        {
            if(!TskIr::IrBins.Fwd)
            {
                TskTop::SetLeds(0x1);
                wall[1] = '_';
                actMsg = (TskAction::Act::ActType)(TskAction::Act::Fwd | TskAction::Act::Corr);
            }
            else if(!TskIr::IrBins.LS)
            {
                wall[0] = '_';
                actMsg = (TskAction::Act::ActType)(TskAction::Act::L90 | TskAction::Act::Corr);
            }
            else if(!TskIr::IrBins.RS)
            {
                wall[2] = '_';
                actMsg = (TskAction::Act::ActType)(TskAction::Act::R90 | TskAction::Act::Corr);
            }
            else
            {
                actMsg = (TskAction::Act::ActType)(TskAction::Act::TBackR | TskAction::Act::Corr);
            }
        }
        //F R L
        else
        {
            if(!TskIr::IrBins.Fwd)
            {
                TskTop::SetLeds(0x1);
                wall[1] = '_';
                actMsg = (TskAction::Act::ActType)(TskAction::Act::Fwd | TskAction::Act::Corr);
            }
            else if(!TskIr::IrBins.RS)
            {
                wall[2] = '_';
                actMsg = (TskAction::Act::ActType)(TskAction::Act::R90 | TskAction::Act::Corr);
            }
            else if(!TskIr::IrBins.LS)
            {
                wall[0] = '_';
                actMsg = (TskAction::Act::ActType)(TskAction::Act::L90 | TskAction::Act::Corr);
            }
            else
            {
                actMsg = (TskAction::Act::ActType)(TskAction::Act::TBackR | TskAction::Act::Corr);
            }
        }
        rtn = xQueuePost(TskPrint::MbCmd, wall, 0);
        configASSERT(rtn == pdPASS);
        if(wall[0] == '#' && wall[2] == '#')
        {
            sprintf(TskTop::dbgStr, "%6.3f", TskIr::irDistFwd());
            rtn = xQueuePost(TskPrint::MbCmd, TskTop::dbgStr, 0);
            configASSERT(rtn == pdPASS);
        }
        TskTop::SetLeds(0x0);

        TskTop::actPrint(actMsg);
        SendActionAndWait(actMsg);
    }
}
