
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <xdc/runtime/System.h>
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

char dbgStr[80];
void cmd_shell(void)
{
	char input[20];
	bool exit_flag = 0;
	TskTop::SetLeds(0x0);

	    /* printf goes to the UART com port */
    DbgUartPutLine("\f======== Welcome to the Console ========\r\n",true);
    DbgUartPutLine("Enter a command followed by return.\r\nType help for a list of commands.\r\n",true);

    /* Loop forever receiving commands */
    while(true) {
        /* Get the user's input */
        // while(input[0] == 0)
        // {
        // 	DbgUartGetLine(input);
        // 	if(input == NULL)
        // 		Task_sleep(10);
        // 	else 
        // 		break;
        // }
    	do{
    		DbgUartGetLine(input);
    	}while(input[0] == '\0');

		if(!strcmp(input, "pid")) {
			DbgUartPutLine("start PID test\r\n", true);
			pidTest();
		}

		else if(!strcmp(input, "actionlr")) {
			DbgUartPutLine("start ACTION LR turn test to change param\r\n",true);
			ActionTestLR();
		}

		else if(!strcmp(input, "actionir")) {
			DbgUartPutLine("start ACTION corr by ir\r\n",true);
			ActionIr();
		}

		else if(!strcmp(input, "random")) {
			DbgUartPutLine("start random mode\r\n",true);
			RandomMode();
		}

        else if (!strcmp(input, "exit")) {
            /* Exit the console task */
            DbgUartPutLine("Are you sure you want to exit the console? Y/N: \r\n",true);
            Task_sleep(10);
            do{
	    		DbgUartGetLine(input);
	    	}while(input[0] == '\0');

            if ((input[0] == 'y' || input[0] == 'Y') && input[1] == 0x00) {
                DbgUartPutLine("Exiting console, goodbye.\r\n",true);
                exit_flag = 1;
                break;
            }
        }

        else {
            /* Print a list of valid commands. */
            DbgUartPutLine("Valid commands:\r\n",true);
            DbgUartPutLine("- pid: start pid test.\r\n",true);
            DbgUartPutLine("- actionlr: start ACTION LR turn test.\r\n",true);
            DbgUartPutLine("- actionir: start ACTION corr by ir.\r\n",true);
            DbgUartPutLine("- random: start random mode.\r\n",true);
            DbgUartPutLine("- exit: Exit the console task.\r\n",true);
        }
		input[0] = '\0';
		if(exit_flag)
		    break;
		Task_sleep(10000);
	}
}

void pidTest(void)
{
	int i=0;
	TskTop::SetLeds(0x7);

	TskMotor::MotorMsg::MsgType motMsg;

	motMsg = TskMotor::MotorMsg::DisableMotors;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	motMsg = TskMotor::MotorMsg::EnableAcqZeros;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);

	Task_sleep(1000);// wait for getting imu zeros & getting ir zeros
	TskTop::SetLeds(0x0);

//	motMsg = TskMotor::MotorMsg::DisableAcqZeros;
//	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	motMsg = TskMotor::MotorMsg::EnableAccl;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	motMsg = TskMotor::MotorMsg::EnableMotors;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	motMsg = TskMotor::MotorMsg::EnableGyro;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);

	Task_sleep(100);// wait for all HW Enable

	TskMotor::DistanceAcc = 0.f;
	TskMotor::AngleAcc = 0.f;
	TskMotor::DistanceAcc_en = 0.f;

	DbgUartPutLine("triggle IR L for V pid, triggle IR R for W PID\r\n", true);

    while(true){
#if 1
    	while(1){
    	    TskTop::SetLeds(0x1);
        	if(TskIr::TestIrTouch(TskIr::IrCh::FL, 1600, 1200))
        	{
        	    TskTop::SetLeds(0x0);
        		TskTop::info_flag = 1;
        		DbgUartPutLine("start Velocity pid test\n", true);
            	break;
        	}

        	else if(TskIr::TestIrTouch(TskIr::IrCh::FR, 1600, 1200))
        	{
        	    TskTop::SetLeds(0x0);
        		TskTop::info_flag = 2;
        		DbgUartPutLine("start Omega pid test\n", true);
            	break;
        	}
    	}
        Task_sleep(500);
        TskTop::info_flag = 0;
#endif
        while(i < 512)
        {
            sprintf(dbgStr, "%4.3f,%4.3f\n",
                TskAction::Desire[i],
                TskAction::Info[i]);
            DbgUartPutLine(dbgStr, true);
            Task_sleep(10);
            i++;
        }
        if(i == 512) {
        	Task_sleep(100);
        	DbgUartPutLine("over\r\n", true);
        	i = 0;
        	sprintf(dbgStr, "%8.3f,%8.3f,%8.3f\r\n",
                  TskMotor::DistanceAcc,
                  TskMotor::DistanceAcc_en,
                  TskMotor::AngleAcc);
        	DbgUartPutLine(dbgStr, true);
        }

        motMsg = TskMotor::MotorMsg::DisableMotors;
		Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);

        break;
    }
}

void ActionTestLR(void)
{
	float pre=0.f,post=0.f;
	char str[10];
	TskTop::SetLeds(0x7);
	TskMotor::MotorMsg::MsgType motMsg;
	TskAction::Act::ActType actMsg;
	TskAction::ActMsg::MsgType end_Msg;

	motMsg = TskMotor::MotorMsg::DisableMotors;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	motMsg = TskMotor::MotorMsg::EnableAcqZeros;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);

	Task_sleep(1000);// wait for getting imu zeros & getting ir zeros
	TskTop::SetLeds(0x0);
//	motMsg = TskMotor::MotorMsg::DisableAcqZeros;
//	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);

	motMsg = TskMotor::MotorMsg::EnableAccl;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	motMsg = TskMotor::MotorMsg::EnableMotors;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	motMsg = TskMotor::MotorMsg::EnableGyro;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);

	Task_sleep(100);// wait for all HW Enable

	TskMotor::DistanceAcc = 0.f;
	TskMotor::AngleAcc = 0.f;
	TskMotor::DistanceAcc_en = 0.f;

    DbgUartPutLine("please input l, r to test LR Turn90\r\n",true);

	while(true)
	{
		do{
    		DbgUartGetLine(str);
    	}while(str[0] == '\0');

		switch(str[0]){
			case 'l':
				DbgUartPutLine("test L Turn90\r\n", true);
				DbgUartPutLine("input L Pre \r\n", true);

		   		do{
		    		DbgUartGetLine(str);
		    	}while(str[0] == '\0');
   				pre = atof(str);

				DbgUartPutLine("input L Post\r\n", true);
		   		do{
		    		DbgUartGetLine(str);
		    	}while(str[0] == '\0');
   				post = atof(str);

   				CP.TURNL90_PRE_ADJ += pre;
   				CP.TURNL90_POST_ADJ += post;
   				sprintf(dbgStr, "L PRE:%5.3f, L POST:%5.3f\r\n",
                  	CP.TURNL90_PRE_ADJ,
                  	CP.TURNL90_POST_ADJ);
        		DbgUartPutLine(dbgStr, true);

        		actMsg = (TskAction::Act::ActType)(TskAction::Act::Start);
		   		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
	       		Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
	       		actMsg = (TskAction::Act::ActType)(TskAction::Act::L90);
		   		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
	       		Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
	       		actMsg = (TskAction::Act::ActType)(TskAction::Act::Stop);
		   		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
	       		Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
				break;
			case 'r':
				DbgUartPutLine("test R Turn90\r\n", true);

				DbgUartPutLine("input R Pre\r\n", true);

		   		do{
		    		DbgUartGetLine(str);
		    	}while(str[0] == '\0');
   				pre = atof(str);

				DbgUartPutLine("input R Post\r\n", true);
		   		do{
		    		DbgUartGetLine(str);
		    	}while(str[0] == '\0');
   				post = atof(str);

   				CP.TURNR90_PRE_ADJ += pre;
   				CP.TURNR90_POST_ADJ += post;
   				sprintf(dbgStr, "R PRE:%5.3f, R POST:%5.3f\r\n",
                  	CP.TURNR90_PRE_ADJ,
                  	CP.TURNR90_POST_ADJ);
        		DbgUartPutLine(dbgStr, true);

        		actMsg = (TskAction::Act::ActType)(TskAction::Act::Start);
		   		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
	       		Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
	       		actMsg = (TskAction::Act::ActType)(TskAction::Act::L90);
		   		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
	       		Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
	       		actMsg = (TskAction::Act::ActType)(TskAction::Act::Stop);
		   		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
	       		Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
				break;
			default:
				DbgUartPutLine("Please input l or r\r\n", true);
				Task_sleep(5000);
				break;
		}
    	sprintf(dbgStr, "%6.3f,%6.3f,%6.3f\r\n",
      		TskMotor::DistanceAcc,
      		TskMotor::DistanceAcc_en,
      		TskMotor::AngleAcc);
    	DbgUartPutLine(dbgStr, true);
		fflush(stdout);
    	break;	
	}
}

/*
 *	FWD wall.left or wall.right HEADING_BY_SIRSIDE_START_DIST
 *								LFWDEND_DIST_W2NW
 *								RFWDEND_DIST_W2NW
 *  	centipede				HEADING_BY_SIRFWD_BGNSTAT_POS
 *								HEADING_BY_SIRFWD_BEGIN_POS
 *
 *	L90							TURNLWAIT_DIST_ADJ
 *
 *	R90							TURNRWAIT_DIST_ADJ
 *
 *  Restart						RESTART_DIST_ADJ
 *
 *  Stop						STOPEND_DIST_ADJ
 *
 *	Back						FWDDISADJ
 *								LRBACKANGLE_ADJ
 *								FLRYAWERROR					
 *        						LBACKCENTER_ADJ
 *								RBACKCENTER_ADJ
 */
void ActionIr(void)
{
	float adj1=0.f,adj2=0.f,adj3=0.f,adj4=0.f,adj5=0.f;
	char str[10];
	TskTop::SetLeds(0x7);
	TskMotor::MotorMsg::MsgType motMsg;
	TskAction::Act::ActType actMsg;
	TskAction::ActMsg::MsgType end_Msg;

	motMsg = TskMotor::MotorMsg::DisableMotors;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	motMsg = TskMotor::MotorMsg::EnableAcqZeros;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);

	Task_sleep(1000);// wait for getting imu zeros & getting ir zeros
	TskTop::SetLeds(0x0);
//	motMsg = TskMotor::MotorMsg::DisableAcqZeros;
//	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);

	motMsg = TskMotor::MotorMsg::EnableAccl;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	motMsg = TskMotor::MotorMsg::EnableMotors;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	motMsg = TskMotor::MotorMsg::EnableGyro;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);

	Task_sleep(100);// wait for all HW Enable

	TskMotor::DistanceAcc = 0.f;
	TskMotor::AngleAcc = 0.f;
	TskMotor::DistanceAcc_en = 0.f;

    DbgUartPutLine("please input L90, R90, FWD, RESTART, STOP，BACK to choose action\r\n",true);

	while(true)
	{
		DbgUartPutLine("input Action command\r\n", true);

   		do{
    		DbgUartGetLine(str);
    	}while(str[0] == '\0');

		if(!strcmp(str, "L90"))
		{
			DbgUartPutLine("test L Turn90 Corr, input adj to adjust LTurnWaitDist\r\n", true);
			do{
				DbgUartGetLine(str);
			}while(str[0] == '\0');
			adj1 = atof(str);
			CP.TURNLWAIT_DIST_ADJ += adj1;
			sprintf(dbgStr, "L ADJ DIST:%5.3f\r\n", CP.TURNLWAIT_DIST_ADJ);
    		DbgUartPutLine(dbgStr, true);
    		fflush(stdout);
    		actMsg = (TskAction::Act::ActType)(TskAction::Act::Start);
	   		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
       		Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
       		actMsg = (TskAction::Act::ActType)(TskAction::Act::L90 | TskAction::Act::Corr);
	   		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
       		Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
       		actMsg = (TskAction::Act::ActType)(TskAction::Act::Stop);
	   		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
       		Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
			break;
		}

		else if(!strcmp(str, "R90"))
		{
			DbgUartPutLine("test R Turn90 Corr, input adj to adjust RTurnWaitDist\r\n", true);
			do{
				DbgUartGetLine(str);
			}while(str[0] == '\0');
			adj1 = atof(str);
			CP.TURNRWAIT_DIST_ADJ += adj1;
			sprintf(dbgStr, "R ADJ DIST:%5.3f\r\n", CP.TURNRWAIT_DIST_ADJ);
    		DbgUartPutLine(dbgStr, true);
    		fflush(stdout);
    		actMsg = (TskAction::Act::ActType)(TskAction::Act::Start);
	   		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
       		Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
       		actMsg = (TskAction::Act::ActType)(TskAction::Act::R90 | TskAction::Act::Corr);
	   		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
       		Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
       		actMsg = (TskAction::Act::ActType)(TskAction::Act::Stop);
	   		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
       		Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
			break;
		}

		else if(!strcmp(str, "RESTART"))
		{
			DbgUartPutLine("test RESTART Corr, input adj to adjust RestartDist\r\n", true);
			do{
				DbgUartGetLine(str);
			}while(str[0] == '\0');
			adj1 = atof(str);
			CP.RESTART_DIST_ADJ += adj1;
			sprintf(dbgStr, "R ADJ DIST:%5.3f\r\n", CP.RESTART_DIST_ADJ);
    		DbgUartPutLine(dbgStr, true);
    		fflush(stdout);
    		actMsg = (TskAction::Act::ActType)(TskAction::Act::Start);
	   		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
       		Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
       		actMsg = (TskAction::Act::ActType)(TskAction::Act::Stop);
	   		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
       		Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
       		actMsg = (TskAction::Act::ActType)(TskAction::Act::Back);
	   		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
       		Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
       		actMsg = (TskAction::Act::ActType)(TskAction::Act::Restart | TskAction::Act::Corr);
	   		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
       		Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
       		actMsg = (TskAction::Act::ActType)(TskAction::Act::Stop);
	   		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
       		Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
			break;
		}

		else if(!strcmp(str, "STOP"))
		{
			DbgUartPutLine("test stop Corr, input adj to adjust StopEndDist\r\n", true);
			do{
				DbgUartGetLine(str);
			}while(str[0] == '\0');
			adj1 = atof(str);
			CP.STOPEND_DIST_ADJ += adj1;
			sprintf(dbgStr, "R ADJ DIST:%5.3f\r\n", CP.STOPEND_DIST_ADJ);
    		DbgUartPutLine(dbgStr, true);
    		fflush(stdout);
    		actMsg = (TskAction::Act::ActType)(TskAction::Act::Start);
	   		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
       		Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
       		actMsg = (TskAction::Act::ActType)(TskAction::Act::Stop | TskAction::Act::Corr);
	   		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
       		Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
			break;
		}

		else if(!strcmp(str, "FWD"))
		{
			DbgUartPutLine("test fwd Corr\r\n", true);
			DbgUartPutLine("please input StartDist change value\r\n", true);
			do{
				DbgUartGetLine(str);
			}while(str[0] == '\0');
			adj1 = atof(str);
			DbgUartPutLine("please input BgnstatPos change value\r\n", true);
			do{
				DbgUartGetLine(str);
			}while(str[0] == '\0');
			adj2 = atof(str);
			DbgUartPutLine("please input BeginPos change value\r\n", true);
			do{
				DbgUartGetLine(str);
			}while(str[0] == '\0');
			adj3 = atof(str);
			DbgUartPutLine("please input LFwdEndDist change value\r\n", true);
			do{
				DbgUartGetLine(str);
			}while(str[0] == '\0');
			adj4 = atof(str);
			DbgUartPutLine("please input RFwdEndDist change value\r\n", true);
			do{
				DbgUartGetLine(str);
			}while(str[0] == '\0');
			adj5 = atof(str);

 			CP.HEADING_BY_SIRSIDE_START_DIST += adj1;
         	CP.HEADING_BY_SIRFWD_BGNSTAT_POS += adj2;
        	CP.HEADING_BY_SIRFWD_BEGIN_POS   += adj3;
         	CP.LFWDEND_DIST_W2NW += adj4;
         	CP.RFWDEND_DIST_W2NW += adj5;

			sprintf(dbgStr, "Params as:%5.3f %5.3f %5.3f %5.3f %5.3f\r\n", 
				CP.HEADING_BY_SIRSIDE_START_DIST,
         		CP.HEADING_BY_SIRFWD_BGNSTAT_POS,
        		CP.HEADING_BY_SIRFWD_BEGIN_POS,
         		CP.LFWDEND_DIST_W2NW,
         		CP.RFWDEND_DIST_W2NW);
    		DbgUartPutLine(dbgStr, true);

    		actMsg = (TskAction::Act::ActType)(TskAction::Act::Start);
	   		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
       		Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
       		actMsg = (TskAction::Act::ActType)(TskAction::Act::Fwd | TskAction::Act::Corr);
	   		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
       		Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
       		actMsg = (TskAction::Act::ActType)(TskAction::Act::Stop);
	   		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
       		Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
			break;
		}

		else if(!strcmp(str, "BACK"))
		{
			DbgUartPutLine("test back Corr\r\n", true);
			DbgUartPutLine("please input FwdDist change value\r\n", true);
			do{
				DbgUartGetLine(str);
			}while(str[0] == '\0');
			adj1 = atof(str);
			DbgUartPutLine("please input LRBackAngle change value\r\n", true);
			do{
				DbgUartGetLine(str);
			}while(str[0] == '\0');
			adj2 = atof(str);
			DbgUartPutLine("please input FLRYawError change value\r\n", true);
			do{
				DbgUartGetLine(str);
			}while(str[0] == '\0');
			adj3 = atof(str);
			DbgUartPutLine("please input LBackCenter change value\r\n", true);
			do{
				DbgUartGetLine(str);
			}while(str[0] == '\0');
			adj4 = atof(str);
			DbgUartPutLine("please input RBackCenter change value\r\n", true);
			do{
				DbgUartGetLine(str);
			}while(str[0] == '\0');
			adj5 = atof(str);

			CP.FWDDISADJ 	   += adj1;
 			CP.LRBACKANGLE_ADJ += adj2;
         	CP.FLRYAWERROR 	   += adj3;
        	CP.LBACKCENTER_ADJ += adj4;
         	CP.RBACKCENTER_ADJ += adj5;

			sprintf(dbgStr, "Params as:%5.3f %5.3f %5.3f %5.3f %5.3f\r\n", 
				CP.FWDDISADJ,
				CP.LRBACKANGLE_ADJ,
         		CP.FLRYAWERROR,
        		CP.LBACKCENTER_ADJ,
         		CP.RBACKCENTER_ADJ);
    		DbgUartPutLine(dbgStr, true);

    		actMsg = (TskAction::Act::ActType)(TskAction::Act::Start);
	   		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
       		Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
       		actMsg = (TskAction::Act::ActType)(TskAction::Act::Stop);
	   		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
       		Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
       		actMsg = (TskAction::Act::ActType)(TskAction::Act::Back);
	   		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
       		Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
			break;
		}

    	sprintf(dbgStr, "%6.3f,%6.3f,%6.3f\r\n",
      		TskMotor::DistanceAcc,
      		TskMotor::DistanceAcc_en,
      		TskMotor::AngleAcc);
    	DbgUartPutLine(dbgStr, true);
    	break;	
	}
}

void RandomMode(void){
	int randint = 0,i = 100;

	TskTop::SetLeds(0x7);

	TskMotor::MotorMsg::MsgType motMsg;
	TskAction::Act::ActType actMsg;
	TskAction::ActMsg::MsgType end_Msg;

	motMsg = TskMotor::MotorMsg::DisableMotors;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	motMsg = TskMotor::MotorMsg::EnableAcqZeros;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);

	Task_sleep(1000);// wait for getting imu zeros & getting ir zeros
	TskTop::SetLeds(0x0);

//	motMsg = TskMotor::MotorMsg::DisableAcqZeros;
//	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	motMsg = TskMotor::MotorMsg::EnableAccl;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	motMsg = TskMotor::MotorMsg::EnableMotors;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);
	motMsg = TskMotor::MotorMsg::EnableGyro;
	Mailbox_post(TskMotor::MbCmd, &motMsg, BIOS_NO_WAIT);

	Task_sleep(100);// wait for all HW Enable

	TskMotor::DistanceAcc = 0.f;
	TskMotor::AngleAcc = 0.f;
	TskMotor::DistanceAcc_en = 0.f;

	DbgUartPutLine("Start Random Mode\r\n", true);
    fflush(stdout);

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
    	TskTop::actPrint(actMsg,dbgStr);
		DbgUartPutLine(dbgStr, true);
		Mailbox_post(TskAction::MbCmd, &actMsg, BIOS_NO_WAIT);
		Mailbox_pend(TskTop::MbCmd, &end_Msg, BIOS_WAIT_FOREVER);
		i += 100;
    }
}
