/*
 * actstatus.cpp
 *
 *  Created on: 2017Äê11ÔÂ9ÈÕ
 *      Author: db_Hu
 */
#include "actstatus.h"

#if 0
    void sendNFwd(int p, bool type)
    {
        switch(type){
            case 'D':
                if(p == 1)
                {
                    solve::QAct->En(TskAction::Act::DRush);
                }
                else{
                    for(int i = 0; i < p - 1; i++)
                    {
                        solve::QAct->En(TskAction::Act::DRushAcc);
                    }
                    solve::QAct->En(TskAction::Act::DRushDea);
                }
                break;
            case 'O':
            default:
                if(p == 1)
                {
                    solve::QAct->En(TskAction::Act::ORush);
                }
                else{
                    for(int i = 0; i < p - 1; i++)
                    {
                        solve::QAct->En(TskAction::Act::ORushAcc);
                    }
                    solve::QAct->En(TskAction::Act::ORushDea);
                }
                break;
        }

    }
    
    void Turn2RushAct(Queue<Turning::Type> *q)
    {
        int status = 0;
        int p = 0;

        Turning::Type turn;
        while(status != 11)
        {
            turn = q->Dequeue();
            switch(status)
            {
            case 0:         //Entry point
                solve::QAct->En(TskAction::Act::RushStart);
                if(turn == Turning::Forward)
                    p = 1;
                break;
            case 1:        //ORTHO 
                if(turn == Turning::Forward);
//                  p++;
                else if(turn == Turning::Right){
                    sendNFwd(p);
                    p = 0;
                    status = 2;
                }
                else if(turn == Turning::Left)
                    status = 3;
                else if(turn == Turning::Backward)
                    status = 10;
                break;
            case 2:       //ORTHO_R
                if(turn == Turning::Left)
                {
                    solve::QAct->En(TskAction::Act::R45i);
                    status = 4;
                }
                else if(turn == Turning::Right)
                {
                    status = 7;
                }
                else if(turn == Turning::Forward)
                {
//                  solve::QAct->En(TskAction::Act::R90r);
                    status = 1;
                }
                break;
            case 3:       //ORTHO_L
                if(turn == Turning::Right)
                {
                    solve::QAct->En(TskAction::Act::L45i);
                    status = 5;
                }
                else if(turn == Turning::Left)
                {
                    status = 6;
                }
                else if(turn == Turning::Forward)
                {
//                  solve::QAct->En(TskAction::Act::L90r);
                    status = 1;
                }
                break;
            case 4:       //ORTHO_RR
                if(turn == Turning::Right)
                {
//                  solve::QAct->En(TskAction::Act::TRush);
                    status = 5;
                }
                else if(turn == Turning::Forward)
                {
                    solve::QAct->En(TskAction::Act::L45o);
                    status = 1;
                }
                else if(turn == Turning::Left)
                {
                    status = 8;
                }
                break;
            case 5:       //DIAG_RL
                if(turn == Turning::Left)
                {
//                  solve::QAct->En(TskAction::Act::TRush);
                    status = 4;
                }
                else if(turn == Turning::Forward)
                {
                    solve::QAct->En(TskAction::Act::R45o);
                    status = 1;
                }
                else if(turn == Turning::Right)
                {
                    status = 9;
                }
                break;
            case 6:       //ORTHO_LL
                if(turn == Turning::Forward)
                {
                    solve::QAct->En(TskAction::Act::L180);
                    status = 1;
                }
                else if(turn == Turning::Right)
                {
                    solve::QAct->En(TskAction::Act::L135i);
                    status = 5;
                }
                break;
            case 7:       //DIAG_LR
                if(turn == Turning::Forward)
                {
                    solve::QAct->En(TskAction::Act::L180);
                    status = 1;
                }
                else if(turn == Turning::Left)
                {
                    solve::QAct->En(TskAction::Act::R135i);
                    status = 4;
                }
                break;
            case 8:       //DIAG_LL
                if(turn == Turning::Right)
                {
//                  solve::QAct->En(TskAction::Act::L90t);
                    status = 9;
                }
                else if(turn == Turning::Forward)
                {
                    solve::QAct->En(TskAction::Act::L135o);
                    status = 1;
                }
                break;
            case 9:       //DIAG_RR
                if(turn == Turning::Left)
                {
//                  solve::QAct->En(TskAction::Act::R90t);
                    status = 8;
                }
                else if(turn == Turning::Forward)
                {
                    solve::QAct->En(TskAction::Act::L135o);
                    status = 1;
                }
                break;
            case 10:       //STOP
                solve::QAct->En(TskAction::Act::L135o);
                status = 11;
                break;
            default:
                configAssert(false);
                status = 11;
                break;
        }
    }
#endif
