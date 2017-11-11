/*
 * actstatus.cpp
 *
 *  Created on: 2017Äê11ÔÂ9ÈÕ
 *      Author: db_Hu
 */
#include "actstatus.h"

#if 0
    void Turn2RushAct(Queue<Turning::Type> *q)
    {
        int status = 0;
        int p = 0;
        Turning::Type turn;
        while(q->Length())
        {
            turn = q->Dequeue();
            switch(status)
            {
            case 0: //Entry point
                if(turn == Turning::Forward)
                    status = 1;
                break;
            case 1:
                if(turn == Turning::Forward);
//                  solve::QAct->En(TskAction::Act::SRush);
                else if(turn == Turning::Right)
                    status = 2;
                else if(turn == Turning::Left)
                    status = 3;
                break;
            case 2:
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
            case 3:
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
            case 4:
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
            case 5:
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
            case 6:
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
            case 7:
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
            case 8:
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
            case 9:
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
            }
            p++;
        }
    }
#endif
