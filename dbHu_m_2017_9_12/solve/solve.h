/*
 * solve.h
 *
 *  Created on: 2016Äê9ÔÂ1ÈÕ
 *      Author: db_Hu
 */

#ifndef SOLVE_SOLVE_H_
#define SOLVE_SOLVE_H_

#include "../Queue/Queue.h"

namespace solve
{
class Solve
{
public: enum SolveType
    {
        Null         = 0x01000000,
		ALGOTEST     = 0x02000000,
		RUSHTEST     = 0x03000000,
		Finish		 = 0x00000001,
    };
};
extern Mailbox_Handle MbAct,MbTop;
extern Queue<TskAction::Act::ActType> *QAct;
void Init();
}




#endif /* SOLVE_SOLVE_H_ */
