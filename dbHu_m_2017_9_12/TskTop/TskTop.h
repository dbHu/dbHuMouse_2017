/*
 * top.h
 *
 *  Created on: Jul 21, 2016
 *      Author: loywong
 */

#ifndef TSKTOP_TSKTOP_H_
#define TSKTOP_TSKTOP_H_


#include <ti/sysbios/knl/Mailbox.h>
#include "action/action.h"
namespace TskTop
{

class MouseMode
{
public:
    enum ModeType
    {
    	Idle = 0,
		EncImuMonitor,
        IrMonitor,
        IrCorrection,
        UartCmd,
        ActionTest,
		SolveTest,
        ClearMaze,
        Gaming1,
        Undef1,
        Undef2,
        Undef3,
        Gaming4,
        Gaming3,
        Gaming2
    };
};

extern Mailbox_Handle MbCmd;
extern volatile MouseMode::ModeType Mode;
extern char dbgStr[128];
void Init();
void SetLeds(unsigned char val);
void actPrint(TskAction::Act::ActType act);

}



#endif /* TSKTOP_TSKTOP_H_ */
