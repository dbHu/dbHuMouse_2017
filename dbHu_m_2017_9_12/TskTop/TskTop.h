/*
 * top.h
 *
 *  Created on: Jul 21, 2016
 *      Author: loywong
 */

#ifndef TSKTOP_TSKTOP_H_
#define TSKTOP_TSKTOP_H_


#include <ti/sysbios/knl/Mailbox.h>
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
        MotionCorrection,
        ListenUartCmd,
		ActionTest,
		SolveTest,
        Undef1,
        Undef2,
        Undef3,
		ClearMaze,
        Gaming4,
        Gaming3,
        Gaming2,
        Gaming1
    };
};

extern Mailbox_Handle MbCmd;
extern volatile MouseMode::ModeType Mode;
extern char dbgStr[80];
void Init();
void SetLeds(unsigned char val);

}



#endif /* TSKTOP_TSKTOP_H_ */
