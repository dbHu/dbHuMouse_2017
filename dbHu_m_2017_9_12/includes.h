/*
 * include.h
 *
 *  Created on: 2016��8��27��
 *      Author: db_Hu
 */

#ifndef INCLUDES_H_
#define INCLUDES_H_

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Timestamp.h>
#include <stdio.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/GPIO.h>

#include <inc/hw_sysctl.h>
#include <inc/hw_pwm.h>
#include <inc/hw_qei.h>
#include <inc/hw_gpio.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_ints.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <dbmouse_chassis.h>

#include "../TskMotor/TskMotor.h"
#include "../TskIr/TskIr.h"
#include "../TskIr/IrCorr.h"
#include "action/action.h"
#include "solve/solve.h"
#include "TskMotor/TskMotor.h"
#include "physparams.h"
#include "TskTop/DbgUart.h"
#include "TskTop/TskTop.h"
#include "Pid/pid.h"
#include "TskMotor/WheelEnc.h"
#include "mmaze/mmaze.h"
#include "mouse/mouse.h"
#include "solve/solve.h"
#include "../Queue/Queue.h"
#endif /* INCLUDES_H_ */
