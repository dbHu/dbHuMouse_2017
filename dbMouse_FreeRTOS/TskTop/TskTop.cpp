/*
 * top.cc
 *
 *  Created on: Jul 21, 2016
 *      Author: loywong
 */
#include "TskTop.h"
#include "TskTop/DbgUart.h"
#include "TskMotor/TskMotor.h"
#include "PinConfig/pinout.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

namespace TskTop{
const int tskPrio = 4;
const int tskStkSize = 2048;
char dbgStr[128];

void SetLeds(unsigned char val)
{
    GPIO_write(DBMOUSE_LED_0, (val & 0x1) ? DBMOUSE_LED_ON : DBMOUSE_LED_OFF);
    GPIO_write(DBMOUSE_LED_1, (val & 0x2) ? DBMOUSE_LED_ON : DBMOUSE_LED_OFF);
    GPIO_write(DBMOUSE_LED_2, (val & 0x4) ? DBMOUSE_LED_ON : DBMOUSE_LED_OFF);
}

void task(void *pvParameters)
{
    BaseType_t rtn;
    size_t hpsize;

    hpsize = xPortGetFreeHeapSize();
	vTaskDelay(1000);

	SetLeds(0x01);
    vTaskDelay(100);

    hpsize = xPortGetFreeHeapSize();
    SetLeds(0x02);
    vTaskDelay(100);
    SetLeds(0x04);
    vTaskDelay(100);
    SetLeds(0x00);

    vTaskDelay(1000);

    hpsize = xPortGetFreeHeapSize();
    TskPrint::Init();
    hpsize = xPortGetFreeHeapSize();
    TskMotor::Init();

    while(1)
    {
        TskPrint::UartGetLine(dbgStr);
    //  sprintf(dbgStr, "Hello from dbmouse!\n");
        rtn=xQueueSend(TskPrint::MbCmd, dbgStr, portMAX_DELAY);
        configASSERT(rtn==pdPASS);
        vTaskDelay(50);
    }
}

void Init()
{
	BaseType_t rtn;

    // MbCmd = xQueueCreate(4, sizeof(TskAction::ActMsg::MsgType));
	// configASSERT(MbCmd);
    // Create tasks
    rtn=xTaskCreate(task, (const portCHAR *)"TopTask",
                tskStkSize, NULL, tskPrio, NULL);
    configASSERT(rtn==pdPASS);
}

}

