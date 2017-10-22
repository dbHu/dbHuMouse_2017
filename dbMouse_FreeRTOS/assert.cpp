/*
 * assert.cpp
 *
 *  Created on: 2017��10��22��
 *      Author: db_Hu
 */
#include <stdio.h>
#include "TskTop/DbgUart.h"
char dbgStr[512];
void vApplicationAssert(const char *file, unsigned int line)
{
    sprintf(dbgStr, "assert fail in %s, line %d", file, line);
    TskPrint::UartWrite(dbgStr);
    while(1);
}



