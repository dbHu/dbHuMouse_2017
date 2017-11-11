/*
 * paramsCorr.h
 *
 *  Created on: 2017Äê11ÔÂ7ÈÕ
 *      Author: db_Hu
 */

#ifndef TSKTOP_PARAMSCORR_H_
#define TSKTOP_PARAMSCORR_H_

#include "TskMotor/TskMotor.h"
#include "Action/Action.h"
struct VarAndName
{
    float * value;
    char * name;
};

struct ActAndName
{
    TskAction::Act::ActType action;
    char * name;
};

const int varnum = 6 + 3 + 14 + 50;

const int actnum = 8 + 22;

extern VarAndName varname[varnum];
extern ActAndName actname[actnum];

#endif /* TSKTOP_PARAMSCORR_H_ */
