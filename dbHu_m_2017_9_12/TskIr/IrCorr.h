/*
 * IrCorr.h
 *
 *  Created on: Aug 15, 2016
 *      Author: loywong
 */

#ifndef TSKIR_IRCORR_H_
#define TSKIR_IRCORR_H_

namespace TskIr
{

void eraseFlashBlock(int blkIdx);
void programFlash(unsigned int addr, unsigned int *data, int wordLen);
void doIrCorrection();
void ReadFlash(unsigned int addr, unsigned char *data, int byteLen);

}

#endif /* TSKIR_IRCORR_H_ */
