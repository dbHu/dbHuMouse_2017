/*
 * Imu.cc
 *
 *  Created on: Aug 3, 2016
 *      Author: loywong
 */

#include "../Board.h"

#include <xdc/runtime/System.h>
#include <xdc/runtime/Assert.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>

#include <ti/drivers/I2C.h>

#include "Imu.h"

const unsigned char Imu20608Addr = 0x69;

I2C_Handle iicImu;
I2C_Params iicImuParams;
I2C_Transaction iicTrans;

Semaphore_Params semParams;
Semaphore_Handle semIicFinish;

#pragma pack(push)
#pragma pack(1)
union _imuData
{
    unsigned char buf[14];
	//char buf[14];
    struct
    {
    	unsigned char axh, axl, ayh, ayl, azh, azl;
    	unsigned char th, tl;
    	unsigned char gxh, gxl, gyh, gyl, gzh, gzl;
    };
} imuData;
#pragma pack(pop)

void iicImuCallback(I2C_Handle iic, I2C_Transaction *trans, bool succes)
{
    Semaphore_post(semIicFinish);
}

unsigned char imuInitSeq0[] = {
        0x19,
        0x00, // 0x19: sr = gyro rate / 1
        0x01, // 0x1A: ext sync disable, dlpf = 1(accl 1k, gyro 1k)
        0x18, // 0x1B: gyro fs = 2k deg/sec (34.6/s)
        0x00  // 0x1C: accl fs = 2g (19.6m/sq.s)
};
unsigned char imuInitSeq1[] = {
        0x6B,
        0x00    // 0x6B: no sleep
};
unsigned char imuReadSeq[] = {
        0x3B  // 0x3B: start of data
};

void ImuStartRead()
{
    I2C_transfer(iicImu, &iicTrans);
}

bool ImuGetValues(ImuValues &imuVal)
{
    bool rtn;
    rtn=Semaphore_pend(semIicFinish, 2);
    Assert_isTrue(rtn,NULL);

    imuVal.acclX = -AcclUnit * (float)(short)((imuData.axh << 8) | imuData.axl);
    imuVal.acclY = AcclUnit * (float)(short)((imuData.ayh << 8) | imuData.ayl);
    imuVal.acclZ = AcclUnit * (float)(short)((imuData.azh << 8) | imuData.azl);

    imuVal.angvX = GyroUnit * (float)(short)((imuData.gxh << 8) | imuData.gxl);
    imuVal.angvY = GyroUnit * (float)(short)((imuData.gyh << 8) | imuData.gyl);
    // TODO : solve this
    imuVal.angvZ = -GyroUnit * (float)(short)((imuData.gzh << 8) | imuData.gzl);

    imuVal.temp = (float)(short)((imuData.th << 8) | imuData.tl) * 2.941176471e-3f + 36.53f;

    return true;
}

void ImuInit()
{
    bool rtn;
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    semIicFinish = Semaphore_create(0, &semParams, NULL);
    if(semIicFinish == NULL)
        System_abort("semIicFinish creation failed.\n");

    /* Create I2C for usage */
    I2C_Params_init(&iicImuParams);

    iicImuParams.transferMode = I2C_MODE_CALLBACK;
    iicImuParams.transferCallbackFxn = iicImuCallback;
    iicImuParams.bitRate = I2C_400kHz;
    iicImu = I2C_open(Board_I2C_IMU, &iicImuParams);
    if (iicImu == NULL)
        System_abort("Error Initializing IicImu!\n");

    iicTrans.slaveAddress = Imu20608Addr;
    iicTrans.writeBuf = imuInitSeq1;
    iicTrans.writeCount = 2;
    iicTrans.readBuf = NULL;
    iicTrans.readCount = 0;

    I2C_transfer(iicImu, &iicTrans);
    rtn=Semaphore_pend(semIicFinish, 2);
    Assert_isTrue(rtn,NULL);

    iicTrans.writeBuf = imuInitSeq0;
    iicTrans.writeCount = 5;

    I2C_transfer(iicImu, &iicTrans);
    rtn=Semaphore_pend(semIicFinish, 2);
    Assert_isTrue(rtn,NULL);


    ////////////////////////////////
//    unsigned char wb[1] = {0x6B};
//    unsigned char rb[1] = {0x00};
//    iicTrans.slaveAddress = Mpu6050Addr;
//    iicTrans.writeBuf = wb;
//    iicTrans.writeCount = 1;
//    iicTrans.readBuf = rb;
//    iicTrans.readCount = 1;
//    I2C_transfer(iicImu, &iicTrans);
//    if(!Semaphore_pend(semIicFinish, 2))
//        System_abort("Pend semIicFinish Failed!\n");
    ////////////////////////////////

    iicTrans.slaveAddress = Imu20608Addr;
    iicTrans.writeBuf = imuReadSeq;
    iicTrans.writeCount = 1;
    iicTrans.readBuf = imuData.buf;
    iicTrans.readCount = 14;
}
//---------------------------------------------------

