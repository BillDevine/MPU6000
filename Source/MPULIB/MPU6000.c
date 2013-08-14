/*
 * MPU6000.c
 *
 *This library is intended to simplify configuration and control of the MPU6000
 *so that users do not have to deal with a near incomprehensible register map.
 *
 *Bill Devine
 *2013
 */

#include <msp430f5342.h>
#include "MPU6000.h"





static int MPU6000ID;
static MPU6KDEV thisMPU={0x08, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD8, 0x01, 0x00,  0x00, 0x00, 0x00};
unsigned char rxdat[512];


/*****************************//**
 * @brief
 *
 *********************************/
int MPU6000Init(void){
	MPU6000ID=setupI2CDevice(0x68, 25, &rxdat[0]);
	changeI2Cdevice(MPU6000ID);
	MPUConfig();
	return MPU6000ID;
}

void MPUConfig(void){
	MPU6000Wake();
	while(getPortStatus() != statOpen);

	unsigned char dat[5]={SMPLRT_DIV, thisMPU->smplrt_div, thisMPU->config, thisMPU->gyro_config, thisMPU->accel_config};
	I2CWrite(0x05, &dat[0]);
	while(getPortStatus != statOpen);

	unsigned char dat1[2]={MOT_THR, thisMPU->mot_thr};
	I2CWrite(0x02, &dat1[0]);
	while(getPortStatus() != statOpen);

	unsigned char dat2[3]={FIFO_EN, thisMPU->fifo_en, thisMPU->i2c_mst_ctrl};
	I2CWrite(0x03, &dat2[0]);
	while(getPortStatus() != statOpen);

	unsigned char dat3[3]={INT_PIN_CFG, thisMPU->int_pin_cfg, thisMPU->int_enable};
	I2CWrite(0x03, &dat3[0]);
	while(getPortStatus() != statOpen);

	unsigned char dat4[3]={MOT_DETECT_CTRL, thisMPU->mot_detect_ctrl, thisMPU->user_ctrl};
	I2CWrite(0x03, &dat4[0]);
	while(getPortStatus() != statOpen);

	unsigned char dat5[2]={PWR_MGMT_2, thisMPU->pwr_mgmt_2};
	I2CWrite(0x02, &dat5[0]);
	while(getPortStatus() != statOpen);
}

int MPU6000SampleRateConfig(unsigned int sampleRate){
	if(sampleRate>1000){
		return -1;
	}else{
		unsigned char sampleRateDivisor;
		(sampleRateDivisor)=(char)(8000/sampleRate);
		if((8000 % sampleRate) < (sampleRate / 2)){
			sampleRateDivisor--;
		}else;
		thisMPU->smplrt_div=sampleRateDivisor;
		MPU6000RegWrite(SMPLRT_DIV, thisMPU->smplrt_div);
		return (8000/(sampleRateDivisor+1));
	}
}

void MPU6000RegWrite(unsigned char RegAdd, unsigned char toWrite){
	unsigned char dat[2]={RegAdd, toWrite};
	I2CWrite(0x02, &dat[0]);
	while(getPortStatus() != statOpen);
}

void MPU6000ExternalSyncConfig(unsigned char FSyncSetting){
	thisMPU->config &= ~(EXT_SYNC_SET7);
	thisMPU->config |=FSyncSetting;
	MPU6000RegWrite(CONFIG, thisMPU->config);
}

void MPU6000DLPFConfig(unsigned char DLPFSetting){
		thisMPU->config &= ~(0x07);
		thisMPU->config |= DLPFSetting;
		MPU6000RegWrite(CONFIG, thisMPU->config);
}

void MPU6000GyroRangeConfig(unsigned char FullScaleRange){
	thisMPU->gyro_config &= ~(FS_2000);
	thisMPU->gyro_config |= FullScaleRange;
	MPU6000RegWrite(GYRO_CONFIG, thisMPU->gyro_config);
}

void MPU6000AccelRangeConfig(unsigned char FullScaleRange ){
	thisMPU->accel_config &= ~(AFS_16G);
	thisMPU->accel_config |= FullScaleRange;
	MPU6000RegWrite(ACCEL_CONFIG, thisMPU->accel_config);
}

void MPU6000MotionDetectThreshConfig(unsigned char Threshold){
	thisMPU->mot_thr=Threshold;
	MPU6000RegWrite(MOT_THR, thisMPU->mot_thr);
}

void MPU6000FIFOEnable(unsigned char Sensors2FIFO){
	thisMPU->fifo_en=Sensors2FIFO;
	MPU6000RegWrite(FIFO_EN, thisMPU->fifo_en);
}

void MPU6000InterruptPinConfig(unsigned char intPinSettings){
	thisMPU->int_pin_cfg=intPinSettings;
	MPU6000RegWrite(INT_PIN_CFG, thisMPU->int_pin_cfg);
}

void MPU6000EnableInterrupt(unsigned char intEnable){
	thisMPU->int_enable=intEnable;
	MPU6000RegWrite(INT_ENABLE, thisMPU->int_enable);
}

unsigned char MPU6000GetInterruptStatus(void){
	unsigned char retVal=0x00;
	I2CRead(0x01, &retVal, INT_STATUS);
	while(getPortStatus() != statOpen);
	return retVal;
}

unsigned char * MPU6000Get3AxisAccel(void){
	unsigned char *retVal=&rxDat[0];
	I2CRead(0x06, &rxDat[0], ACCEL_XOUT_H);
	while(getPortStatus() != statOpen);
	return retVal;
}

unsigned char * MPU6000GetTemp(void){
	unsigned char *retVal=&rxDat[0];
	I2CRead(0x02, &rxDat[0], TEMP_OUT_H);
	while(getPortStatus() != statOpen);
	return retVal;
}

unsigned char * MPU6000Get3AxisGyro(void){
	unsigned char *retVal=&rxDat[0];
	I2CRead(0x06, &rxDat[0], GYRO_XOUT_H);
	while(getPortStatus() != statOpen);
	return retVal;
}

void MPU6000AccelDelayConfig(unsigned char AccelDelay){
	thisMPU->mot_detect_ctrl=AccelDelay;
	MPU6000RegWrite(MOT_DETECT_CTRL, thisMPU->mot_detect_ctrl);
}

void MPU6000UserControlConfig(unsigned char UserControl){
	thisMPU->user_ctrl=UserControl;
	MPU6000RegWrite(USER_CTRL, thisMPU->user_ctrl);
}

void MPU6000Reset(void){
	MPU6000RegWrite(PWR_MGMT_1, DEVICE_RESET);
	MPUConfig();
}

void MPU6000ToggleSleep(void){
		thisMPU->pwr_mgmt_1 ^= SLEEP;
		MPU6000RegWrite(PWR_MGMT_1, thisMPU->pwr_mgmt_1);
}

void MPU6000ToggleCycleMode(void){
	thisMPU->pwr_mgmt_1 &= ~(SLEEP);
	thisMPU->pwr_mgmt_1 ^= CYCLE;
	MPU6000RegWrite(PWR_MGMT_1, thisMPU->pwr_mgmt_1);
}

void MPU6000ToggleTempSensor(void){
	thisMPU->pwr_mgmt_1 ^= TEMP_DIS;
	MPU6000RegWrite(PWR_MGMT_1, thisMPU->pwr_mgmt_1);
}

void MPU6000ClockSelect(unsigned char clockSelect){
	thisMPU->pwr_mgmt_1 &= ~(CLK_SEL_STOP);
	thisMPU->pwr_mgmt_1 |= clockSelect;
	MPU6000RegWrite(PWR_MGMT_1, thisMPU->pwr_mgmt_1);
}

void MPU6000PowerToggleMEMS(unsigned char sensors2Toggle){
	thisMPU->pwr_mgmt_2 ^= sensors2Toggle;
	MPU6000RegWrite(PWR_MGMT_2, thisMPU->pwr_mgmt_2);
}

void MPU6000LowPowerWakeControl(unsigned char LPWakeFreq){
	thisMPU->pwr_mgmt_2 &= ~(LP_WAKE_40HZ);
	thisMPU->pwr_mgmt_2 |= LPWakeFreq;
	MPU6000RegWrite(PWR_MGMT_2, thisMPU->pwr_mgmt_2);
}

/*unsigned char MPU6000TestI2C(unsigned int commID){
	//returns 0x00 if working, 0xFF if not working;

	changeI2CDevice(commID);
	unsigned char retVal=0x00;
	unsigned char dat=0x00;
	I2CRead(0x01, &dat,  WHO_AM_I);
	while(isBusy());
	if(dat==deviceArray[commID]->rAddr){
		retVal=0x00;
	}else{
		retVal=0xFF;
	}
	return retVal;
}
*/
void MPU6000Wake(void){
	//returns 0x00 if the MPU is successfully brought out of SleepMode
	//returns 0x01 if the MPU does not wake up.

	unsigned char txData[2]={PWR_MGMT_1, 0x01};
	I2CWrite(0x02, &txData[0]);
	while(isBusy());
	unsigned char txData[2]={PWR_MGMT_1, 0x81};
	I2CWrite(0x02, &txData[0]);
	while(isBusy());
    unsigned char txData[2]={PWR_MGMT_1, 0x40};
	I2CWrite(0x02, &txData[0]);
	while(isBusy());
    unsigned char txData[2]={PWR_MGMT_1, 0x00};
	I2CWrite(0x02, &txData[0]);
	while(isBusy());
}
