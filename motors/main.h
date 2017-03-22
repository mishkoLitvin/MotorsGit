/*
 * main.h
 *
 *  Created on: 25 ñ³÷. 2017
 *      Author: user
 */
#include "motorDriver.h"
#include "pid_reg3.h"
#include "conv.h"


#ifndef MAIN_H_
#define MAIN_H_

typedef enum goMode_{
	GoZeroMode = 0,
	NoNameMode, 	//1
	GoPositionMode,	//2
	GoScanMode,		//3
	GoTestMode,		//4
	GoIdleMode,		//5
	GoPiMode		//6
}goMode;

typedef enum velMoveMode_{
	MoveNotDef = 0,	//0
	MoveUpAcel,		//1
	MoveUpStable,	//2
	MoveUpDecel,	//3
	MoveUpZero,		//4
	MoveDownAcel,	//5
	MoveDownStable,	//6
	MoveDownDecel,	//7
	MoveDownZero,	//8
	MoveZeroBegin,	//9
	MoveZeroEnd		//10
}velMoveMode;


#define EPSILON
//#define BETA

#define RAD2DEG 57.295779578
#define DEG2RAD 0.0174532925

float alphaF(float a);
void main();
void SPI_RX_isr(void);
inline void zeroStart(int index);
void lockDevStart();
void makeTest();
void calcSpeed();


__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SCI_RX_isr(void);
__interrupt void GYRO_X_isr(void);
__interrupt void adc_X_ISR(void);

#ifdef FLASH
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;
extern Uint16 RamfuncsLoadSize;
#endif

FLASH_REGS *flashRegs;

unsigned int sciFramePart;
unsigned int sciCounter, sciSendCnt;
SAAD_FRAME saadFrameSend, saadFrameResive;
SAAD_TESTS saadTests;
short setSci;
short sendSCI;
short sendDubSCI;

McBSPdata mcbspData;
int apsL, apsR;
float alpha;
float alpha1, speed;

SPIData spiModule;

PIE_VECT_TABLE* interrupSVectTable;

float adcRes[16];
PWMValues pwmValues;

unsigned int sciFramePart;
unsigned int sciCounter, sciSendCnt;
SAAD_FRAME saadFrameSend, saadFrameResive;
SAAD_TESTS saadTests;
short setSci;
short sendSCI;
short sendDubSCI;
short lockDevEn;
short lockDevSend;

MotorData motor0, motor1;
extern float vel = 10, speed;
extern float acel = 300;
extern float dt = 5E-4;
long mode1cnt, mode3cnt;
unsigned long timer0cnt = 0, timer1cnt;
float timeUp, time;
goMode mode = GoZeroMode;
velMoveMode moveMode = MoveNotDef;

float iTot[2];
Vect3 abc[2];
Vect2 alBe[2];
Vect2 dq[2];
Vect2 dqP[2];
Vect2 alBeI[2];
Vect3 abcI[2];

PIDREG3 pidVel[2] = {PIDREG3_DEFAULTS, PIDREG3_DEFAULTS};
PIDREG3 pidD[2] = {PIDREG3_DEFAULTS, PIDREG3_DEFAULTS};
PIDREG3 pidQ[2] = {PIDREG3_DEFAULTS, PIDREG3_DEFAULTS};
float refD[2] = {0.0, 0.0}, refQ[2] = {0.0, 0.0};

unsigned int lockDevStepCount = 250;

void zeroStart(int index)
{
#ifdef EPSILON

	motor0.index = 0;
	motor0.pwmValues.index = 0;
	motor0.direction = 1;
	motor0.phaseZero = 23;//60.;
	motor0.phaseTime = motor0.phaseZero;
	motor0.pwmData = getMaxPWMVal(0)*0.80;
	motor0.velocity = 0;
	motor0.polesCount = 10.;

	motor1.index = 1;
	motor1.pwmValues.index = 1;
	motor1.direction = 1;
	motor1.phaseZero = 53;
	motor1.phaseTime = motor1.phaseZero;
	motor1.pwmData = getMaxPWMVal(3)*0.80;
	motor1.velocity = 0;
	motor1.polesCount = 10.;

	motor0.leftPos = 10.;
	motor0.rightPos = -10.;
	motor0.leftPosScan = 10.;
	motor0.rightPosScan = -10.;

	pidD[0].Kp = 1.;
	pidD[0].Kd = 0.;
	pidD[0].Ki = 0.;

	pidD[1].Kp = 1.;
	pidD[1].Kd = 0.;
	pidD[1].Ki = 0.;

	pidQ[0].Kp = 1.;
	pidQ[0].Kd = 0.;
	pidQ[0].Ki = 0.01;

	pidQ[1].Kp = 1.;
	pidQ[1].Kd = 0.;
	pidQ[1].Ki = 0.01;

	pidVel[0].Kp = -0.01;
	pidVel[0].Kd = -0.02;
	pidVel[0].Ki = 0.000;

	pidVel[1].Kp = -0.01;
	pidVel[1].Kd = -0.02;
	pidVel[1].Ki = 0.000;
#endif
#ifdef BETA

	motor0.phaseZero = 10.;//6.;
	motor1.phaseZero = 11.5;//3.1;

	motor0.index = 0;
	motor0.pwmValues.index = 0;
	motor0.direction = 1;
	motor0.phaseTime = motor0.phaseZero;
	motor0.pwmData = getMaxPWMVal(0)*0.8;
	motor0.velocity = 0;

	motor1.index = 1;
	motor1.pwmValues.index = 1;
	motor1.direction = 1;
	motor1.phaseTime = motor1.phaseZero;
	motor1.pwmData = getMaxPWMVal(0)*0.8;
	motor1.velocity = 0;


	motor0.polesCount = 32.757/2.;
	motor1.polesCount = 32.757/2.;

	motor0.leftPos = 20.;
	motor0.rightPos = -20.;
	motor0.leftPosScan = 20.;
	motor0.rightPosScan = -20.;

	pidD[0].Kp = 1.;
	pidD[0].Kd = 0.;
	pidD[0].Ki = 0.;

	pidD[1].Kp = 1.;
	pidD[1].Kd = 0.;
	pidD[1].Ki = 0.;

	pidQ[0].Kp = 1.;
	pidQ[0].Kd = 0.;
	pidQ[0].Ki = 0.01;

	pidQ[1].Kp = 1.;
	pidQ[1].Kd = 0.;
	pidQ[1].Ki = 0.01;

	pidVel[0].Kp = -0.01;
	pidVel[0].Kd = -0.02;
	pidVel[0].Ki = 0.000;

	pidVel[1].Kp = -0.01;
	pidVel[1].Kd = -0.02;
	pidVel[1].Ki = 0.000;

#endif


	gyroUpdateData();

	sciCounter = 0;
	sciSendCnt = 0;
	sciFramePart = 0;
	sendSCI = 0;
	sendDubSCI = 0;
	setSci = 1;

//	mode = NoNameMode;
	mode = GoIdleMode;
	mode1cnt = 0;



	motor0.position = 0;
	motor0.phasePosStep = 0;

	motor1.position = 0;
	motor1.phasePosStep = 0;

	vel = 60.;
	motor0.scanVel = 60.;
	motor0.aceleration = 300.;
	acel = 300.; //(deg / sec) / sec
	time = 0;
	timeUp = vel/acel;

	saadFrameSend.COMMAND_BYTE.all = 0;
	saadFrameSend.CTRLSUM.all = 0;
	saadFrameSend.POSITION.all = 0;
	saadFrameSend.VELOCITY.all = 0;
	saadFrameSend.START_BIT.all = 0;

	saadFrameResive.COMMAND_BYTE.all = 0;
	saadFrameResive.CTRLSUM.all = 0;
	saadFrameResive.POSITION.all = 0;
	saadFrameResive.VELOCITY.all = 0;
	saadFrameResive.START_BIT.all = 0;

	SAAD_CTRL_ALL.CTRL.all = 0;
	SAAD_CTRL_ALL.POWER = 0;
	SAAD_CTRL_ALL.CTRL.bit.LOCK_DEV = 1;
	lockDevEn = 0;
	lockDevSend = 0;
	saadTests.all = 0;

	GPIO_setLow(gpioS, LD_SLEEP);
	GPIO_setHigh(gpioS, LED1);
	GPIO_setHigh(gpioS, LED1);
	GPIO_setHigh(gpioS, LED1);
}

void makeTest()
{
	if((adcRead(5)>5)|(adcRead(13)>5)|(adcRead(15)>5)|(adcRead(7)<2340))
		saadTests.bit.CTRL_PWR = 1;
	else
		saadTests.bit.CTRL_PWR = 0;

	if((spiData->POWER_CFG!=0x0B)
			|(spiData->SENSE_CFG1!=0x29)
			|(spiData->DR_CFG!=0)
			|(spiData->spiID!=0xB1))
		saadTests.bit.CTRL_VEL = 1;
	else
		saadTests.bit.CTRL_VEL = 0;

	if(((tempData1.data1&(1<<9))>0)|((tempData2.data1&(1<<9))>0))
		saadTests.bit.CTRL_ANGLE = 1;
	else
		saadTests.bit.CTRL_ANGLE = 0;


	if((!GPIO_read(gpioS,MOTOR_1_FAULT))|(!GPIO_read(gpioS,MOTOR_2_FAULT)))
		saadTests.bit.CTRL_MOTOR = 1;
	else
		saadTests.bit.CTRL_MOTOR = 0;

	mode = GoTestMode;
}


#endif /* MAIN_H_ */
