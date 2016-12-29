/*
 * main.c
 */

#include "memCopy.h"
#include "gpioDriver.h"
#include "adcDriver.h"
#include "clkDriver.h"
#include "mcbspDriver.h"
#include "spiDriverMAX.h"
#include "sciDriver.h"
#include "systemDriver.h"
#include "interruptDriver.h"
#include "timerDriver.h"
#include "pwmDriver.h"
#include "scidata.h"
#include "crs8.h"
#include "F2806x_SysCtrl.h"


#include "motorDriver.h"
#include "pid_reg3.h"

#include "stdlib.h"

#define EPSILON
//#define BETA

void main(void);

FLASH_REGS *flashRegs;

McBSPdata mcbspData;
int apsL, apsR;

SPIData spiModule;
short gyroEn;

PIE_VECT_TABLE* interrupSVectTable;

long adcRes[16];
PWMValues pwmValues;

MotorData motor0, motor1;

void main();

//#pragma CODE_SECTION(cpu_timer0_isr, "ramfuncs");
//#pragma CODE_SECTION(cpu_timer1_isr, "ramfuncs");
//#pragma CODE_SECTION(cpu_timer2_isr, "ramfuncs");


__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SCI_RX_isr(void);
__interrupt void SPI_RX_isr(void);
__interrupt void GYRO_X_isr(void);



void zeroStart(int index);
void makeTest();

long j, jj;
int ch;
unsigned long timer1cnt, timer2cnt;
int stepsCount;
double temp;
long i;
int notEnd, flGyro, flPS;
long gCnt;

unsigned char motorF;

int sciRxC[4];
int sciFramePart;
unsigned int sciCounter, sciSendCnt;
SAAD_FRAME saadFrameSend, saadFrameResive;
SAAD_TESTS saadTests;
int setSci;
int sendSCI;

float alpha, alpha0;
int mode;
long mode1cnt, mode3cnt;
float acel, timeUp, vel, vel0, time;
short stabilize;

Uint32 sciErrorCntCtrlSum, sciErrorCntFrameDown, sciErrorParity,
frameCountR, frameCountT;

PIDREG3 pid = PIDREG3_DEFAULTS;

#ifdef FLASH
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;
extern Uint16 RamfuncsLoadSize;
#endif


#ifdef BETA
float alphaF(float a)
{
	return
			-6.4116+1.4
			+22.99281*a
			+10.55034*a*a
			+9.740350*a*a*a
			-49.66408*a*a*a*a
			+25.62861*a*a*a*a*a
			+103.0209*a*a*a*a*a*a
			-80.98880*a*a*a*a*a*a*a;
}
#endif


#ifdef EPSILON
float alphaF(float a)
{
	return
			-2.30137
			+22.90774*a
			+6.27669*a*a
			+8.17421*a*a*a
			-32.85827*a*a*a*a
			+2.92022*a*a*a*a*a
			+27.0813*a*a*a*a*a*a
			-12.42804*a*a*a*a*a*a*a;
}
#endif


void main(void) {

	systemInit();
	systemSetup();

	interrupsInit();
	interrupsCtrlInit();

	EALLOW;  // This is needed to write to EALLOW protected registers
	interrupSVectTable->TINT0 = &cpu_timer0_isr;
	interrupSVectTable->TINT1 = &cpu_timer1_isr;
	interrupSVectTable->TINT2 = &cpu_timer2_isr;
	interrupSVectTable->SCIRXINTA = &SCI_RX_isr;
	interrupSVectTable->SPIRXINTA = &SPI_RX_isr;
	interrupSVectTable->XINT2 = &GYRO_X_isr;
	//	interrupSVectTable->ADCINT1 = &adc_readAll;
	EDIS;


#ifdef FLASH
	flashInit();
	flashSetup();
#endif

	gpioInit();
	setupGPIOs();
	setupGPIOsInterrupt();

	mcbspaInit();
	mcbspaSetup();
	mcbspSetupSpi();

#ifdef BETA
	AMCconf(0);
#endif
#ifdef EPSILON
	AMCconf(1);
#endif

	setSPIData(&spiModule);
	spiInit();
	spiSetup();
	spiGyroConf();
	gyroVerify();
//	gyroSelfTest(spiHandle);
//	gyroUpdateData(spiHandle);

	sciaInit();
	sciaSetup();

	timerInit();
	timerSConfig();

	pwmSInit();
	pwmSetupWTest();

	adcInit();
	ADC_config();

#ifdef BETA
	zeroStart(0);
#endif

#ifdef EPSILON
	zeroStart(1);
#endif


	interruptSEnable();
	interrupsCpuSetup();

	for(;;)
	{
//		gyroVerify();

		adcRes[0] = adcRead(0);
		adcRes[1] = adcRead(1);
		adcRes[2] = adcRead(2);

		adcRes[8] = adcRead(8);
		adcRes[9] = adcRead(9);
		adcRes[10] = adcRead(10);


		if(mode == 1)
		{
			if(mode1cnt<13000)
				motor0.velocity = 10.;
			else
				if((mode1cnt>13000)&(mode1cnt<26000))
						motor0.velocity = -10.;
				else
					if(mode1cnt>26000)
					{
						mode = 2;
					}
		}
	}

}

__interrupt void cpu_timer0_isr(void)
{
	GPIO_setHigh(gpioS, LED2);
	if(SAAD_CTRL_ALL.CTRL.bit.WORK == 1)
	{
		if(time<=vel/acel)
		{
			if(fabs(motor0.velocity)>vel)
				time=vel/acel+5E-5;
			else
			{
				time+=5E-5;
				motor0.velocity = motor0.velocity+motor0.aceleration*5E-5;
			}

		}
		else
			motor0.velocity = vel*motor0.velocity/fabs(motor0.velocity);
	}

	float deltaPhT = (((motor0.velocity/180.)*PI)*5E-4)*motor0.polesCount;

	if(((fabs(motor0.phasePosition-alpha))<2)&(mode==2))
		stabilize = 1;
	else
		stabilize = 0;



	if((stabilize==1)&(mode==2))
	{
		if(fabs(motor0.phasePosition-alpha)>0.1)
			deltaPhT = ((motor0.phasePosition-alpha)/180.*PI)/motor0.polesCount;
		else
			deltaPhT = 0;

		if(deltaPhT<(PI/motor0.polesCount))
		{
#ifdef EPSILON
			motor0.phaseTime += deltaPhT*0.05;
			motor1.phaseTime += deltaPhT*0.05;
#endif
#ifdef BETA
			motor0.phaseTime += deltaPhT*0.02;
			motor1.phaseTime += deltaPhT*0.02;
#endif

		}
		else
		{
			motor0.phaseTime += PI/motor0.polesCount*0.1;
			motor1.phaseTime += PI/motor0.polesCount*0.1;
		}
	}
	else
	{
		motor0.phaseTime += deltaPhT;
		motor1.phaseTime += deltaPhT;
	}


	if(motor0.phaseTime >=2.*PI)motor0.phaseTime-=2.*PI;
	if(motor0.phaseTime <=0)motor0.phaseTime+=2.*PI;

	if(motor1.phaseTime >=2.*PI)motor1.phaseTime-=2.*PI;
	if(motor1.phaseTime <=0)motor1.phaseTime+=2.*PI;


	if(SAAD_CTRL_ALL.POWER==0)
	{
		calcPWM(&motor0);
		calcPWM(&motor1);
		for(i=0;i<6;i++)
			setPWMValues(i, 0);

	}
	else
	{
//		if(SAAD_CTRL_ALL.CTRL.bit.WORK==0)
//			for(i=0;i<3;i++)
//				setPWMValues(i, 0);
//		else
		setPWMValuesArr(calcPWM(&motor0));
		setPWMValuesArr(calcPWM(&motor1));
	}
	if((mode == 1)&(SAAD_CTRL_ALL.POWER==1)&(SAAD_CTRL_ALL.CTRL.bit.LOCK_DEV==0))
		mode1cnt++;
	GPIO_setLow(gpioS, LED2);
	interrupS->PIEACK.all = PIEACK_GROUP1;

}

__interrupt void cpu_timer2_isr(void)
{
	timer2cnt++;
	mcbspData = mcbsp_read(0x00);
	if(mcbspData.data1&0x1000)
		apsL = mcbsp_read(0x1D).data1;
	if(mcbspData.data1&0x2000)
		apsR = mcbsp_read(0x1F).data1;

	alpha = (apsL*1.-apsR*1.)/1./(apsL*1.+apsR*1.);
	alpha0 = (apsL*1.-apsR*1.)/1./(apsL*1.+apsR*1.);
	alpha = alphaF(alpha);

	if(setSci == 1)
		saadFrameSend.POSITION.all = (alpha*50);

	if(timer2cnt>300)
	{
		motor1.velocity = (motor1.phasePosition-alpha)/0.1050;
		motor1.phasePosition = alpha;
		timer2cnt = 0;
	}

	if(mode == 1)
	{
		if(alpha>motor0.rightPos)
			motor0.rightPos = alpha;
		if(alpha<motor0.leftPos)
			motor0.leftPos = alpha;
	}

//	spiWrite( 0x02, 0x01);//data update freq
	gyroUpdateData();

	if(mode == 2)
	{
		if(alpha>motor0.phasePosition)
		{
			motor0.velocity = -1.*temp;
		}
		if(alpha<motor0.phasePosition)
			motor0.velocity = 1.*temp;
	}


	if(mode == 3)
	{
		if(mode3cnt<1003)
			mode3cnt++;
		if((alpha<=(motor0.rightPos+vel*vel*0.5/acel))&(time>vel/acel))
		{
//			motor0.velocity = 0;
			motor0.aceleration = 1.*acel;
			time = -timeUp;
			mode3cnt = 0;
		}

		if((alpha>=(motor0.leftPos-vel*vel*0.5/acel))&(time>vel/acel))
		{
//			motor0.velocity = 0;
			motor0.aceleration = -1.*acel;
			time = -timeUp;
			mode3cnt = 0;
		}

	}


	flPS = 0;
//	p += dp;

}

__interrupt void cpu_timer1_isr(void)
{
	timer1cnt++;
	if((sciFramePart == 1)&
			(sciaRegs->SCIFFRX.bit.RXFFST<4))
	{
		sciaRead();sciaRead();sciaRead();sciaRead();
		sciErrorCntFrameDown++;
		sciFramePart = 0;
	}

	if((sciaRegs->SCIFFRX.bit.RXFFST>0)&
			(sciaRegs->SCIFFRX.bit.RXFFST<4))
		sciFramePart = 1;

	if(sciaRegs->SCIFFRX.bit.RXFFST==4)
	{
		frameCountR++;

		sciFramePart = 0;

		saadFrameResive.COMMAND_BYTE.bit.COMMAND_H = (0x00FF)&sciaRead();
		saadFrameResive.DATA.bit.DATA_L = (0x00FF)&sciaRead();
		saadFrameResive.DATA.bit.DATA_H = (0x00FF)&sciaRead();
		saadFrameResive.CTRLSUM.bit.CTRLSUM_H = (0x00FF)&sciaRead();

		if((saadFrameResive.COMMAND_BYTE.bit.COMMAND_H == 0x10)
				|(saadFrameResive.COMMAND_BYTE.bit.COMMAND_H == 0x20)
				|(saadFrameResive.COMMAND_BYTE.bit.COMMAND_H == 0xC0)
				|(saadFrameResive.COMMAND_BYTE.bit.COMMAND_H == 0x80)
				|(saadFrameResive.COMMAND_BYTE.bit.COMMAND_H == 0x30))
			saadFrameSend.DATA.all = saadFrameResive.DATA.all;
		saadFrameSend.COMMAND_BYTE.bit.COMMAND_H = saadFrameResive.COMMAND_BYTE.bit.COMMAND_H+1;

		if(saadFrameResive.CTRLSUM.bit.CTRLSUM_H!=crc8DATA(saadFrameResive))
			sciErrorCntCtrlSum++;
//		else
			if(saadFrameResive.COMMAND_BYTE.bit.COMMAND_H==0xC0)
			{
				if(saadFrameResive.DATA.all==0xAAAA)
					SAAD_CTRL_ALL.CTRL.bit.LOCK_DEV = 1;
				else
					if(saadFrameResive.DATA.all==0xBBBB)
						SAAD_CTRL_ALL.CTRL.bit.LOCK_DEV = 0;
				sendSCI = 1;
			}
			else
				if(saadFrameResive.COMMAND_BYTE.bit.COMMAND_H==0x80)
				{
					if(saadFrameResive.DATA.all==0xAAAA)
						SAAD_CTRL_ALL.CTRL.bit.AVTO = 1;
					else
						if(saadFrameResive.DATA.all==0xBBBB)
							SAAD_CTRL_ALL.CTRL.bit.AVTO = 0;
					sendSCI = 1;
				}
				else
					if(saadFrameResive.COMMAND_BYTE.bit.COMMAND_H==0x10)
					{
						if(saadFrameResive.DATA.all==0xAAAA)
							SAAD_CTRL_ALL.POWER = 1;
						else
							if(saadFrameResive.DATA.all==0xBBBB)
								SAAD_CTRL_ALL.POWER = 0;
						sendSCI = 1;
					}
					else
						if(saadFrameResive.COMMAND_BYTE.bit.COMMAND_H==0x30)
						{
							makeTest();
							if(saadFrameResive.CTRLSUM.bit.CTRLSUM_H!=crc8DATA(saadFrameResive))
								sciErrorCntCtrlSum++;
							sendSCI = 1;
						}
						else
#ifdef BETA
							if(saadFrameResive.COMMAND_BYTE.bit.COMMAND_H==0x50)
#endif
#ifdef EPSILON
							if(saadFrameResive.COMMAND_BYTE.bit.COMMAND_H==0x60)
#endif
							{
								saadFrameResive.POSITION.all = saadFrameResive.DATA.all;
								motor0.phasePosition =  (((float)saadFrameResive.DATA.all)*0.02);
								mode = 2;
								//							SAAD_CTRL_ALL.CTRL.bit.LOCK_DEV = 0;
								setSci = 1;
								sendSCI = 2;
							}
							else
								if(saadFrameResive.COMMAND_BYTE.bit.COMMAND_H==0x70)
								{
									mode = 3;
									temp = ((float)saadFrameResive.DATA.all)*0.02;
									vel = ((float)saadFrameResive.DATA.all)*0.02;
									motor0.aceleration = 500.;
									acel = 500.; //(deg / sec) / sec
									time = 0;
									timeUp = vel/acel;

									saadFrameResive.VELOCITY.all = saadFrameResive.DATA.all;
									setSci = 1;
									sendSCI = 2;
								}
								else
									if(saadFrameResive.COMMAND_BYTE.bit.COMMAND_H==0x20)
									{
										if(saadFrameResive.DATA.all==0xAAAA)
											SAAD_CTRL_ALL.CTRL.bit.WORK = 1;
										else
											if(saadFrameResive.DATA.all==0xBBBB)
												SAAD_CTRL_ALL.CTRL.bit.WORK = 0;
										sendSCI = 1;
									}
	}

	if((timer1cnt>2)|(sendSCI == 1)|(sendSCI == 2))
	{
		timer1cnt = 0;
		if(sendSCI>0)
			if(sciaRegs->SCIFFTX.bit.TXFFST == 0)
			{
				frameCountT++;
				if(saadFrameResive.COMMAND_BYTE.bit.COMMAND_H==0xC0)
				{
					sciaWrite(saadFrameSend.COMMAND_BYTE.bit.COMMAND_H);
					sciaWrite(saadFrameSend.DATA.bit.DATA_L);
					sciaWrite(saadFrameSend.DATA.bit.DATA_H);
				}
				else
					if(saadFrameResive.COMMAND_BYTE.bit.COMMAND_H==0x80)
					{
						sciaWrite(saadFrameSend.COMMAND_BYTE.bit.COMMAND_H);
						sciaWrite(saadFrameSend.DATA.bit.DATA_L);
						sciaWrite(saadFrameSend.DATA.bit.DATA_H);
					}
					else
						if(saadFrameResive.COMMAND_BYTE.bit.COMMAND_H==0x10)
						{
							saadFrameSend.DATA.bit.DATA_L = SAAD_CTRL_ALL.CTRL.all;
							sciaWrite(saadFrameSend.COMMAND_BYTE.bit.COMMAND_H);
							sciaWrite(saadFrameSend.DATA.bit.DATA_L);
							sciaWrite(saadFrameSend.DATA.bit.DATA_H);
						}
						else
							if(saadFrameResive.COMMAND_BYTE.bit.COMMAND_H==0x30)
							{
								if(saadTests.all == 0x00)
									saadFrameSend.COMMAND_BYTE.bit.COMMAND_H = 0x31;
								else
								{
									saadFrameSend.COMMAND_BYTE.bit.COMMAND_H = 0x32;
									saadFrameSend.DATA.bit.DATA_L = saadTests.all;
								}
								sciaWrite(saadFrameSend.COMMAND_BYTE.bit.COMMAND_H);
								sciaWrite(saadFrameSend.DATA.bit.DATA_L);
								sciaWrite(saadFrameSend.DATA.bit.DATA_H);
							}
							else
#ifdef BETA
								if(saadFrameResive.COMMAND_BYTE.bit.COMMAND_H==0x50)
#endif
#ifdef EPSILON
								if(saadFrameResive.COMMAND_BYTE.bit.COMMAND_H==0x60)
#endif
								{
									saadFrameSend.DATA.all = saadFrameSend.POSITION.all;
									sciaWrite(saadFrameSend.COMMAND_BYTE.bit.COMMAND_H);
									sciaWrite(saadFrameSend.DATA.bit.DATA_L);
									sciaWrite(saadFrameSend.DATA.bit.DATA_H);
								}
								else
									if(saadFrameResive.COMMAND_BYTE.bit.COMMAND_H==0x70)
									{
//										saadFrameSend.DATA.all = saadFrameSend.POSITION.all;
										saadFrameSend.DATA.all = saadFrameSend.VELOCITY.all;
//										saadFrameSend.DATA.all = (short) (motor0.velocity);
										sciaWrite(saadFrameSend.COMMAND_BYTE.bit.COMMAND_H);
										sciaWrite(saadFrameSend.DATA.bit.DATA_L);
										sciaWrite(saadFrameSend.DATA.bit.DATA_H);
									}
									else
										if(saadFrameResive.COMMAND_BYTE.bit.COMMAND_H==0x20)
										{
											sciaWrite(saadFrameSend.COMMAND_BYTE.bit.COMMAND_H);
											sciaWrite(saadFrameSend.DATA.bit.DATA_L);
											sciaWrite(saadFrameSend.DATA.bit.DATA_H);
										}

				sciaWrite(crc8DATA(saadFrameSend));
				if(sendSCI == 1)
					sendSCI = 0;
			}
	}
}

__interrupt void SCI_RX_isr(void)
{
	interrupS->PIEACK.all = PIEACK_GROUP9;
}

__interrupt void SPI_RX_isr(void)
{
	GPIO_toggle(gpioS, LED1);
	//	if(spiaRegs->SPIFFRX.bit.RXFFST == 4)
	{
		spiData->xH = spiaRegs->SPIRXBUF;
		spiData->xL = spiaRegs->SPIRXBUF;
		spiData->yH = spiaRegs->SPIRXBUF;
		spiData->yL = spiaRegs->SPIRXBUF;

		spiData->xData = (spiData->xH<<8)|spiData->xL;
		spiData->xAngle = spiData->xData/240.;
		spiData->yData = (spiData->yH<<8)|spiData->yL;
		spiData->yAngle = spiData->yData/240.;
	}
	if(setSci == 1)
	{
#ifdef BETA
		saadFrameSend.VELOCITY.all = spiData->xData;
#endif
#ifdef EPSILON
		saadFrameSend.VELOCITY.all = spiData->yData;
#endif

	}


	gyroEn = 0;

	spiaRegs->SPIFFRX.bit.RXFFINTCLR = 1;
	interrupS->PIEACK.all = PIEACK_GROUP6;
}

__interrupt void GYRO_X_isr(void)
{
	gyroEn = 1;
	interrupS->PIEACK.all = PIEACK_GROUP1;
}


void zeroStart(int index)
{
	if(index == 0)
	{
		motor0.index = 0;
		motor0.direction = 1;
		motor0.phaseZero = 1.1;
		motor0.phaseTime = motor0.phaseZero;
		motor0.pwmData = getMaxPWMVal(0)*0.30;
		motor0.velocity = 0;
		motor0.polesCount = 10.;

		motor1.index = 1;
		motor1.direction = 1;
		motor1.phaseZero = 3.02;
		motor1.phaseTime = motor1.phaseZero;
		motor1.pwmData = getMaxPWMVal(0)*0.30;
		motor1.velocity = 0;
		motor1.polesCount = 10.;

		motor0.leftPos = 19.;
		motor0.rightPos = -18.;
	}
	else
		if(index == 1)
		{
			motor0.index = 0;
			motor0.direction = 1;
			motor0.phaseTime = motor0.phaseZero;
			motor0.pwmData = getMaxPWMVal(0)*0.45;
			motor0.velocity = 0;

			motor1.index = 1;
			motor1.direction = 1;
			motor1.phaseTime = motor1.phaseZero;
			motor1.pwmData = getMaxPWMVal(0)*0.45;
			motor1.velocity = 0;

			motor0.phaseZero = 4.3;//6.;
			motor1.phaseZero = 0;//3.1;
			motor0.polesCount = 32.757/2.;
			motor1.polesCount = 32.757/2.;

			motor0.leftPos = 22.;
			motor0.rightPos = -22.;
		}

	timer1cnt = 0;
	stepsCount = 10;

	for(i = 0; i<1000;)i++;

	pwmValues = calcPWM(&motor0);

	flGyro = 1;
	flPS = 1;
	gyroUpdateData();
	j = 0;

	sciCounter = 0;
	sciSendCnt = 0;
	sciFramePart = 0;
	sendSCI = 0;
	setSci = 0;

	temp = 2.;

	mode = 0;
	mode1cnt = 0;

//	dp = 0;
//	p = 0;
	sciErrorCntCtrlSum = 0;
	sciErrorCntFrameDown = 0;
	sciErrorParity = 0;
	frameCountR = 0;
	frameCountT = 0;



	motor0.phasePosition = 0;
	motor0.phasePosStep = 0;

	motor1.phasePosition = 0;
	motor1.phasePosStep = 0;

	notEnd = 1;
	vel = 0;

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
	saadTests.all = 0;

	gyroEn = 0;
}

void makeTest()
{
	if((adcRead(5)>5)|(adcRead(13)>5)|(adcRead(15)>5)|(adcRead(7)<2340))
		saadTests.bit.CTRL_PWR = 1;
	else
		saadTests.bit.CTRL_PWR = 0;

	if((spiData->POWER_CFG!=0x0B)
			|(spiData->SENSE_CFG1!=0x11)
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
}
