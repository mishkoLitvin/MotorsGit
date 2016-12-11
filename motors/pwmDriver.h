/*
 * pwmDriver.h
 *
 *  Created on: 16 вер. 2016
 *      Author: Mishko Lytvyn
 */

#define PWM_ePWM1_BASE_ADDR          (0x00006800)
#define PWM_ePWM2_BASE_ADDR          (0x00006840)
#define PWM_ePWM3_BASE_ADDR          (0x00006880)
#define PWM_ePWM4_BASE_ADDR          (0x000068C0)
#define PWM_ePWM5_BASE_ADDR          (0x00006900)
#define PWM_ePWM6_BASE_ADDR          (0x00006940)
#define PWM_ePWM7_BASE_ADDR          (0x00006980)
#define PWM_ePWM8_BASE_ADDR          (0x000069C0)

#include "F2806x_EPwm.h"
#include "F2806x_EPwm_defines.h"
#include "clkDriver.h"


typedef struct PWMValues_
{
	int index;
	int pwmV[3];
}PWMValues;


PWM_Handle pwmHandle[6];
EPWM_REGS *pwmRegS[6];

void pwmSInit(void);
void pwmSetup();
void setPWMValues(int channel, int value);
void setPWMValuesArr(PWMValues valueArr);
int getMaxPWMVal(int channel);

void pwmSInit(void)
{
	pwmRegS[0] = (EPWM_REGS*) PWM_ePWM1_BASE_ADDR;
	pwmRegS[1] = (EPWM_REGS*) PWM_ePWM2_BASE_ADDR;
	pwmRegS[2] = (EPWM_REGS*) PWM_ePWM3_BASE_ADDR;
	pwmRegS[3] = (EPWM_REGS*) PWM_ePWM4_BASE_ADDR;
	pwmRegS[4] = (EPWM_REGS*) PWM_ePWM5_BASE_ADDR;
	pwmRegS[5] = (EPWM_REGS*) PWM_ePWM6_BASE_ADDR;

	return;
}

//void pwmSetup()
//{
//	int i;
//	for(i=0;i<3;i++)
//	{
//		pwmRegS[i]->TBPRD = 1000; // Period = 601 TBCLK counts
//		pwmRegS[i]->CMPA.half.CMPA = 200; // Compare A = 200 TBCLK counts
//		pwmRegS[i]->CMPB = 400; // Compare B = 400 TBCLK counts
//		pwmRegS[i]->TBPHS.all = 0; // Set Phase register to zero
//		pwmRegS[i]->TBCTR = 0; // clear TB counter
//		pwmRegS[i]->TBCTL.bit.CTRMODE = 00;
//		pwmRegS[i]->TBCTL.bit.PHSEN = 0; // Phase loading disabled
//		pwmRegS[i]->TBCTL.bit.PRDLD = 0;
//		pwmRegS[i]->TBCTL.bit.SYNCOSEL = 11;
//		pwmRegS[i]->TBCTL.bit.HSPCLKDIV = 001; // TBCLK = SYSCLKOUT/2
//		pwmRegS[i]->TBCTL.bit.CLKDIV = 00;    //div sysclk/1
//		pwmRegS[i]->CMPCTL.bit.SHDWAMODE = 0;
//		pwmRegS[i]->CMPCTL.bit.SHDWBMODE = 0;
//		pwmRegS[i]->CMPCTL.bit.LOADAMODE = 00; // load on TBCTR = Zero
//		pwmRegS[i]->CMPCTL.bit.LOADBMODE = 00; // load on TBCTR = Zero
//		pwmRegS[i]->AQCTLA.bit.CAU = 01;
//		pwmRegS[i]->AQCTLA.bit.ZRO = 10;
//		pwmRegS[i]->AQCTLB.bit.CBU = 01;
//		pwmRegS[i]->AQCTLB.bit.ZRO = 11;
//	}
//}

void pwmSetupWTest()
{
	int i;
	i = 0;
	pwmRegS[i]->TBPRD = 601; // Period = 601 TBCLK counts
	pwmRegS[i]->CMPA.half.CMPA = 200; // Compare A = 200 TBCLK counts
	pwmRegS[i]->CMPB = 400; // Compare B = 400 TBCLK counts
	pwmRegS[i]->TBPHS.all = 0; // Set Phase register to zero
	pwmRegS[i]->TBCTR = 0; // clear TB counter
	pwmRegS[i]->TBCTL.bit.CTRMODE = 00;
	pwmRegS[i]->TBCTL.bit.PHSEN = 1; // Phase loading disabled
	pwmRegS[i]->TBCTL.bit.PRDLD = 0;
	pwmRegS[i]->TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
	pwmRegS[i]->TBCTL.bit.HSPCLKDIV = 001; // TBCLK = SYSCLKOUT/2
	pwmRegS[i]->TBCTL.bit.CLKDIV = 1;    //div sysclk/1
	pwmRegS[i]->CMPCTL.bit.SHDWAMODE = 0;
	pwmRegS[i]->CMPCTL.bit.SHDWBMODE = 0;
	pwmRegS[i]->CMPCTL.bit.LOADAMODE = 00; // load on TBCTR = Zero
	pwmRegS[i]->CMPCTL.bit.LOADBMODE = 00; // load on TBCTR = Zero
	pwmRegS[i]->AQCTLA.bit.CAU = 1;
	pwmRegS[i]->AQCTLA.bit.ZRO = 2;
	pwmRegS[i]->AQCTLB.bit.CBU = 1;
	pwmRegS[i]->AQCTLB.bit.ZRO = 3;

	i = 1;
	pwmRegS[i]->TBPRD = 601; // Period = 601 TBCLK counts
	pwmRegS[i]->CMPA.half.CMPA = 200; // Compare A = 200 TBCLK counts
	pwmRegS[i]->CMPB = 400; // Compare B = 400 TBCLK counts
	pwmRegS[i]->TBPHS.all = 200; // Set Phase register to zero
	pwmRegS[i]->TBCTR = 0; // clear TB counter
	pwmRegS[i]->TBCTL.bit.CTRMODE = 00;
	pwmRegS[i]->TBCTL.bit.PHSEN = 1; // Phase loading disabled
	pwmRegS[i]->TBCTL.bit.PRDLD = 0;
	pwmRegS[i]->TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
	pwmRegS[i]->TBCTL.bit.HSPCLKDIV = 001; // TBCLK = SYSCLKOUT/2
	pwmRegS[i]->TBCTL.bit.CLKDIV = 1;    //div sysclk/1
	pwmRegS[i]->CMPCTL.bit.SHDWAMODE = 0;
	pwmRegS[i]->CMPCTL.bit.SHDWBMODE = 0;
	pwmRegS[i]->CMPCTL.bit.LOADAMODE = 00; // load on TBCTR = Zero
	pwmRegS[i]->CMPCTL.bit.LOADBMODE = 00; // load on TBCTR = Zero
	pwmRegS[i]->AQCTLA.bit.CAU = 1;
	pwmRegS[i]->AQCTLA.bit.ZRO = 2;
	pwmRegS[i]->AQCTLB.bit.CBU = 1;
	pwmRegS[i]->AQCTLB.bit.ZRO = 3;

	i = 2;
	pwmRegS[i]->TBPRD = 601; // Period = 601 TBCLK counts
	pwmRegS[i]->CMPA.half.CMPA = 200; // Compare A = 200 TBCLK counts
	pwmRegS[i]->CMPB = 400; // Compare B = 400 TBCLK counts
	pwmRegS[i]->TBPHS.all = 400; // Set Phase register to zero
	pwmRegS[i]->TBCTR = 0; // clear TB counter
	pwmRegS[i]->TBCTL.bit.CTRMODE = 00;
	pwmRegS[i]->TBCTL.bit.PHSEN = 1; // Phase loading en
	pwmRegS[i]->TBCTL.bit.PRDLD = 0;
	pwmRegS[i]->TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
	pwmRegS[i]->TBCTL.bit.HSPCLKDIV = 001; // TBCLK = SYSCLKOUT/2
	pwmRegS[i]->TBCTL.bit.CLKDIV = 1;    //div sysclk/1
	pwmRegS[i]->CMPCTL.bit.SHDWAMODE = 0;
	pwmRegS[i]->CMPCTL.bit.SHDWBMODE = 0;
	pwmRegS[i]->CMPCTL.bit.LOADAMODE = 00; // load on TBCTR = Zero
	pwmRegS[i]->CMPCTL.bit.LOADBMODE = 00; // load on TBCTR = Zero
	pwmRegS[i]->AQCTLA.bit.CAU = 1;
	pwmRegS[i]->AQCTLA.bit.ZRO = 2;
	pwmRegS[i]->AQCTLB.bit.CBU = 1;
	pwmRegS[i]->AQCTLB.bit.ZRO = 3;


	i = 3;
	pwmRegS[i]->TBPRD = 601; // Period = 601 TBCLK counts
	pwmRegS[i]->CMPA.half.CMPA = 200; // Compare A = 200 TBCLK counts
	pwmRegS[i]->CMPB = 400; // Compare B = 400 TBCLK counts
	pwmRegS[i]->TBPHS.all = 0; // Set Phase register to zero
	pwmRegS[i]->TBCTR = 0; // clear TB counter
	pwmRegS[i]->TBCTL.bit.CTRMODE = 00;
	pwmRegS[i]->TBCTL.bit.PHSEN = 1; // Phase loading disabled
	pwmRegS[i]->TBCTL.bit.PRDLD = 0;
	pwmRegS[i]->TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
	pwmRegS[i]->TBCTL.bit.HSPCLKDIV = 001; // TBCLK = SYSCLKOUT/2
	pwmRegS[i]->TBCTL.bit.CLKDIV = 1;    //div sysclk/1
	pwmRegS[i]->CMPCTL.bit.SHDWAMODE = 0;
	pwmRegS[i]->CMPCTL.bit.SHDWBMODE = 0;
	pwmRegS[i]->CMPCTL.bit.LOADAMODE = 00; // load on TBCTR = Zero
	pwmRegS[i]->CMPCTL.bit.LOADBMODE = 00; // load on TBCTR = Zero
	pwmRegS[i]->AQCTLA.bit.CAU = 1;
	pwmRegS[i]->AQCTLA.bit.ZRO = 2;
	pwmRegS[i]->AQCTLB.bit.CBU = 1;
	pwmRegS[i]->AQCTLB.bit.ZRO = 3;

	i = 4;
	pwmRegS[i]->TBPRD = 601; // Period = 601 TBCLK counts
	pwmRegS[i]->CMPA.half.CMPA = 200; // Compare A = 200 TBCLK counts
	pwmRegS[i]->CMPB = 400; // Compare B = 400 TBCLK counts
	pwmRegS[i]->TBPHS.all = 200; // Set Phase register to zero
	pwmRegS[i]->TBCTR = 0; // clear TB counter
	pwmRegS[i]->TBCTL.bit.CTRMODE = 00;
	pwmRegS[i]->TBCTL.bit.PHSEN = 1; // Phase loading disabled
	pwmRegS[i]->TBCTL.bit.PRDLD = 0;
	pwmRegS[i]->TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
	pwmRegS[i]->TBCTL.bit.HSPCLKDIV = 001; // TBCLK = SYSCLKOUT/2
	pwmRegS[i]->TBCTL.bit.CLKDIV = 1;    //div sysclk/1
	pwmRegS[i]->CMPCTL.bit.SHDWAMODE = 0;
	pwmRegS[i]->CMPCTL.bit.SHDWBMODE = 0;
	pwmRegS[i]->CMPCTL.bit.LOADAMODE = 00; // load on TBCTR = Zero
	pwmRegS[i]->CMPCTL.bit.LOADBMODE = 00; // load on TBCTR = Zero
	pwmRegS[i]->AQCTLA.bit.CAU = 1;
	pwmRegS[i]->AQCTLA.bit.ZRO = 2;
	pwmRegS[i]->AQCTLB.bit.CBU = 1;
	pwmRegS[i]->AQCTLB.bit.ZRO = 3;

	i = 5;
	pwmRegS[i]->TBPRD = 601; // Period = 601 TBCLK counts
	pwmRegS[i]->CMPA.half.CMPA = 200; // Compare A = 200 TBCLK counts
	pwmRegS[i]->CMPB = 400; // Compare B = 400 TBCLK counts
	pwmRegS[i]->TBPHS.all = 400; // Set Phase register to zero
	pwmRegS[i]->TBCTR = 0; // clear TB counter
	pwmRegS[i]->TBCTL.bit.CTRMODE = 00;
	pwmRegS[i]->TBCTL.bit.PHSEN = 1; // Phase loading en
	pwmRegS[i]->TBCTL.bit.PRDLD = 0;
	pwmRegS[i]->TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
	pwmRegS[i]->TBCTL.bit.HSPCLKDIV = 001; // TBCLK = SYSCLKOUT/2
	pwmRegS[i]->TBCTL.bit.CLKDIV = 1;    //div sysclk/1
	pwmRegS[i]->CMPCTL.bit.SHDWAMODE = 0;
	pwmRegS[i]->CMPCTL.bit.SHDWBMODE = 0;
	pwmRegS[i]->CMPCTL.bit.LOADAMODE = 00; // load on TBCTR = Zero
	pwmRegS[i]->CMPCTL.bit.LOADBMODE = 00; // load on TBCTR = Zero
	pwmRegS[i]->AQCTLA.bit.CAU = 1;
	pwmRegS[i]->AQCTLA.bit.ZRO = 2;
	pwmRegS[i]->AQCTLB.bit.CBU = 1;
	pwmRegS[i]->AQCTLB.bit.ZRO = 3;
}

void setPWMValues(int channel, int value)
{
	pwmRegS[channel]->CMPA.half.CMPA = value; // adjust duty for output EPWM1A only
	pwmRegS[channel]->CMPB = pwmRegS[channel]->TBPRD-1;;
}

void disablePWM()
{
	pwmRegS[0]->CMPA.half.CMPA = 0;
	pwmRegS[1]->CMPA.half.CMPA = 0;
	pwmRegS[2]->CMPA.half.CMPA = 0;
	pwmRegS[3]->CMPA.half.CMPA = 0;
	pwmRegS[4]->CMPA.half.CMPA = 0;
	pwmRegS[5]->CMPA.half.CMPA = 0;
}

void setPWMValuesArr(PWMValues valueArr)
{
	int i;
	for(i=0;i<3; i++)
		setPWMValues(valueArr.index*3+i,valueArr.pwmV[i]);
}

int getMaxPWMVal(int channel)
{
	return pwmRegS[channel]->TBPRD-1;
}


