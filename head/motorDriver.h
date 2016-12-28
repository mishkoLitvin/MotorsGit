/*
 * motorDriver.h
 *
 *  Created on: 12 квіт. 2016
 *      Author: Mishko
 *
 *     This file contains variables for motor management,
 *     combined into one structure.
 */

#include "math.h"

#define PI 3.141593

typedef struct MotorData_
{
	int index;
	float phasePosition;
	float phasePosStep;
	float phaseTime;
	float phaseZero;
	int pwmData, pwmData0, pwmData1, pwmData2;
	int direction;
	float velocity;
	float aceleration;
	float polesCount;
	float leftPos, rightPos;

}MotorData ;

PWMValues calcPWM(MotorData *motorData)
{
	PWMValues val;
	val.index = motorData->index;
	val.pwmV[0] = motorData->pwmData*(cos(motorData->phaseTime)+1.0)/2.0;
	val.pwmV[1] = motorData->pwmData*(cos(motorData->phaseTime+2.0*PI/3.0*motorData->direction)+1.0)/2.0;
	val.pwmV[2] = motorData->pwmData*(cos(motorData->phaseTime-2.0*PI/3.0*motorData->direction)+1.0)/2.0;

	motorData->pwmData0 = val.pwmV[0];
	motorData->pwmData1 = val.pwmV[1];
	motorData->pwmData2 = val.pwmV[2];



//	val.pwmV[0] = motorData.pwmData*(cos(motorData.phaseTime+2.0*PI/3.0*motorData.direction)+1.0)/2.0;
//	val.pwmV[1] = motorData.pwmData*(cos(motorData.phaseTime)+1.0)/2.0;
//	val.pwmV[2] = motorData.pwmData*(cos(motorData.phaseTime-2.0*PI/3.0*motorData.direction)+1.0)/2.0;

	return val;
}

void stopMotor(MotorData *motorData)
{
	motorData->pwmData = 0;
}

void pidCalc()
{

}




