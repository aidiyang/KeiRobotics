/*
 * Pid.cpp
 *
 *  Created on: 2014¦~11¤ë8¤é
 *      Author: YunKei
 */

#include <App.h>
#include <Pid.h>
#include <stdio.h>
#include <Ticks.h>
#include <MathTools.h>

Pid::Pid(float kp, float ki, float kd, float integralLimit) : Kp(kp), Ki(ki), Kd(kd), Integral(0), IntegralLimit(integralLimit), PreErr(0), Interval(0){
	PrevTick = App::mApp->mTicks->getTicks();
}

void Pid::setKp(float kp){
	Kp = kp;
}

void Pid::setKi(float ki){
	Ki = ki;
}

void Pid::setKd(float kd){
	Kd = kd;
}

float Pid::getKp(){
	return Kp;
}

float Pid::getKi(){
	return Ki;
}

float Pid::getKd(){
	return Kd;
}

void Pid::clear(){
	Integral = 0;
	PreErr = 0;
	PrevTick = 0;
}

float Pid::pid(float target, float current){
	Interval = App::mApp->mTicks->getTicks() - PrevTick;
	PrevTick = App::mApp->mTicks->getTicks();
	Interval /= 1000.0f;
	if(Interval <= 0){
		Interval = 1;
	}
	float err = target - current;
	Integral += err;
	Integral *= Interval;
	Integral = MathTools::Trim(-IntegralLimit, Integral, IntegralLimit);
	float derivative = err - PreErr;
	PreErr = err;
	return Kp * err + Ki * Integral + Kd * derivative / Interval;
}

float Pid::getIntegral(){
	return Integral;
}
