/*
 * Encoder.cpp
 *
 *  Created on: 2016¦~2¤ë23¤é
 *      Author: wongy
 */

#include <Encoder.h>
#include <App.h>
#include <MathTools.h>

using namespace Sensors;
using namespace Math;

Encoder::EncoderConfiguration::EncoderConfiguration(Configuration* signalA,
		Configuration* signalB,
		TimerSelections timerConf, float timerClk) : _signalA(signalA),
		_signalB(signalB),
		_timerConf(timerConf),
		TimerClk(timerClk){
}

Encoder::Encoder(EncoderConfiguration* conf, float scale, float angle) : Angle(Angle), Conf(conf), Scale(scale), Interval(0), Vel(0), Pos(0), PrevVel(0), PrevRawVel(0), RawVel(0){
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	uint8_t GPIO_AF_TIMx;
	uint32_t RCC_TIMx;
	uint8_t signalASource;
	uint8_t signalBSource;
	uint16_t TIM_Prescaler;
	switch(conf->_timerConf){
		case EncoderConfiguration::TimerConf1:
			TIMx = TIM1;
			GPIO_AF_TIMx = GPIO_AF_TIM1;
			RCC_TIMx = RCC_APB2Periph_TIM1;
			TIM_Prescaler = 42000000.0f / conf->TimerClk - 1;
			break;
		case EncoderConfiguration::TimerConf2:
			TIMx = TIM8;
			GPIO_AF_TIMx = GPIO_AF_TIM8;
			RCC_TIMx = RCC_APB2Periph_TIM8;
			TIM_Prescaler = 42000000.0f / conf->TimerClk - 1;
			break;
		case EncoderConfiguration::TimerConf3:
			TIMx = TIM2;
			GPIO_AF_TIMx = GPIO_AF_TIM2;
			RCC_TIMx = RCC_APB1Periph_TIM2;
			TIM_Prescaler = 42000000.0f / conf->TimerClk - 1;
			break;
		case EncoderConfiguration::TimerConf4:
			TIMx = TIM3;
			GPIO_AF_TIMx = GPIO_AF_TIM3;
			RCC_TIMx = RCC_APB1Periph_TIM3;
			TIM_Prescaler = 42000000.0f / conf->TimerClk - 1;
			break;
		case EncoderConfiguration::TimerConf5:
			TIMx = TIM4;
			GPIO_AF_TIMx = GPIO_AF_TIM4;
			RCC_TIMx = RCC_APB1Periph_TIM4;
			TIM_Prescaler = 42000000.0f / conf->TimerClk - 1;
			break;
		case EncoderConfiguration::TimerConf6:
			TIMx = TIM5;
			GPIO_AF_TIMx = GPIO_AF_TIM5;
			RCC_TIMx = RCC_APB1Periph_TIM5;
			TIM_Prescaler = 42000000.0f / conf->TimerClk - 1;
			break;
	}

	for(int i = 0; i < 16; i++){
		if(conf->_signalA->_pin == _BV(i)){
			signalASource = i;
		}
		if(conf->_signalB->_pin == _BV(i)){
			signalBSource = i;
		}
	}

	if(conf->_timerConf < 2){
		RCC_APB2PeriphClockCmd(RCC_TIMx, ENABLE);
	}
	else{
		RCC_APB1PeriphClockCmd(RCC_TIMx, ENABLE);
	}

	RCC_AHB1PeriphClockCmd(conf->_signalA->_rcc, ENABLE);
	RCC_AHB1PeriphClockCmd(conf->_signalB->_rcc, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = conf->_signalA->_pin;
	GPIO_Init(conf->_signalA->_port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = conf->_signalB->_pin;
	GPIO_Init(conf->_signalB->_port, &GPIO_InitStructure);

	GPIO_PinAFConfig(conf->_signalA->_port, signalASource, GPIO_AF_TIMx);
	GPIO_PinAFConfig(conf->_signalB->_port, signalBSource, GPIO_AF_TIMx);

	TIM_TimeBaseStructure.TIM_Prescaler = TIM_Prescaler;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIMx, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

	TIM_SetCounter(TIMx, 32768);
	TIM_Cmd(TIMx, ENABLE);
	PrevTick = App::mApp->mTicks->getTicks();
}

void Encoder::Update(float angle){
	RawVel = (int32_t)TIM_GetCounter(TIMx) - 32768;
	TIM_SetCounter(TIMx, 32768);
	Interval = App::mApp->mTicks->getTicks() - PrevTick;
	PrevTick = App::mApp->mTicks->getTicks();
	Interval /= 1000.0f;
	if(Interval <= 0){
		return;
	}
	RawVel *= 2.0f * MathTools::PI * Scale;
	RawVel *= 100.0f;
	RawVel /= Interval;
	RawVel /= Conf->TimerClk / 1000000.0f;
//	if(App::mApp->mQuaternion != 0){
//		angle = fabs(angle);
//		angle = angle > MathTools::PI / 2.0f ? MathTools::PI - angle : angle;
//		if(MathTools::RadianToDegree(angle) > 20){
//			Vel = RawVel / calcEncoderErrorCompensationFactor(angle);
//		}
//		else{
//			Vel = RawVel;
//		}
//	}
//	else{
		Vel = RawVel;
//	}
	Pos += 0.5f * (RawVel + PrevRawVel) * Interval;
	PrevRawVel = RawVel;
}

float Encoder::getVel(){
	return Vel;
}

void Encoder::setPos(float value){
	RawVel = 0;
	PrevRawVel = 0;
	Vel = 0;
	PrevVel = 0;
	Pos = value;
}

void Encoder::Reset(){
	RawVel = 0;
	PrevRawVel = 0;
	Vel = 0;
	PrevVel = 0;
	Pos = 0;
}

float Encoder::getRawVel(){
	return RawVel;
}

float Encoder::getPos(){
	return Pos;
}

float Encoder::calcEncoderErrorCompensationFactor(float angle){
	double x8 = angle*angle*angle*angle*angle*angle*angle*angle;
	double x7 = angle*angle*angle*angle*angle*angle*angle;
	double x6 = angle*angle*angle*angle*angle*angle;
	double x5 = angle*angle*angle*angle*angle;
	double x4 = angle*angle*angle*angle;
	double x3 = angle*angle*angle;
	double x2 = angle*angle;
	double x = angle;
	return 1.0 + 0.8749*x2 - 0.7317*x + 0.169;
}
