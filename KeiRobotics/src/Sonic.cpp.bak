/*
 * Sonic.cpp
 *
 *  Created on: 2014¦~12¤ë18¤é
 *      Author: YunKei
 */

#include <Sonic.h>
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_it.h>
#include <stm32f4xx_tim.h>
#include <App.h>
#include <stdio.h>

using namespace Sensors;

Sonic::SonicConfiguration::SonicConfiguration(Configuration* trigger, Configuration* echo, uint8_t echoSource) : _trigger(trigger), _echo(echo), _echoSource(echoSource){
}

int Sonic::OverFlowCount = 0;

Sonic::Sonic(SonicConfiguration* conf) : DeltaUS(0), Conf(conf){
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_AHB1PeriphClockCmd(conf->_echo->_rcc, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = conf->_echo->_pin;
	GPIO_Init(conf->_echo->_port, &GPIO_InitStructure);

	GPIO_PinAFConfig(conf->_echo->_port, conf->_echoSource, GPIO_AF_TIM1);

	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_Period = 10000 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0xf;

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInit(TIM1, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(TIM1, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM_ICInit(TIM1, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE ;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(TIM1, TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);

	RCC_AHB1PeriphClockCmd(conf->_trigger->_rcc, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = conf->_trigger->_pin;
	GPIO_Init(conf->_trigger->_port, &GPIO_InitStructure);
	GPIO_ResetBits(conf->_trigger->_port, conf->_trigger->_pin);

	TIM_Cmd(TIM1, ENABLE);
}

void Sonic::Reset(){
	OverFlowCount = 0;
	TIM_SetCounter(TIM1, 0);
}

void Sonic::setDeltaUS(float value){
	DeltaUS = value;
}

float Sonic::getDeltaUS(){
	return DeltaUS;
}

void Sonic::Update(){
	GPIO_SetBits(Conf->_trigger->_port, Conf->_trigger->_pin);
	Delay::DelayUS(10);
	GPIO_ResetBits(Conf->_trigger->_port, Conf->_trigger->_pin);
}

void Sonic::TriggerSet(){
	GPIO_SetBits(Conf->_trigger->_port, Conf->_trigger->_pin);
}

void Sonic::TriggerReset(){
	GPIO_ResetBits(Conf->_trigger->_port, Conf->_trigger->_pin);
}

void TIM1_CC_IRQHandler(){
	if(TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET){
		static float timestamp;
		if(GPIO_ReadInputDataBit(App::mApp->mConfig->SonicConf1->_echo->_port, App::mApp->mConfig->SonicConf1->_echo->_pin) == SET){
			timestamp = TIM_GetCapture1(TIM1) + 10000 * Sonic::OverFlowCount;
		}
		else{
			float value = TIM_GetCapture1(TIM1) + 10000 * Sonic::OverFlowCount;
			value -= timestamp;
//			value /= 168.0;
			App::mApp->mSonic1->setDeltaUS(value);
		}
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
	}

	if(TIM_GetITStatus(TIM1, TIM_IT_CC2) != RESET){
		static float timestamp;
		if(GPIO_ReadInputDataBit(App::mApp->mConfig->SonicConf2->_echo->_port, App::mApp->mConfig->SonicConf2->_echo->_pin) == SET){
			timestamp = TIM_GetCapture2(TIM1) + 10000 * Sonic::OverFlowCount;
		}
		else{
			float value = TIM_GetCapture2(TIM1) + 10000 * Sonic::OverFlowCount;
			value -= timestamp;
//			value /= 168.0;
			App::mApp->mSonic2->setDeltaUS(value);
		}
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);
	}

	if(TIM_GetITStatus(TIM1, TIM_IT_CC3) != RESET){
		static float timestamp;
		if(GPIO_ReadInputDataBit(App::mApp->mConfig->SonicConf3->_echo->_port, App::mApp->mConfig->SonicConf3->_echo->_pin) == SET){
			timestamp = TIM_GetCapture3(TIM1) + 10000 * Sonic::OverFlowCount;
		}
		else{
			float value = TIM_GetCapture3(TIM1) + 10000 * Sonic::OverFlowCount;
			value -= timestamp;
			value /= 168.0;
			App::mApp->mSonic3->setDeltaUS(value);
		}
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);
	}

	if(TIM_GetITStatus(TIM1, TIM_IT_CC4) != RESET){
		static float timestamp;
		if(GPIO_ReadInputDataBit(App::mApp->mConfig->SonicConf4->_echo->_port, App::mApp->mConfig->SonicConf4->_echo->_pin) == SET){
			timestamp = TIM_GetCapture4(TIM1) + 10000 * Sonic::OverFlowCount;
		}
		else{
			float value = TIM_GetCapture4(TIM1) + 10000 * Sonic::OverFlowCount;
			value -= timestamp;
			value /= 168.0;
			App::mApp->mSonic4->setDeltaUS(value);
		}
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);
	}
}

void TIM1_UP_TIM10_IRQHandler(){
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET){
		Sonic::OverFlowCount++;
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}
