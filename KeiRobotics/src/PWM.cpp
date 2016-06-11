/*
 * PWM.cpp
 *
 *  Created on: 2014¦~11¤ë11¤é
 *      Author: YunKei
 */

#include <PWM.h>
#include <inttypes.h>
#include <stdio.h>
#include <Controlling.h>

using namespace Control;

PWM::PWMConfiguration::PWMConfiguration(Configuration* pwm1, Configuration* pwm2, Configuration* pwm3, Configuration* pwm4, float freq) : _pwm1(pwm1), _pwm2(pwm2), _pwm3(pwm3), _pwm4(pwm4), _freq(freq){
};

PWM::PWM(PWMConfiguration* conf) : Conf(conf), MinPWM(0), MaxPWM(10000){

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);

	RCC_AHB1PeriphClockCmd(conf->_pwm1->_rcc, ENABLE);
	RCC_AHB1PeriphClockCmd(conf->_pwm2->_rcc, ENABLE);
	RCC_AHB1PeriphClockCmd(conf->_pwm3->_rcc, ENABLE);
	RCC_AHB1PeriphClockCmd(conf->_pwm4->_rcc, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = conf->_pwm1->_pin;
	GPIO_Init(conf->_pwm1->_port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = conf->_pwm2->_pin;
	GPIO_Init(conf->_pwm2->_port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = conf->_pwm3->_pin;
	GPIO_Init(conf->_pwm3->_port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = conf->_pwm4->_pin;
	GPIO_Init(conf->_pwm4->_port, &GPIO_InitStructure);
	uint8_t pwm1Source;
	uint8_t pwm2Source;
	uint8_t pwm3Source;
	uint8_t pwm4Source;
	for(int i = 0; i < 16; i++){
		if(conf->_pwm1->_pin == _BV(i)){
			pwm1Source = i;
		}
		if(conf->_pwm2->_pin == _BV(i)){
			pwm2Source = i;
		}
		if(conf->_pwm3->_pin == _BV(i)){
			pwm3Source = i;
		}
		if(conf->_pwm4->_pin == _BV(i)){
			pwm4Source = i;
		}
	}
	GPIO_PinAFConfig(conf->_pwm1->_port, pwm1Source, GPIO_AF_TIM9);
	GPIO_PinAFConfig(conf->_pwm2->_port, pwm2Source, GPIO_AF_TIM9);
	GPIO_PinAFConfig(conf->_pwm3->_port, pwm3Source, GPIO_AF_TIM12);
	GPIO_PinAFConfig(conf->_pwm4->_port, pwm4Source, GPIO_AF_TIM12);

	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	UpperLimit1 = 42000000.0 / conf->_freq - 1;
	TIM_TimeBaseStructure.TIM_Period = (uint32_t)UpperLimit1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;

	TIM_OC1Init(TIM9, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM9, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM9, ENABLE);
	TIM_Cmd(TIM9, ENABLE);
	TIM_CtrlPWMOutputs(TIM9, ENABLE);


	UpperLimit2 = 84000000.0 / conf->_freq - 1;
	TIM_TimeBaseStructure.TIM_Period = (uint32_t)UpperLimit2;
	TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure);
	TIM_OC1Init(TIM12, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM12, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM12, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM12, ENABLE);
	TIM_Cmd(TIM12, ENABLE);
	TIM_CtrlPWMOutputs(TIM12, ENABLE);

	Control1(0);
	Control2(0);
	Control3(0);
	Control4(0);
}

void PWM::Control1(float dutyCycle){
	uint16_t value = MinPWM;
	if(dutyCycle > 10000 || dutyCycle < 0){
		return;
	}

	else{
		value += (uint16_t)(UpperLimit1 * dutyCycle / 10000.0);
		value = value > UpperLimit1 ? UpperLimit1 : value;
	}
	TIM_SetCompare1(TIM9, value);
}

void PWM::Control2(float dutyCycle){
	uint16_t value = MinPWM;
	if(dutyCycle > 10000 || dutyCycle < 0){
		return;
	}

	else{
		value += (uint16_t)(UpperLimit1 * dutyCycle / 10000);
		value = value > UpperLimit1 ? UpperLimit1 : value;
	}
	TIM_SetCompare2(TIM9, value);
}

void PWM::Control3(float dutyCycle){
	uint16_t value = MinPWM;
	if(dutyCycle > 10000 || dutyCycle < 0){
		return;
	}

	else{
		value += (uint16_t)(UpperLimit2 * dutyCycle / 10000);
		value = value > UpperLimit2 ? UpperLimit2 : value;
	}
	TIM_SetCompare1(TIM12, value);
}

void PWM::Control4(float dutyCycle){
	uint16_t value = MinPWM;
	if(dutyCycle > 10000 || dutyCycle < 0){
		return;
	}

	else{
		value += (uint16_t)(UpperLimit2 * dutyCycle / 10000);
		value = value > UpperLimit2 ? UpperLimit2 : value;
	}
	TIM_SetCompare2(TIM12, value);
}
