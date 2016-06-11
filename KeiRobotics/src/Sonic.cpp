/*
 * Sonic.cpp
 *
 *  Created on: 2016¦~4¤ë13¤é
 *      Author: wongy
 */

#include <App.h>
#include <Led.h>
#include <Sonic.h>
#include <Delay.h>
#include <Configuration.h>
#include <stm32f4xx.h>
#include <stm32f4xx_it.h>
#include <stdio.h>

using namespace Sensors;
using namespace Time;

int Sonic::SonicNum = 0;
int Sonic::OverFlowCount = 0;

Sonic::SonicConfiguration::SonicConfiguration(Led::LedConfiguration* trigger, Configuration* echo) : Trigger(trigger), Echo(echo){
}

void TIM6_DAC_IRQHandler(){
	if(TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET){
		Sonic::OverFlowCount++;
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
	}
}

void SonicInterrupt1(){
	static float timestamp;
	if(GPIO_ReadInputDataBit(App::mApp->mSonic1->Echo->_pin->_port, App::mApp->mSonic1->Echo->_pin->_pin) == SET){
		timestamp = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
	}
	else{
		float value = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
		value -= timestamp;
		App::mApp->mSonic1->SonicFilter->Update(value / 5.8f);
		App::mApp->mSonic1->Distance = App::mApp->mSonic1->SonicFilter->getAverage() / 1000.0f;
	}
}

void SonicInterrupt2(){
	static float timestamp;
	if(GPIO_ReadInputDataBit(App::mApp->mSonic2->Echo->_pin->_port, App::mApp->mSonic2->Echo->_pin->_pin) == SET){
		timestamp = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
	}
	else{
		float value = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
		value -= timestamp;
		App::mApp->mSonic2->SonicFilter->Update(value / 5.8f);
		App::mApp->mSonic2->Distance = App::mApp->mSonic2->SonicFilter->getAverage() / 1000.0f;
	}
}

void SonicInterrupt3(){
	static float timestamp;
	if(GPIO_ReadInputDataBit(App::mApp->mSonic3->Echo->_pin->_port, App::mApp->mSonic3->Echo->_pin->_pin) == SET){
		timestamp = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
	}
	else{
		float value = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
		value -= timestamp;
		App::mApp->mSonic3->SonicFilter->Update(value / 5.8f);
		App::mApp->mSonic3->Distance = App::mApp->mSonic3->SonicFilter->getAverage() / 1000.0f;
	}
}

void SonicInterrupt4(){
	static float timestamp;
	if(GPIO_ReadInputDataBit(App::mApp->mSonic4->Echo->_pin->_port, App::mApp->mSonic4->Echo->_pin->_pin) == SET){
		timestamp = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
	}
	else{
		float value = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
		value -= timestamp;
		App::mApp->mSonic4->SonicFilter->Update(value / 5.8f);
		App::mApp->mSonic4->Distance = App::mApp->mSonic4->SonicFilter->getAverage() / 1000.0f;
	}
}

void SonicInterrupt5(){
	static float timestamp;
	if(GPIO_ReadInputDataBit(App::mApp->mSonic5->Echo->_pin->_port, App::mApp->mSonic5->Echo->_pin->_pin) == SET){
		timestamp = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
	}
	else{
		float value = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
		value -= timestamp;
		App::mApp->mSonic5->SonicFilter->Update(value / 5.8f);
		App::mApp->mSonic5->Distance = App::mApp->mSonic5->SonicFilter->getAverage() / 1000.0f;
	}
}

void SonicInterrupt6(){
	static float timestamp;
	if(GPIO_ReadInputDataBit(App::mApp->mSonic6->Echo->_pin->_port, App::mApp->mSonic6->Echo->_pin->_pin) == SET){
		timestamp = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
	}
	else{
		float value = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
		value -= timestamp;
		App::mApp->mSonic6->SonicFilter->Update(value / 5.8f);
		App::mApp->mSonic6->Distance = App::mApp->mSonic6->SonicFilter->getAverage() / 1000.0f;
	}
}

void SonicInterrupt7(){
	static float timestamp;
	if(GPIO_ReadInputDataBit(App::mApp->mSonic7->Echo->_pin->_port, App::mApp->mSonic7->Echo->_pin->_pin) == SET){
		timestamp = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
	}
	else{
		float value = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
		value -= timestamp;
		App::mApp->mSonic7->SonicFilter->Update(value / 5.8f);
		App::mApp->mSonic7->Distance = App::mApp->mSonic7->SonicFilter->getAverage() / 1000.0f;
	}
}

void SonicInterrupt8(){
	static float timestamp;
	if(GPIO_ReadInputDataBit(App::mApp->mSonic8->Echo->_pin->_port, App::mApp->mSonic8->Echo->_pin->_pin) == SET){
		timestamp = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
	}
	else{
		float value = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
		value -= timestamp;
		App::mApp->mSonic8->SonicFilter->Update(value / 5.8f);
		App::mApp->mSonic8->Distance = App::mApp->mSonic8->SonicFilter->getAverage() / 1000.0f;
	}
}

void SonicInterrupt9(){
	static float timestamp;
	if(GPIO_ReadInputDataBit(App::mApp->mSonic9->Echo->_pin->_port, App::mApp->mSonic9->Echo->_pin->_pin) == SET){
		timestamp = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
	}
	else{
		float value = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
		value -= timestamp;
		App::mApp->mSonic9->SonicFilter->Update(value / 5.8f);
		App::mApp->mSonic9->Distance = App::mApp->mSonic9->SonicFilter->getAverage() / 1000.0f;
	}
}

void SonicInterrupt10(){
	static float timestamp;
	if(GPIO_ReadInputDataBit(App::mApp->mSonic10->Echo->_pin->_port, App::mApp->mSonic10->Echo->_pin->_pin) == SET){
		timestamp = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
	}
	else{
		float value = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
		value -= timestamp;
		App::mApp->mSonic10->SonicFilter->Update(value / 5.8f);
		App::mApp->mSonic10->Distance = App::mApp->mSonic10->SonicFilter->getAverage() / 1000.0f;
	}
}

void SonicInterrupt11(){
	static float timestamp;
	if(GPIO_ReadInputDataBit(App::mApp->mSonic11->Echo->_pin->_port, App::mApp->mSonic11->Echo->_pin->_pin) == SET){
		timestamp = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
	}
	else{
		float value = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
		value -= timestamp;
		App::mApp->mSonic11->SonicFilter->Update(value / 5.8f);
		App::mApp->mSonic11->Distance = App::mApp->mSonic11->SonicFilter->getAverage() / 1000.0f;
	}
}

void SonicInterrupt12(){
	static float timestamp;
	if(GPIO_ReadInputDataBit(App::mApp->mSonic12->Echo->_pin->_port, App::mApp->mSonic12->Echo->_pin->_pin) == SET){
		timestamp = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
	}
	else{
		float value = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
		value -= timestamp;
		App::mApp->mSonic12->SonicFilter->Update(value / 5.8f);
		App::mApp->mSonic12->Distance = App::mApp->mSonic12->SonicFilter->getAverage() / 1000.0f;
	}
}

void SonicInterrupt13(){
	static float timestamp;
	if(GPIO_ReadInputDataBit(App::mApp->mSonic13->Echo->_pin->_port, App::mApp->mSonic13->Echo->_pin->_pin) == SET){
		timestamp = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
	}
	else{
		float value = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
		value -= timestamp;
		App::mApp->mSonic13->SonicFilter->Update(value / 5.8f);
		App::mApp->mSonic13->Distance = App::mApp->mSonic13->SonicFilter->getAverage() / 1000.0f;
	}
}

void SonicInterrupt14(){
	static float timestamp;
	if(GPIO_ReadInputDataBit(App::mApp->mSonic14->Echo->_pin->_port, App::mApp->mSonic14->Echo->_pin->_pin) == SET){
		timestamp = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
	}
	else{
		float value = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
		value -= timestamp;
		App::mApp->mSonic14->SonicFilter->Update(value / 5.8f);
		App::mApp->mSonic14->Distance = App::mApp->mSonic14->SonicFilter->getAverage() / 1000.0f;
	}
}

void SonicInterrupt15(){
	static float timestamp;
	if(GPIO_ReadInputDataBit(App::mApp->mSonic15->Echo->_pin->_port, App::mApp->mSonic15->Echo->_pin->_pin) == SET){
		timestamp = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
	}
	else{
		float value = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
		value -= timestamp;
		App::mApp->mSonic15->SonicFilter->Update(value / 5.8f);
		App::mApp->mSonic15->Distance = App::mApp->mSonic15->SonicFilter->getAverage() / 1000.0f;
	}
}

void SonicInterrupt16(){
	static float timestamp;
	if(GPIO_ReadInputDataBit(App::mApp->mSonic16->Echo->_pin->_port, App::mApp->mSonic16->Echo->_pin->_pin) == SET){
		timestamp = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
	}
	else{
		float value = TIM_GetCounter(TIM6) + 10000 * Sonic::OverFlowCount;
		value -= timestamp;
		App::mApp->mSonic16->SonicFilter->Update(value / 5.8f);
		App::mApp->mSonic16->Distance = App::mApp->mSonic16->SonicFilter->getAverage() / 1000.0f;
	}
}

Sonic::Sonic(SonicConfiguration* conf) : Conf(conf), Distance(0){
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE ;
	NVIC_Init(&NVIC_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	TIM_TimeBaseStructure.TIM_Prescaler = 83;
	TIM_TimeBaseStructure.TIM_Period = 10000 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM6, ENABLE);

	switch(SonicNum){
		case 0:
			Echo = new ExternalInterrupt(conf->Echo, ExternalInterrupt::RISING_FALLING, SonicInterrupt1);
			break;
		case 1:
			Echo = new ExternalInterrupt(conf->Echo, ExternalInterrupt::RISING_FALLING, SonicInterrupt2);
			break;
		case 2:
			Echo = new ExternalInterrupt(conf->Echo, ExternalInterrupt::RISING_FALLING, SonicInterrupt3);
			break;
		case 3:
			Echo = new ExternalInterrupt(conf->Echo, ExternalInterrupt::RISING_FALLING, SonicInterrupt4);
			break;
		case 4:
			Echo = new ExternalInterrupt(conf->Echo, ExternalInterrupt::RISING_FALLING, SonicInterrupt5);
			break;
		case 5:
			Echo = new ExternalInterrupt(conf->Echo, ExternalInterrupt::RISING_FALLING, SonicInterrupt6);
			break;
		case 6:
			Echo = new ExternalInterrupt(conf->Echo, ExternalInterrupt::RISING_FALLING, SonicInterrupt7);
			break;
		case 7:
			Echo = new ExternalInterrupt(conf->Echo, ExternalInterrupt::RISING_FALLING, SonicInterrupt8);
			break;
		case 8:
			Echo = new ExternalInterrupt(conf->Echo, ExternalInterrupt::RISING_FALLING, SonicInterrupt9);
			break;
		case 9:
			Echo = new ExternalInterrupt(conf->Echo, ExternalInterrupt::RISING_FALLING, SonicInterrupt10);
			break;
		case 10:
			Echo = new ExternalInterrupt(conf->Echo, ExternalInterrupt::RISING_FALLING, SonicInterrupt11);
			break;
		case 11:
			Echo = new ExternalInterrupt(conf->Echo, ExternalInterrupt::RISING_FALLING, SonicInterrupt12);
			break;
		case 12:
			Echo = new ExternalInterrupt(conf->Echo, ExternalInterrupt::RISING_FALLING, SonicInterrupt13);
			break;
		case 13:
			Echo = new ExternalInterrupt(conf->Echo, ExternalInterrupt::RISING_FALLING, SonicInterrupt14);
			break;
		case 14:
			Echo = new ExternalInterrupt(conf->Echo, ExternalInterrupt::RISING_FALLING, SonicInterrupt15);
			break;
		case 15:
			Echo = new ExternalInterrupt(conf->Echo, ExternalInterrupt::RISING_FALLING, SonicInterrupt16);
			break;
	}
	App::mApp->mSonicTrigger[SonicNum] = Trigger = new Led(conf->Trigger);
	App::mApp->mExti[SonicNum] = Echo;
	SonicNum++;
	SonicFilter = new MovingWindowAverageFilter(2);
}

void Sonic::Update(){
	Trigger->LedControl(true);
	Delay::DelayUS(10);
	Trigger->LedControl(false);
}

