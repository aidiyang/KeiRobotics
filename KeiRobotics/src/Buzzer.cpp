/*
 * Buzzer.cpp
 *
 *  Created on: 2014¦~8¤ë3¤é
 *      Author: YunKei
 */

#include <App.h>
#include <Buzzer.h>
#include <Task.h>
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>

Buzzer* _mBuzzer;

Buzzer::Buzzer() : duration(0){

	GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(BUZZER_RCC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = BUZZER_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(BUZZER_PORT, &GPIO_InitStructure);
	_mBuzzer = this;
}

Buzzer* Buzzer::getInstance(){
	return _mBuzzer;
}

int Buzzer::getDuration(){
	return duration;
}

void Buzzer::setDuration(int value){
	duration = value;
}

void Buzzer::BuzzerControl(bool onState){
	GPIO_WriteBit(BUZZER_PORT, BUZZER_PIN, onState ? Bit_SET : Bit_RESET);
}

void BuzzerOn(){
	GPIO_WriteBit(BUZZER_PORT, BUZZER_PIN, Bit_SET);
}

void BuzzerOff(){
	GPIO_WriteBit(BUZZER_PORT, BUZZER_PIN, Bit_RESET);
	if(Buzzer::getInstance()->getDuration() > 0){
		Buzzer::getInstance()->setDuration(Buzzer::getInstance()->getDuration() - 1);
	}
	else if(Buzzer::getInstance()->getDuration() == 0){
//		App::mApp->mTask->DeAttach(BuzzerOn);
//		App::mApp->mTask->DeAttach(BuzzerOff);
	}
}

void Buzzer::Toggle(){
	GPIO_ToggleBits(BUZZER_PORT, BUZZER_PIN);
}

void Buzzer::Frequency(float freq, int d, bool onState){

	if(onState){
//		App::mApp->mTask->Attach((int)(1000.0f / freq), BuzzerOn, "BuzzerOn", true, -1);
//		App::mApp->mTask->Attach((int)(1000.0f / freq), (int)(800.0f / freq), BuzzerOff, "BuzzerOff", true, -1);
		if(d == -1){
			setDuration(-1);
		}
		else{
			setDuration(d / (int)(1000.0f / freq));
		}
	}
	else{
		GPIO_WriteBit(BUZZER_PORT, BUZZER_PIN, Bit_RESET);
//		App::mApp->mTask->DeAttach(BuzzerOn);
//		App::mApp->mTask->DeAttach(BuzzerOff);
	}
}
