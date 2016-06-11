/*
 * ExternalInterrupt.cpp
 *
 *  Created on: 2016¦~4¤ë9¤é
 *      Author: wongy
 */

#include <AdditionalTools.h>
#include <ExternalInterrupt.h>
#include <stm32f4xx_it.h>
#include <App.h>
#include <stdio.h>

using namespace System;
uint32_t ExternalInterrupt::EXTI_Line = 0;

ExternalInterrupt::ExternalInterrupt(Configuration* pin, Mode mode, pInterruptTask pTask) : mInterruptTask(pTask), _pin(pin), _mode(mode){

	GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(pin->_rcc, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    EXTITrigger_TypeDef EXTI_Mode;
    uint8_t EXTI_IRQ;

    switch(mode){
    	 case Mode::RISING:
    		 EXTI_Mode = EXTI_Trigger_Rising;
    		 break;
    	 case Mode::FALLING:
    		 EXTI_Mode = EXTI_Trigger_Falling;
    		 break;
    	 case Mode::RISING_FALLING:
    		 EXTI_Mode = EXTI_Trigger_Rising_Falling;
    		 break;
    }

	EXTI_DeInit();
	for(int i = 0; i < 11; i++){
		if(pin->_rcc == _BV(i)){
			EXTI_PortSourceGPIO = i;
		}
	}

	for(int i = 0; i < 16; i++){
		if(pin->_pin == _BV(i)){
			EXTI_Line |= _BV(i);
		}
		if(pin->_pin == _BV(i)){
			EXTI_PinSource = i;
		}
	}

	if(EXTI_PinSource == 0){
		EXTI_IRQ = EXTI0_IRQn;
	}
	else if(EXTI_PinSource == 1){
		EXTI_IRQ = EXTI1_IRQn;
	}
	else if(EXTI_PinSource == 2){
		EXTI_IRQ = EXTI2_IRQn;
	}
	else if(EXTI_PinSource == 3){
		EXTI_IRQ = EXTI3_IRQn;
	}
	else if(EXTI_PinSource == 4){
		EXTI_IRQ = EXTI4_IRQn;
	}
	else if(EXTI_PinSource >= 5 && EXTI_PinSource <= 9){
		EXTI_IRQ = EXTI9_5_IRQn;
	}
	else if(EXTI_PinSource >= 10 && EXTI_PinSource <= 15){
		EXTI_IRQ = EXTI15_10_IRQn;
	}

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStruct.GPIO_Pin = pin->_pin;
	GPIO_Init(pin->_port, &GPIO_InitStruct);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIO, EXTI_PinSource);

	EXTI_InitStructure.EXTI_Line = EXTI_Line;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Mode;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void EXTI0_IRQHandler(){
	if(EXTI_GetFlagStatus(EXTI_Line0) == SET)
	{
		for(int i = 0; i < 16; i++){
			if(App::mApp->mExti[i] != 0){
				if(App::mApp->mExti[i]->_pin->_pin == EXTI_Line0){
					App::mApp->mExti[i]->mInterruptTask();
				}
			}
		}
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

void EXTI1_IRQHandler(){
	if(EXTI_GetFlagStatus(EXTI_Line1) == SET)
	{
		for(int i = 0; i < 16; i++){
			if(App::mApp->mExti[i] != 0){
				if(App::mApp->mExti[i]->_pin->_pin == EXTI_Line1){
					App::mApp->mExti[i]->mInterruptTask();
				}
			}
		}
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

void EXTI2_IRQHandler(){
	if(EXTI_GetFlagStatus(EXTI_Line2) == SET)
	{
		for(int i = 0; i < 16; i++){
			if(App::mApp->mExti[i] != 0){
				if(App::mApp->mExti[i]->_pin->_pin == EXTI_Line2){
					App::mApp->mExti[i]->mInterruptTask();
				}
			}
		}
		EXTI_ClearITPendingBit(EXTI_Line2);
	}
}

void EXTI3_IRQHandler(){
	if(EXTI_GetFlagStatus(EXTI_Line3) == SET)
	{
		for(int i = 0; i < 16; i++){
			if(App::mApp->mExti[i] != 0){
				if(App::mApp->mExti[i]->_pin->_pin == EXTI_Line3){
					App::mApp->mExti[i]->mInterruptTask();
				}
			}
		}
		EXTI_ClearITPendingBit(EXTI_Line3);
	}
}

void EXTI4_IRQHandler(){
	if(EXTI_GetFlagStatus(EXTI_Line4) == SET)
	{
		for(int i = 0; i < 16; i++){
			if(App::mApp->mExti[i] != 0){
				if(App::mApp->mExti[i]->_pin->_pin == EXTI_Line4){
					App::mApp->mExti[i]->mInterruptTask();
				}
			}
		}
		EXTI_ClearITPendingBit(EXTI_Line4);
	}
}

void EXTI9_5_IRQHandler(){
	for(int i = 5; i < 10; i++){
		if(EXTI_GetFlagStatus(_BV(i)) == SET)
		{
			for(int j = 0; j < 16; j++){
				if(App::mApp->mExti[j] != 0){
					if(App::mApp->mExti[j]->_pin->_pin == _BV(i)){
						App::mApp->mExti[j]->mInterruptTask();
					}
				}
			}
			EXTI_ClearITPendingBit(_BV(i));
		}
	}
}

void EXTI15_10_IRQHandler(){
	for(int i = 6; i < 16; i++){
		if(EXTI_GetFlagStatus(_BV(i)) == SET)
		{
			for(int j = 0; j < 16; j++){
				if(App::mApp->mExti[j] != 0){
					if(App::mApp->mExti[j]->_pin->_pin == _BV(i)){
						App::mApp->mExti[j]->mInterruptTask();
					}
				}
			}
			EXTI_ClearITPendingBit(_BV(i));
		}
	}
}

