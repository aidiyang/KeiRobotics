/*
 * PhasesMonitoring.cpp
 *
 *  Created on: 2015¦~3¤ë20¤é
 *      Author: YunKei
 */

#include <PhasesMonitoring.h>
#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_it.h>
#include <inttypes.h>
#include <Kalman.h>
#include <stdio.h>
#include <Task.h>
#include <Usart.h>

PhasesMonitoring* _mPhasesMonitoring;

void ToggleTask(){
	PhasesMonitoring::getInstance()->setInterruptIndex(PhasesMonitoring::getInstance()->getToggleCount() >= 0 &&
				PhasesMonitoring::getInstance()->getToggleCount() < 10 ? 0 :

				PhasesMonitoring::getInstance()->getToggleCount() >= 40 &&
				PhasesMonitoring::getInstance()->getToggleCount() < 50 ? 1 :

				PhasesMonitoring::getInstance()->getToggleCount() >= 80 &&
				PhasesMonitoring::getInstance()->getToggleCount() < 90 ? 2 :

				PhasesMonitoring::getInstance()->getToggleCount() >= 120 &&
				PhasesMonitoring::getInstance()->getToggleCount() < 130 ? 3 : -1);
	if(PhasesMonitoring::getInstance()->getToggleCount() == 0 ||
			PhasesMonitoring::getInstance()->getToggleCount() == 40 ||
			PhasesMonitoring::getInstance()->getToggleCount() == 80 ||
			PhasesMonitoring::getInstance()->getToggleCount() == 120){
		PhasesMonitoring::getInstance()->setInterruptEnabled(PhasesMonitoring::getInstance()->getInterruptIndex(), true);
	}
	else if(PhasesMonitoring::getInstance()->getToggleCount() == 10 ||
			PhasesMonitoring::getInstance()->getToggleCount() == 50 ||
			PhasesMonitoring::getInstance()->getToggleCount() == 90 ||
			PhasesMonitoring::getInstance()->getToggleCount() == -30){
		PhasesMonitoring::getInstance()->setInterruptEnabled(PhasesMonitoring::getInstance()->getInterruptIndex(), false);
	}
	PhasesMonitoring::getInstance()->setToggleCount(PhasesMonitoring::getInstance()->getToggleCount() + 1);
	if(PhasesMonitoring::getInstance()->getToggleCount() == 130){
		PhasesMonitoring::getInstance()->setToggleCount(-30);
	}
}

PhasesMonitoring::PhasesMonitoring() : toggleCount(0), interruptCount(0){
	_mPhasesMonitoring = this;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);

	TIM_TimeBaseStructure.TIM_Prescaler = 16-1;
	TIM_TimeBaseStructure.TIM_Period = 0xffff;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0xf;

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE ;
	NVIC_Init(&NVIC_InitStructure) ;

	TIM_Cmd(TIM3, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 | TIM_IT_Update, ENABLE);

	for(int i = 0; i < 4; i++){
		period[i] = PERIOD_UPPER_LIMIT + 1;
		OVFCount[i] = 0;
		preTimestamp[i] = 0;
		startCount[i] = STARTUP_COUNT;
		float R[2] = {0.000005f, -1};
		phaseKalman[0] = new Kalman(0.000001f, R, 0, 1.0f);
		phaseKalman[1] = new Kalman(0.000001f, R, 0, 1.0f);
		phaseKalman[2] = new Kalman(0.000001f, R, 0, 1.0f);
		phaseKalman[3] = new Kalman(0.000001f, R, 0, 1.0f);
//		phaseAverage[0] = new MovingWindowAverageFilter(5);
//		phaseAverage[1] = new MovingWindowAverageFilter(5);
//		phaseAverage[2] = new MovingWindowAverageFilter(5);
//		phaseAverage[3] = new MovingWindowAverageFilter(5);
	}
}

PhasesMonitoring* PhasesMonitoring::getInstance(){
	return _mPhasesMonitoring;
}

void PhasesMonitoring::setToggleCount(int value){
	toggleCount = value;
}

int PhasesMonitoring::getToggleCount(){
	return toggleCount;
}

void PhasesMonitoring::setInterruptCount(int value){
	interruptCount = value;
}

int PhasesMonitoring::getInterruptCount(){
	return interruptCount;
}

void PhasesMonitoring::setInterruptEnabled(int index, bool value){

	if(value){
		switch(index){
			case 0:
				TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
				break;
			case 1:
				TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
				break;
			case 2:
				TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
				break;
			case 3:
				TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);
				break;
		}
	}
	else{
		switch(index){
		case 0:
			TIM_ITConfig(TIM3, TIM_IT_CC1, DISABLE);
			break;
		case 1:
			TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);
			break;
		case 2:
			TIM_ITConfig(TIM3, TIM_IT_CC3, DISABLE);
			break;
		case 3:
			TIM_ITConfig(TIM3, TIM_IT_CC4, DISABLE);
			break;
		}

	}
}

float PhasesMonitoring::getPeriod(int index){
	return period[index];
}

void PhasesMonitoring::setPeriod(int index, float value){
	period[index] = value;
}

int PhasesMonitoring::getOVFCount(int index){
	return OVFCount[index];
}

void PhasesMonitoring::incrementOVFCount(int index){
	OVFCount[index]++;
}

void PhasesMonitoring::resetOVFCount(int index){
	OVFCount[index] = 0;
}

int PhasesMonitoring::getExpaired(int index){
	return expaired[index];
}

void PhasesMonitoring::incrementExpaired(int index){
	expaired[index]++;
}

void PhasesMonitoring::resetExpaired(int index){
	expaired[index] = 0;
}

int PhasesMonitoring::getStartCount(int index){
	return startCount[index];
}

void PhasesMonitoring::setStartCount(int index, int value){
	startCount[index] = value;
}

float PhasesMonitoring::getPreTimestamp(int index){
	return preTimestamp[index];
}

void PhasesMonitoring::setPreTimestamp(int index, float value){
	preTimestamp[index] = value;
}

float PhasesMonitoring::getRPM(int index){

	if(period[index] > PERIOD_UPPER_LIMIT){
		return 0;
	}
	return 60.0f / 7.0f * PERIOD_UPPER_LIMIT / period[index];
}

Kalman* PhasesMonitoring::getKalman(int index){
	return phaseKalman[index];
}

void PhasesMonitoring::setInterruptIndex(int value){
	interruptIndex = value;
}

int PhasesMonitoring::getInterruptIndex(){
	return interruptIndex;
}

//MovingWindowAverageFilter* PhasesMonitoring::getMovingWindowAverageFilter(int index){
//	return phaseAverage[index];
//}

void TIM3_IRQHandler(){
	PhasesMonitoring::getInstance()->setInterruptCount(PhasesMonitoring::getInstance()->getInterruptCount() + 1);
	uint8_t index = 0xff;
	float timestamp = 0;
	float value;
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET){
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

		for(int i = 0; i < 4; i++){

			PhasesMonitoring::getInstance()->setInterruptEnabled(i, true);
			PhasesMonitoring::getInstance()->incrementOVFCount(i);
			PhasesMonitoring::getInstance()->incrementExpaired(i);
			if(PhasesMonitoring::getInstance()->getExpaired(i) > STARTUP_COUNT){
				PhasesMonitoring::getInstance()->setPeriod(i, PERIOD_UPPER_LIMIT + 1);
				PhasesMonitoring::getInstance()->setStartCount(i, STARTUP_COUNT);
			}
		}
//		ToggleTask();
	}

	if(TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET){
		float temp = TIM_GetCapture1(TIM3);
		if(PhasesMonitoring::getInstance()->getPreTimestamp(0) == 0){
			PhasesMonitoring::getInstance()->setPreTimestamp(0, temp);
		}
		else{
			timestamp = temp;
		}
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
		index = 0;
	}
	else if(TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET){
		float temp = TIM_GetCapture2(TIM3);
		if(PhasesMonitoring::getInstance()->getPreTimestamp(1) == 0){
			PhasesMonitoring::getInstance()->setPreTimestamp(1, temp);
		}
		else{
			timestamp = temp;
		}
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
		index = 1;
	}
	else if(TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET){
		float temp = TIM_GetCapture3(TIM3);
		if(PhasesMonitoring::getInstance()->getPreTimestamp(2) == 0){
			PhasesMonitoring::getInstance()->setPreTimestamp(2, temp);
		}
		else{
			timestamp = temp;
		}
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
		index = 2;
	}
	else if(TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET){
		float temp = TIM_GetCapture4(TIM3);
		if(PhasesMonitoring::getInstance()->getPreTimestamp(3) == 0){
			PhasesMonitoring::getInstance()->setPreTimestamp(3, temp);
		}
		else{
			timestamp = temp;
		}
		index = 3;
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
	}
	if(index != 0xff){
		PhasesMonitoring::getInstance()->resetExpaired(index);
		if(timestamp > 0){
			value = timestamp - PhasesMonitoring::getInstance()->getPreTimestamp(index) + 65536.0 * (float)PhasesMonitoring::getInstance()->getOVFCount(index);
			if(value > PERIOD_LOWER_LIMIT){
				double temp = 0;
//				PhasesMonitoring::getInstance()->getKalman(index)->Filtering(&temp, (double)value, -1);
	//			PhasesMonitoring::getInstance()->getMovingWindowAverageFilter(index)->Update(value);
				PhasesMonitoring::getInstance()->setPeriod(index, value);//PhasesMonitoring::getInstance()->getMovingWindowAverageFilter(index)->getAverage());
				PhasesMonitoring::getInstance()->setPreTimestamp(index, 0);
				PhasesMonitoring::getInstance()->resetOVFCount(index);
				PhasesMonitoring::getInstance()->setInterruptEnabled(index, false);
			}
		}
	}
}
