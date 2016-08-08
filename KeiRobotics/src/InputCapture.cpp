/*
 * InputCapture.cpp
 *
 *  Created on: 2016¦~7¤ë29¤é
 *      Author: wongy
 */

#include <InputCapture.h>
#include <stm32f4xx_it.h>
#include <App.h>
#include <UART.h>

using namespace Sensors;

InputCaptureConfiguration::InputCaptureConfiguration(Configuration* cap1, Configuration* cap2, Configuration* cap3, Configuration* cap4, TimerConf timerConf, float timerClk) : _cap1(cap1), _cap2(cap2), _cap3(cap3), _cap4(cap4), _timerConf(timerConf), TimerClk(timerClk){

}

InputCapture::InputCapture(InputCaptureConfiguration* conf) : Conf(conf),
		PrevTimestamp1(0),
		PrevTimestamp2(0),
		PrevTimestamp3(0),
		PrevTimestamp4(0),
		PrevOVFCount1(0),
		PrevOVFCount2(0),
		PrevOVFCount3(0),
		PrevOVFCount4(0),
		Period1(0),
		Period2(0),
		Period3(0),
		Period4(0),
		OVFCount(0){

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	uint8_t GPIO_AF_TIMx;
	uint32_t RCC_TIMx;
	uint8_t cap1Source;
	uint8_t cap2Source;
	uint8_t cap3Source;
	uint8_t cap4Source;
	uint16_t TIM_Prescaler;
	uint8_t TIMx_IRQn;

	switch(conf->_timerConf){
		case InputCaptureConfiguration::TimerConf1:
			TIMx = TIM1;
			GPIO_AF_TIMx = GPIO_AF_TIM1;
			RCC_TIMx = RCC_APB2Periph_TIM1;
			TIMx_IRQn = TIM1_CC_IRQn;
			TIM_Prescaler = 42000000.0f / conf->TimerClk - 1;
			break;
		case InputCaptureConfiguration::TimerConf2:
			TIMx = TIM8;
			GPIO_AF_TIMx = GPIO_AF_TIM8;
			RCC_TIMx = RCC_APB2Periph_TIM8;
			TIMx_IRQn = TIM8_CC_IRQn;
			TIM_Prescaler = 42000000.0f / conf->TimerClk - 1;
			break;
		case InputCaptureConfiguration::TimerConf3:
			TIMx = TIM2;
			GPIO_AF_TIMx = GPIO_AF_TIM2;
			RCC_TIMx = RCC_APB1Periph_TIM2;
			TIMx_IRQn = TIM2_IRQn;
			TIM_Prescaler = 42000000.0f / conf->TimerClk - 1;
			break;
		case InputCaptureConfiguration::TimerConf4:
			TIMx = TIM3;
			GPIO_AF_TIMx = GPIO_AF_TIM3;
			RCC_TIMx = RCC_APB1Periph_TIM3;
			TIMx_IRQn = TIM3_IRQn;
			TIM_Prescaler = 42000000.0f / conf->TimerClk - 1;
			break;
		case InputCaptureConfiguration::TimerConf5:
			TIMx = TIM4;
			GPIO_AF_TIMx = GPIO_AF_TIM4;
			RCC_TIMx = RCC_APB1Periph_TIM4;
			TIMx_IRQn = TIM4_IRQn;
			TIM_Prescaler = 42000000.0f / conf->TimerClk - 1;
			break;
		case InputCaptureConfiguration::TimerConf6:
			TIMx = TIM5;
			GPIO_AF_TIMx = GPIO_AF_TIM5;
			RCC_TIMx = RCC_APB1Periph_TIM5;
			TIMx_IRQn = TIM5_IRQn;
			TIM_Prescaler = 42000000.0f / conf->TimerClk - 1;
			break;
	}

	if(conf->_timerConf < 2){
		RCC_APB2PeriphClockCmd(RCC_TIMx, ENABLE);
	}
	else{
		RCC_APB1PeriphClockCmd(RCC_TIMx, ENABLE);
	}

	for(int i = 0; i < 16; i++){
		if(conf->_cap1->_pin == _BV(i)){
			cap1Source = i;
		}
		if(conf->_cap2->_pin == _BV(i)){
			cap2Source = i;
		}
		if(conf->_cap3->_pin == _BV(i)){
			cap3Source = i;
		}
		if(conf->_cap4->_pin == _BV(i)){
			cap4Source = i;
		}
	}

	RCC_AHB1PeriphClockCmd(conf->_cap1->_rcc, ENABLE);
	RCC_AHB1PeriphClockCmd(conf->_cap2->_rcc, ENABLE);
	RCC_AHB1PeriphClockCmd(conf->_cap3->_rcc, ENABLE);
	RCC_AHB1PeriphClockCmd(conf->_cap4->_rcc, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = conf->_cap1->_pin;
	GPIO_Init(conf->_cap1->_port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = conf->_cap2->_pin;
	GPIO_Init(conf->_cap2->_port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = conf->_cap3->_pin;
	GPIO_Init(conf->_cap3->_port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = conf->_cap4->_pin;
	GPIO_Init(conf->_cap4->_port, &GPIO_InitStructure);

	GPIO_PinAFConfig(conf->_cap1->_port, cap1Source, GPIO_AF_TIMx);
	GPIO_PinAFConfig(conf->_cap2->_port, cap2Source, GPIO_AF_TIMx);
	GPIO_PinAFConfig(conf->_cap3->_port, cap3Source, GPIO_AF_TIMx);
	GPIO_PinAFConfig(conf->_cap4->_port, cap4Source, GPIO_AF_TIMx);

	TIM_TimeBaseStructure.TIM_Prescaler = TIM_Prescaler;
	TIM_TimeBaseStructure.TIM_Period = 0xffff;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0xf;

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInit(TIMx, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(TIMx, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM_ICInit(TIMx, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInit(TIMx, &TIM_ICInitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIMx_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE ;
	NVIC_Init(&NVIC_InitStructure);

	if(TIMx == TIM1){
		NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
		NVIC_Init(&NVIC_InitStructure);
	}
	else if(TIMx == TIM8){
		NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn;
		NVIC_Init(&NVIC_InitStructure);
	}

	TIM_Cmd(TIMx, ENABLE);
	TIM_ITConfig(TIMx, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 | TIM_IT_Update, ENABLE);
}

void TIM1_UP_TIM10_IRQHandler(){

	if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET){
		App::mApp->mInputCapture->OVFCount++;
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}

void TIM8_UP_TIM13_IRQHandler(){
	if(TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET){
		App::mApp->mInputCapture->OVFCount++;
		TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
	}
}

void TIM1_CC_IRQHandler(){

	if(TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET){
		float Timestamp1 = TIM_GetCapture1(TIM1);
		App::mApp->mInputCapture->Period1 = Timestamp1 - App::mApp->mInputCapture->PrevTimestamp1 + (App::mApp->mInputCapture->OVFCount - App::mApp->mInputCapture->PrevOVFCount1) * 65536;
		App::mApp->mInputCapture->Period1 /= App::mApp->mInputCapture->Conf->TimerClk;
		App::mApp->mInputCapture->PrevTimestamp1 = Timestamp1;
		App::mApp->mInputCapture->PrevOVFCount1 = App::mApp->mInputCapture->OVFCount;
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
	}
	else if(TIM_GetITStatus(TIM1, TIM_IT_CC2) != RESET){
		float Timestamp2 = TIM_GetCapture2(TIM1);
		App::mApp->mInputCapture->Period2 = Timestamp2 - App::mApp->mInputCapture->PrevTimestamp2 + (App::mApp->mInputCapture->OVFCount - App::mApp->mInputCapture->PrevOVFCount2) * 65536;
		App::mApp->mInputCapture->Period2 /= App::mApp->mInputCapture->Conf->TimerClk;
		App::mApp->mInputCapture->PrevTimestamp2 = Timestamp2;
		App::mApp->mInputCapture->PrevOVFCount2 = App::mApp->mInputCapture->OVFCount;
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);
	}
	else if(TIM_GetITStatus(TIM1, TIM_IT_CC3) != RESET){
		float Timestamp3 = TIM_GetCapture3(TIM1);
		App::mApp->mInputCapture->Period3 = Timestamp3 - App::mApp->mInputCapture->PrevTimestamp3 + (App::mApp->mInputCapture->OVFCount - App::mApp->mInputCapture->PrevOVFCount3) * 65536;
		App::mApp->mInputCapture->Period3 /= App::mApp->mInputCapture->Conf->TimerClk;
		App::mApp->mInputCapture->PrevTimestamp3 = Timestamp3;
		App::mApp->mInputCapture->PrevOVFCount3 = App::mApp->mInputCapture->OVFCount;
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);
	}
	else if(TIM_GetITStatus(TIM1, TIM_IT_CC4) != RESET){
		float Timestamp4 = TIM_GetCapture4(TIM1);
		App::mApp->mInputCapture->Period4 = Timestamp4 - App::mApp->mInputCapture->PrevTimestamp4 + (App::mApp->mInputCapture->OVFCount - App::mApp->mInputCapture->PrevOVFCount4) * 65536;
		App::mApp->mInputCapture->Period4 /= App::mApp->mInputCapture->Conf->TimerClk;
		App::mApp->mInputCapture->PrevTimestamp4 = Timestamp4;
		App::mApp->mInputCapture->PrevOVFCount4 = App::mApp->mInputCapture->OVFCount;
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);
	}
}

void TIM8_CC_IRQHandler(){
	if(TIM_GetITStatus(TIM8, TIM_IT_CC1) != RESET){
		float Timestamp1 = TIM_GetCapture1(TIM8);
		App::mApp->mInputCapture->Period1 = Timestamp1 - App::mApp->mInputCapture->PrevTimestamp1 + (App::mApp->mInputCapture->OVFCount - App::mApp->mInputCapture->PrevOVFCount1) * 65536;
		App::mApp->mInputCapture->Period1 /= App::mApp->mInputCapture->Conf->TimerClk;
		App::mApp->mInputCapture->PrevTimestamp1 = Timestamp1;
		App::mApp->mInputCapture->PrevOVFCount1 = App::mApp->mInputCapture->OVFCount;
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC1);
	}
	else if(TIM_GetITStatus(TIM8, TIM_IT_CC2) != RESET){
		float Timestamp2 = TIM_GetCapture2(TIM8);
		App::mApp->mInputCapture->Period2 = Timestamp2 - App::mApp->mInputCapture->PrevTimestamp2 + (App::mApp->mInputCapture->OVFCount - App::mApp->mInputCapture->PrevOVFCount2) * 65536;
		App::mApp->mInputCapture->Period2 /= App::mApp->mInputCapture->Conf->TimerClk;
		App::mApp->mInputCapture->PrevTimestamp2 = Timestamp2;
		App::mApp->mInputCapture->PrevOVFCount2 = App::mApp->mInputCapture->OVFCount;
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC2);
	}
	else if(TIM_GetITStatus(TIM8, TIM_IT_CC3) != RESET){
		float Timestamp3 = TIM_GetCapture3(TIM8);
		App::mApp->mInputCapture->Period3 = Timestamp3 - App::mApp->mInputCapture->PrevTimestamp3 + (App::mApp->mInputCapture->OVFCount - App::mApp->mInputCapture->PrevOVFCount3) * 65536;
		App::mApp->mInputCapture->Period3 /= App::mApp->mInputCapture->Conf->TimerClk;
		App::mApp->mInputCapture->PrevTimestamp3 = Timestamp3;
		App::mApp->mInputCapture->PrevOVFCount3 = App::mApp->mInputCapture->OVFCount;
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC3);
	}
	else if(TIM_GetITStatus(TIM8, TIM_IT_CC4) != RESET){
		float Timestamp4 = TIM_GetCapture4(TIM8);
		App::mApp->mInputCapture->Period4 = Timestamp4 - App::mApp->mInputCapture->PrevTimestamp4 + (App::mApp->mInputCapture->OVFCount - App::mApp->mInputCapture->PrevOVFCount4) * 65536;
		App::mApp->mInputCapture->Period4 /= App::mApp->mInputCapture->Conf->TimerClk;
		App::mApp->mInputCapture->PrevTimestamp4 = Timestamp4;
		App::mApp->mInputCapture->PrevOVFCount4 = App::mApp->mInputCapture->OVFCount;
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC4);
	}
}

void TIM2_IRQHandler(){
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){
		App::mApp->mInputCapture->OVFCount++;
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
	else if(TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET){
		float Timestamp1 = TIM_GetCapture1(TIM2);
		App::mApp->mInputCapture->Period1 = Timestamp1 - App::mApp->mInputCapture->PrevTimestamp1 + (App::mApp->mInputCapture->OVFCount - App::mApp->mInputCapture->PrevOVFCount1) * 65536;
		App::mApp->mInputCapture->Period1 /= App::mApp->mInputCapture->Conf->TimerClk;
		App::mApp->mInputCapture->PrevTimestamp1 = Timestamp1;
		App::mApp->mInputCapture->PrevOVFCount1 = App::mApp->mInputCapture->OVFCount;
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
	}
	else if(TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET){
		float Timestamp2 = TIM_GetCapture2(TIM2);
		App::mApp->mInputCapture->Period2 = Timestamp2 - App::mApp->mInputCapture->PrevTimestamp2 + (App::mApp->mInputCapture->OVFCount - App::mApp->mInputCapture->PrevOVFCount2) * 65536;
		App::mApp->mInputCapture->Period2 /= App::mApp->mInputCapture->Conf->TimerClk;
		App::mApp->mInputCapture->PrevTimestamp2 = Timestamp2;
		App::mApp->mInputCapture->PrevOVFCount2 = App::mApp->mInputCapture->OVFCount;
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
	}
	else if(TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET){
		float Timestamp3 = TIM_GetCapture3(TIM2);
		App::mApp->mInputCapture->Period3 = Timestamp3 - App::mApp->mInputCapture->PrevTimestamp3 + (App::mApp->mInputCapture->OVFCount - App::mApp->mInputCapture->PrevOVFCount3) * 65536;
		App::mApp->mInputCapture->Period3 /= App::mApp->mInputCapture->Conf->TimerClk;
		App::mApp->mInputCapture->PrevTimestamp3 = Timestamp3;
		App::mApp->mInputCapture->PrevOVFCount3 = App::mApp->mInputCapture->OVFCount;
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
	}
	else if(TIM_GetITStatus(TIM2, TIM_IT_CC4) != RESET){
		float Timestamp4 = TIM_GetCapture4(TIM2);
		App::mApp->mInputCapture->Period4 = Timestamp4 - App::mApp->mInputCapture->PrevTimestamp4 + (App::mApp->mInputCapture->OVFCount - App::mApp->mInputCapture->PrevOVFCount4) * 65536;
		App::mApp->mInputCapture->Period4 /= App::mApp->mInputCapture->Conf->TimerClk;
		App::mApp->mInputCapture->PrevTimestamp4 = Timestamp4;
		App::mApp->mInputCapture->PrevOVFCount4 = App::mApp->mInputCapture->OVFCount;
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
	}
}

void TIM3_IRQHandler(){
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET){
		App::mApp->mInputCapture->OVFCount++;
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
	else if(TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET){
		float Timestamp1 = TIM_GetCapture1(TIM3);
		App::mApp->mInputCapture->Period1 = Timestamp1 - App::mApp->mInputCapture->PrevTimestamp1 + (App::mApp->mInputCapture->OVFCount - App::mApp->mInputCapture->PrevOVFCount1) * 65536;
		App::mApp->mInputCapture->Period1 /= App::mApp->mInputCapture->Conf->TimerClk;
		App::mApp->mInputCapture->PrevTimestamp1 = Timestamp1;
		App::mApp->mInputCapture->PrevOVFCount1 = App::mApp->mInputCapture->OVFCount;
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
	}
	else if(TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET){
		float Timestamp2 = TIM_GetCapture2(TIM3);
		App::mApp->mInputCapture->Period2 = Timestamp2 - App::mApp->mInputCapture->PrevTimestamp2 + (App::mApp->mInputCapture->OVFCount - App::mApp->mInputCapture->PrevOVFCount2) * 65536;
		App::mApp->mInputCapture->Period2 /= App::mApp->mInputCapture->Conf->TimerClk;
		App::mApp->mInputCapture->PrevTimestamp2 = Timestamp2;
		App::mApp->mInputCapture->PrevOVFCount2 = App::mApp->mInputCapture->OVFCount;
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
	}
	else if(TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET){
		float Timestamp3 = TIM_GetCapture3(TIM3);
		App::mApp->mInputCapture->Period3 = Timestamp3 - App::mApp->mInputCapture->PrevTimestamp3 + (App::mApp->mInputCapture->OVFCount - App::mApp->mInputCapture->PrevOVFCount3) * 65536;
		App::mApp->mInputCapture->Period3 /= App::mApp->mInputCapture->Conf->TimerClk;
		App::mApp->mInputCapture->PrevTimestamp3 = Timestamp3;
		App::mApp->mInputCapture->PrevOVFCount3 = App::mApp->mInputCapture->OVFCount;
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
	}
	else if(TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET){
		float Timestamp4 = TIM_GetCapture4(TIM3);
		App::mApp->mInputCapture->Period4 = Timestamp4 - App::mApp->mInputCapture->PrevTimestamp4 + (App::mApp->mInputCapture->OVFCount - App::mApp->mInputCapture->PrevOVFCount4) * 65536;
		App::mApp->mInputCapture->Period4 /= App::mApp->mInputCapture->Conf->TimerClk;
		App::mApp->mInputCapture->PrevTimestamp4 = Timestamp4;
		App::mApp->mInputCapture->PrevOVFCount4 = App::mApp->mInputCapture->OVFCount;
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
	}
}

void TIM4_IRQHandler(){
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET){
		App::mApp->mInputCapture->OVFCount++;
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
	else if(TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET){
		float Timestamp1 = TIM_GetCapture1(TIM4);
		App::mApp->mInputCapture->Period1 = Timestamp1 - App::mApp->mInputCapture->PrevTimestamp1 + (App::mApp->mInputCapture->OVFCount - App::mApp->mInputCapture->PrevOVFCount1) * 65536;
		App::mApp->mInputCapture->Period1 /= App::mApp->mInputCapture->Conf->TimerClk;
		App::mApp->mInputCapture->PrevTimestamp1 = Timestamp1;
		App::mApp->mInputCapture->PrevOVFCount1 = App::mApp->mInputCapture->OVFCount;
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
	}
	else if(TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET){
		float Timestamp2 = TIM_GetCapture2(TIM4);
		App::mApp->mInputCapture->Period2 = Timestamp2 - App::mApp->mInputCapture->PrevTimestamp2 + (App::mApp->mInputCapture->OVFCount - App::mApp->mInputCapture->PrevOVFCount2) * 65536;
		App::mApp->mInputCapture->Period2 /= App::mApp->mInputCapture->Conf->TimerClk;
		App::mApp->mInputCapture->PrevTimestamp2 = Timestamp2;
		App::mApp->mInputCapture->PrevOVFCount2 = App::mApp->mInputCapture->OVFCount;
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
	}
	else if(TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET){
		float Timestamp3 = TIM_GetCapture3(TIM4);
		App::mApp->mInputCapture->Period3 = Timestamp3 - App::mApp->mInputCapture->PrevTimestamp3 + (App::mApp->mInputCapture->OVFCount - App::mApp->mInputCapture->PrevOVFCount3) * 65536;
		App::mApp->mInputCapture->Period3 /= App::mApp->mInputCapture->Conf->TimerClk;
		App::mApp->mInputCapture->PrevTimestamp3 = Timestamp3;
		App::mApp->mInputCapture->PrevOVFCount3 = App::mApp->mInputCapture->OVFCount;
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);
	}
	else if(TIM_GetITStatus(TIM4, TIM_IT_CC4) != RESET){
		float Timestamp4 = TIM_GetCapture4(TIM4);
		App::mApp->mInputCapture->Period4 = Timestamp4 - App::mApp->mInputCapture->PrevTimestamp4 + (App::mApp->mInputCapture->OVFCount - App::mApp->mInputCapture->PrevOVFCount4) * 65536;
		App::mApp->mInputCapture->Period4 /= App::mApp->mInputCapture->Conf->TimerClk;
		App::mApp->mInputCapture->PrevTimestamp4 = Timestamp4;
		App::mApp->mInputCapture->PrevOVFCount4 = App::mApp->mInputCapture->OVFCount;
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);
	}
}

void TIM5_IRQHandler(){
	if(TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET){
		App::mApp->mInputCapture->OVFCount++;
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	}
	else if(TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET){
		float Timestamp1 = TIM_GetCapture1(TIM5);
		App::mApp->mInputCapture->Period1 = Timestamp1 - App::mApp->mInputCapture->PrevTimestamp1 + (App::mApp->mInputCapture->OVFCount - App::mApp->mInputCapture->PrevOVFCount1) * 65536;
		App::mApp->mInputCapture->Period1 /= App::mApp->mInputCapture->Conf->TimerClk;
		App::mApp->mInputCapture->PrevTimestamp1 = Timestamp1;
		App::mApp->mInputCapture->PrevOVFCount1 = App::mApp->mInputCapture->OVFCount;
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC1);
	}
	else if(TIM_GetITStatus(TIM5, TIM_IT_CC2) != RESET){
		float Timestamp2 = TIM_GetCapture2(TIM5);
		App::mApp->mInputCapture->Period2 = Timestamp2 - App::mApp->mInputCapture->PrevTimestamp2 + (App::mApp->mInputCapture->OVFCount - App::mApp->mInputCapture->PrevOVFCount2) * 65536;
		App::mApp->mInputCapture->Period2 /= App::mApp->mInputCapture->Conf->TimerClk;
		App::mApp->mInputCapture->PrevTimestamp2 = Timestamp2;
		App::mApp->mInputCapture->PrevOVFCount2 = App::mApp->mInputCapture->OVFCount;
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC2);
	}
	else if(TIM_GetITStatus(TIM5, TIM_IT_CC3) != RESET){
		float Timestamp3 = TIM_GetCapture3(TIM5);
		App::mApp->mInputCapture->Period3 = Timestamp3 - App::mApp->mInputCapture->PrevTimestamp3 + (App::mApp->mInputCapture->OVFCount - App::mApp->mInputCapture->PrevOVFCount3) * 65536;
		App::mApp->mInputCapture->Period3 /= App::mApp->mInputCapture->Conf->TimerClk;
		App::mApp->mInputCapture->PrevTimestamp3 = Timestamp3;
		App::mApp->mInputCapture->PrevOVFCount3 = App::mApp->mInputCapture->OVFCount;
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC3);
	}
	else if(TIM_GetITStatus(TIM5, TIM_IT_CC4) != RESET){
		float Timestamp4 = TIM_GetCapture4(TIM5);
		App::mApp->mInputCapture->Period4 = Timestamp4 - App::mApp->mInputCapture->PrevTimestamp4 + (App::mApp->mInputCapture->OVFCount - App::mApp->mInputCapture->PrevOVFCount4) * 65536;
		App::mApp->mInputCapture->Period4 /= App::mApp->mInputCapture->Conf->TimerClk;
		App::mApp->mInputCapture->PrevTimestamp4 = Timestamp4;
		App::mApp->mInputCapture->PrevOVFCount4 = App::mApp->mInputCapture->OVFCount;
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC4);
	}
}
