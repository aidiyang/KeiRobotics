/*
 * Ticks.cpp
 *
 *  Created on: 2014¦~8¤ë3¤é
 *      Author: YunKei
 */

#include <App.h>
#include <Ticks.h>
#include <stm32f4xx.h>
#include <stm32f4xx_iwdg.h>
#include <stdio.h>
#include <Task.h>
#include <Communicating.h>
#include <Controlling.h>
#include <Delay.h>

uint16_t Ticks::maxTicks = 60000;

void SysTick_Handler(void){
	App::mApp->mTicks->TicksIncrement();
	if(App::mApp->mTicks->getTicks() >= Ticks::maxTicks){
		App::mApp->mTicks->setTicks(0);
	}

	if(App::mApp->mTicks->getTicks() % 1000 == 0){
		App::mTicks->Sec++;
	}

	if(App::mApp->mTask != 0 && App::mApp->mTask->IsPrintTaskNum){
		if(App::mApp->mTask->mTaskObj[App::mApp->mTask->currentTaskNum]->hangCount++ > App::mApp->mTask->mTaskObj[App::mApp->mTask->currentTaskNum]->TaskPeriod){
			static int delayCount = 0;
			if(delayCount++ > 10){
				delayCount = 0;
				printf("%s:%d\r\n", App::mApp->mTask->mTaskObj[App::mApp->mTask->currentTaskNum]->TaskName.c_str(), App::mApp->mTask->mTaskObj[App::mApp->mTask->currentTaskNum]->duration[1] - App::mApp->mTask->mTaskObj[App::mApp->mTask->currentTaskNum]->duration[0]);
			}
		}
	}
}

uint16_t Ticks::getTimeout(){

	if(getTicks() >= timeoutStartTimestamp){
		return getTicks() - timeoutStartTimestamp;
	}
	else{
		return maxTicks - timeoutStartTimestamp + getTicks();
	}
}

Ticks::Ticks(bool onWatchDog) : Sec(0), ticks(0), timeoutCount(0), timeoutStartTimestamp(0), OnWatchDog(onWatchDog){

	SysTick_Config(168000);
	if(onWatchDog){
		IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
		IWDG_SetPrescaler(IWDG_Prescaler_256);
		IWDG_SetReload(250);
		IWDG_ReloadCounter();
		IWDG_Enable();
	}
}

bool Ticks::TicksComp(uint16_t period, uint16_t ticksImg){

	if(ticksImg % period == 0){
		return true;
	}
	else{
		return false;
	}
}

void Ticks::TicksIncrement(){
	ticks++;
}

void Ticks::setTicks(uint16_t setValue){
	ticks = setValue;
}

uint16_t Ticks::getTicks(){
	return ticks;
}

bool Ticks::Timeout(){
	return (((int)timeoutCount - (int)getTimeout()) < 0) ? true : false;
}

void Ticks::setTimeout(uint16_t value){
	timeoutCount = value;
	timeoutStartTimestamp = getTicks();
}

void Ticks::PrintTime(){
	int hr = Sec / 3600;
	int min = (Sec - hr * 3600) / 60;
	int sec = (Sec - hr * 3600 - min * 60);
	printf("Time Executed:%d:%d:%d\r\n", hr, min, sec);
}
