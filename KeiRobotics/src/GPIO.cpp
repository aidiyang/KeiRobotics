/*
 * GPIO.cpp
 *
 *  Created on: 2016¦~6¤ë7¤é
 *      Author: wongy
 */

#include <GPIO.h>
#include <App.h>
#include <Config.h>
#include <Task.h>
#include <stdio.h>
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <string>
#include <cstdlib>
#include <Delay.h>

using namespace System;
using namespace std;

GPIO::GPIOConfiguration::GPIOConfiguration(Configuration* gpio, BitAction onState) : _gpio(gpio), _onState(onState), _offState(onState ? Bit_RESET : Bit_SET){
};

BlinkObj::BlinkObj(GPIO* pGPIO){
	Conf = pGPIO->Conf;
	uint16_t num = rand() % 1000 + 1000;
	snprintf(addrStr, 4, "%d", num);
	addrStr[5] = '\n';
}

GPIO::GPIO(GPIOConfiguration* conf) : Conf(conf){
	GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(conf->_gpio->_rcc, ENABLE);
	GPIO_InitStructure.GPIO_Pin = conf->_gpio->_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(conf->_gpio->_port, &GPIO_InitStructure);

	GPIOControl(false);
}

void GPIO::GPIOControl(bool onState){
	GPIO_WriteBit(Conf->_gpio->_port, Conf->_gpio->_pin, onState ? Conf->_onState : Conf->_offState);
}

void GPIO::BlinkTask(Bundle* bundle){
	for(int i = 0; i < bundle->BlinkObjNum; i++){
		if(bundle->mTaskObj->TaskName.compare(bundle->mBlinkObj[i]->addrStr) == 0){
			GPIO_ToggleBits(bundle->mBlinkObj[i]->Conf->_gpio->_port, bundle->mBlinkObj[i]->Conf->_gpio->_pin);
		}
	}
}

void GPIO::Toggle(){
	GPIO_ToggleBits(Conf->_gpio->_port, Conf->_gpio->_pin);
}

void GPIO::Blink(GPIO* pGPIO, bool onState, uint16_t period, int count){

	if(onState){
		Task::mBundle->mBlinkObj[Task::mBundle->BlinkObjNum] = new BlinkObj(pGPIO);
		if(count >= 0){
			App::mApp->mTask->Attach(period / 2, BlinkTask, Task::mBundle->mBlinkObj[Task::mBundle->BlinkObjNum]->addrStr, false, 2 * count);
		}
		else{
			App::mApp->mTask->Attach(period / 2, BlinkTask, Task::mBundle->mBlinkObj[Task::mBundle->BlinkObjNum]->addrStr, true);
		}
		Task::mBundle->BlinkObjNum++;
	}
	else{
		Task::mBundle->BlinkObjNum--;
		App::mApp->mTask->DeAttach(Task::mBundle->mBlinkObj[Task::mBundle->BlinkObjNum]->addrStr);
		delete Task::mBundle->mBlinkObj[Task::mBundle->BlinkObjNum];
	}
}


