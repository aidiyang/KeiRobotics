/*
 * Configuration.cpp
 *
 *  Created on: 2015¦~11¤ë27¤é
 *      Author: wongy
 */

#include <Configuration.h>

using namespace System;

Configuration::Configuration(GPIO_TypeDef* port, uint16_t pin) : _port(port), _pin(pin){
	if(port == GPIOA){
		_rcc = RCC_AHB1Periph_GPIOA;
	}
	else if(port == GPIOB){
		_rcc = RCC_AHB1Periph_GPIOB;
	}
	else if(port == GPIOC){
		_rcc = RCC_AHB1Periph_GPIOC;
	}
	else if(port == GPIOD){
		_rcc = RCC_AHB1Periph_GPIOD;
	}
	else if(port == GPIOE){
		_rcc = RCC_AHB1Periph_GPIOE;
	}
	else if(port == GPIOF){
		_rcc = RCC_AHB1Periph_GPIOF;
	}
	else if(port == GPIOG){
		_rcc = RCC_AHB1Periph_GPIOG;
	}
	else if(port == GPIOH){
		_rcc = RCC_AHB1Periph_GPIOH;
	}
	else if(port == GPIOI){
		_rcc = RCC_AHB1Periph_GPIOI;
	}
	else if(port == GPIOJ){
		_rcc = RCC_AHB1Periph_GPIOJ;
	}
	else if(port == GPIOK){
		_rcc = RCC_AHB1Periph_GPIOK;
	}
}

