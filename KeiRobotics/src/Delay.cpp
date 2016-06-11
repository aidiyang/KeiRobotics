/*
 * Delay.cpp
 *
 *  Created on: 2014¦~8¤ë3¤é
 *      Author: YunKei
 */

#include <Delay.h>

using namespace Time;

void Delay::Simple1USDelay(){

	__asm__ __volatile__("MOV	R0,#42\n\t"
						"DELAY:"
						"SUB	R0,#1\n\t"
						"CMP	R0,#0\n\t"
						"BNE	DELAY");
}

void Delay::DelayUS(uint16_t nus){

	for(uint16_t i = 0; i < nus; i++){
		Simple1USDelay();
	}
}

void Delay::DelayMS(uint16_t nms){

	for(uint16_t i = 0; i < nms; i++){
		DelayUS(1000);
	}
}

