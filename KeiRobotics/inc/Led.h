/*
 * Led.h
 *
 *  Created on: 2014¦~8¤ë3¤é
 *      Author: YunKei
 */

#ifndef LED_H_
#define LED_H_

#include <inttypes.h>
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <Configuration.h>

using namespace System;

namespace Debug{

	class Led{

		public:
			class LedConfiguration{
				public:
					LedConfiguration(Configuration* led, BitAction onState);
					Configuration* _led;
					BitAction _onState;
					BitAction _offState;
				private:
			};
			Led(LedConfiguration* conf);
			void LedControl(bool onState);
			void Toggle();
			void Blink(bool onState, uint16_t period, int count = -1);
		private:
			LedConfiguration* Conf;
	};
};

#endif /* LED_H_ */
