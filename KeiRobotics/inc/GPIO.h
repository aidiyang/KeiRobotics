/*
 * GPIO.h
 *
 *  Created on: 2016¦~6¤ë7¤é
 *      Author: wongy
 */

#ifndef GPIO_H_
#define GPIO_H_

#include <inttypes.h>
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <Configuration.h>
#include <string>
#include <Bundle.h>

using namespace System;
using namespace std;

namespace System{
	class Configuration;
	class Bundle;
}

namespace System{

	class GPIO{

		public:
			class GPIOConfiguration{
				public:
					GPIOConfiguration(Configuration* gpio, BitAction onState);
					Configuration* _gpio;
					BitAction _onState;
					BitAction _offState;
				private:
			};
			typedef void (*pBlink)();
			GPIO(GPIOConfiguration* conf);
			void GPIOControl(bool onState);
			void Toggle();
			static void BlinkTask(Bundle* bundle);
			void Blink(GPIO* pGPIO, bool onState, uint16_t period, int count = -1);
			GPIOConfiguration* Conf;
		private:
	};

	class BlinkObj{
		public:
			BlinkObj(GPIO* pGPIO);
			GPIO::GPIOConfiguration* Conf;
			char addrStr[5];
	};
};

#endif /* GPIO_H_ */
