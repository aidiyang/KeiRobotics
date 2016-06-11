/*
 * Configuration.h
 *
 *  Created on: 2015¦~11¤ë27¤é
 *      Author: wongy
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <inttypes.h>

namespace System{
	class Configuration{
		public:
			Configuration(GPIO_TypeDef* port, uint16_t pin);
			uint32_t _rcc;
			GPIO_TypeDef* _port;
			uint16_t _pin;
		private:
	};
};

#endif /* CONFIGURATION_H_ */
