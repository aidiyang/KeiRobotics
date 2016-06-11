/*
 * ADConverter.h
 *
 *  Created on: 2015¦~12¤ë7¤é
 *      Author: wongy
 */

#ifndef ADCONVERTER_H_
#define ADCONVERTER_H_

#include <Configuration.h>
#include <stm32f4xx_adc.h>
#include <stm32f4xx_dma.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <inttypes.h>

using namespace System;

namespace Sensors{
	class ADConverter{
		public:
			class ADCConfiguration{
				public:
					ADCConfiguration(Configuration* adc, uint8_t ADCChannel, uint8_t ADCCycles);
					Configuration* _adc;
					uint8_t _ADCChannel;
					uint8_t _ADCCycles;
				private:
			};
			ADConverter(ADCConfiguration* conf);
			double getReading();
		private:
			ADCConfiguration* Conf;
			uint16_t _ADCData;
	};
};

#endif /* ADCONVERTER_H_ */
