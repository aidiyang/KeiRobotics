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
#include <App.h>

using namespace System;

namespace Sensors{

	class ADCConfiguration{
		public:
			enum ADCMode{FreeRun, OneShot};
			ADCConfiguration(Configuration* adc, uint8_t ADCChannel, uint8_t ADCCycles, ADCMode mode);
			Configuration* _adc;
			uint8_t _ADCChannel;
			uint8_t _ADCCycles;
			ADCMode Mode;
		private:
	};

	class ADConverter{
		public:
			ADConverter(ADCConfiguration* conf);
			double getReading();
		private:
			ADCConfiguration* Conf;
			uint16_t _ADCData;
	};
};

#endif /* ADCONVERTER_H_ */
