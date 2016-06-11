/*
 * I2C.h
 *
 *  Created on: 2014¦~8¤ë18¤é
 *      Author: YunKei
 */

#ifndef I2C_H_
#define I2C_H_

#include <inttypes.h>
#include <stm32f4xx.h>
#include <stm32f4xx_i2c.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <Configuration.h>

using namespace System;

namespace Communication{

	class I2C{

		public:
			class I2CConfiguration{
				public:
					enum CLOCK{SPEED_100K = 100000, SPEED_400K = 400000};
					I2CConfiguration(I2C_TypeDef* I2Cx, Configuration* scl, uint8_t sclSource, Configuration* sdal, uint8_t sdaSource, CLOCK clock);
					I2C_TypeDef* _I2Cx;
					Configuration* _scl;
					Configuration* _sda;
					uint8_t _sclSource;
					uint8_t _sdaSource;
					CLOCK _clock;
			};
			I2C(I2CConfiguration* conf);
			bool Write(uint8_t, uint8_t, uint8_t);
			uint8_t Read(uint8_t, uint8_t);
			bool BurstWrite(uint8_t, uint8_t, uint8_t, uint8_t*);
			bool BurstRead(uint8_t, uint8_t, uint8_t, uint8_t*);
			int ErrorCount;
		private:
			I2CConfiguration* Conf;
			void ResetI2C();
	};
};

#endif /* I2C_H_ */
