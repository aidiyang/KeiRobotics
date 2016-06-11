/*
 * ExternalInterrupt.h
 *
 *  Created on: 2016¦~4¤ë9¤é
 *      Author: wongy
 */

#ifndef EXTERNALINTERRUPT_H_
#define EXTERNALINTERRUPT_H_

#include <stm32f4xx.h>
#include <stm32f4xx_exti.h>
#include <Configuration.h>

namespace System{
	class ExternalInterrupt{
		public:
			typedef void (*pInterruptTask)();
			enum Mode{RISING,FALLING,RISING_FALLING};
			ExternalInterrupt(Configuration* pin, Mode mode, pInterruptTask pTask);
			pInterruptTask mInterruptTask;
			Configuration* _pin;
			Mode _mode;
			uint8_t EXTI_PortSourceGPIO;
		    static uint32_t EXTI_Line;
		    uint8_t EXTI_PinSource;

		private:
	};
};

#endif /* EXTERNALINTERRUPT_H_ */
